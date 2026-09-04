# Copyright 2026 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Node to control OpenArm."""

import argparse
import dataclasses
import enum
import numbers
import dora
import openarm_driver
import os
import pathlib
import pyarrow as pa
import numpy as np


class ArmStatus(str, enum.Enum):
    """Arm control states."""

    STOPPED = "stopped"
    STARTED = "started"
    ALIGNED = "aligned"


@dataclasses.dataclass
class AlignState:
    """State for alignment."""

    align_target: np.ndarray = None
    step_limit: float = 0.001


def _align(arm, state, new_position, name, threshold, send_position, trigger=None):
    """Safety: Align OpenArm with the position."""
    if trigger == "gripper":  # Check if gripper is active (threshold ~ -10 deg)
        gripper_position = new_position[-1]  # Last value is gripper's position
        if name == "right_arm":
            is_gripping = gripper_position > np.deg2rad(-5)
        elif name == "left_arm":
            is_gripping = gripper_position < np.deg2rad(5)
        if not is_gripping:
            return False

    current_position = np.array(arm.fetch_position(), dtype=np.float32)

    if state.align_target is None:
        state.align_target = current_position.copy()

    def is_aligned(position1, position2):
        return np.all(np.abs(position1[:-1] - position2[:-1]) < threshold)

    # Commit the final target before reporting alignment complete.
    if is_aligned(new_position, current_position):
        return send_position(new_position)
    diff = new_position - state.align_target
    step_move = np.clip(diff, -state.step_limit, state.step_limit)
    state.align_target += step_move

    send_position(state.align_target)

    # Check the physical position on the next command after the arm has moved.
    return False


def _env_flag(name, default=False):
    v = os.getenv(name)
    if v is None:
        return default
    return v.strip().lower() in ("1", "true", "yes", "on")


QPOS_TYPE = pa.struct([("qpos", pa.list_(pa.float32()))])

STATE_TYPE = pa.struct(
    [
        ("qpos", pa.list_(pa.float32())),
        ("qvel", pa.list_(pa.float32())),
        ("qtorque", pa.list_(pa.float32())),
        ("tmos", pa.list_(pa.int32())),
        ("trotor", pa.list_(pa.int32())),
    ]
)


def build_qpos_output(qpos: np.ndarray) -> pa.Array:
    """Wrap a qpos array as a length-1 StructArray: [{"qpos": [...]}]."""
    return pa.array([{"qpos": qpos}], type=QPOS_TYPE)


def build_state_output(state) -> pa.Array:
    """Wrap a state dict as a length-1 StructArray: [{"qpos": [...], ...}]."""
    return pa.array(
        [
            {
                "qpos": state["qpos"],
                "qvel": state["qvel"],
                "qtorque": state["qtorque"],
                "tmos": state["tmos"],
                "trotor": state["trotor"],
            }
        ],
        type=STATE_TYPE,
    )


def extract_values(value: pa.Array, key: str) -> np.ndarray:
    """Read `key` from a length-1 StructArray, or a flat array as-is."""
    if pa.types.is_struct(value.type):
        value = value.field(key)[0].values
    return np.array(value, dtype=np.float32)


def command_epoch_matches(metadata: dict, start_epoch: int) -> bool:
    """Accept legacy commands or commands addressed to the current start."""
    if "start_epoch" not in metadata:
        return True
    value = metadata["start_epoch"]
    return (
        isinstance(value, numbers.Integral)
        and not isinstance(value, bool)
        and int(value) == start_epoch
    )


def startup_command_metadata(arm) -> dict | None:
    """Describe the final command dispatched by the driver's start trajectory."""
    executed_timestamp = arm.last_command_executed_timestamp_ns
    if executed_timestamp is None:
        return None
    return {"timestamp": executed_timestamp}


def main():
    """Move to the given position and output the current position."""
    parser = argparse.ArgumentParser(description="Control OpenArm")
    parser.add_argument(
        "--side",
        choices=["right", "left"],
        default="right",
        help="right or left",
    )
    parser.add_argument(
        "--config",
        default=None,
        help="The configuration file for this OpenArm",
        type=pathlib.Path,
    )
    parser.add_argument(
        "--align-trigger",
        choices=["gripper"],
        default=None,
        help="Alignment trigger: gripper (default: None)",
    )
    parser.add_argument(
        "--align-threshold",
        default=0.1,
        help="Alignment threshold [rad] (default: 0.1)",
        type=float,
    )
    parser.add_argument(
        "--align-delta-limit",
        default=0.001,
        help="Maximum joint delta per alignment command [rad] (default: 0.001).",
        type=float,
    )
    parser.add_argument(
        "--align",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Align to incoming position commands after start (default: enabled).",
    )
    parser.add_argument(
        "--stop",
        action=argparse.BooleanOptionalAction,
        default=_env_flag("STOP", True),
        help="Stop the arm on exit.",
    )
    parser.add_argument(
        "--refresh-every-request",
        action=argparse.BooleanOptionalAction,
        default=_env_flag("REFRESH", True),
        help="Refresh OpenArm on every request to make it more accurate.",
    )
    parser.add_argument(
        "--start-on-startup",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Start the arm on startup.",
    )
    args = parser.parse_args()
    if args.align_delta_limit <= 0.0:
        parser.error("--align-delta-limit must be positive")
    node = dora.Node()
    name = f"{args.side}_arm"
    config = openarm_driver.Config(args.config)
    align_threshold = args.align_threshold
    arm = None
    start_epoch = 0
    latest_command_metadata = None
    ready_status = ArmStatus.ALIGNED if args.align else ArmStatus.STARTED

    def output_metadata(metadata: dict | None = None) -> dict:
        result = dict(metadata or {})
        result["start_epoch"] = start_epoch
        return result

    if args.start_on_startup:
        arm = openarm_driver.SingleArmDriver(name, config)
        arm.start()
        start_epoch += 1
        latest_command_metadata = startup_command_metadata(arm)
        align_state = (
            AlignState(step_limit=args.align_delta_limit) if args.align else None
        )
        status = ArmStatus.STARTED
        node.send_output("status", pa.array([status]), output_metadata())
    else:
        align_state = None
        status = ArmStatus.STOPPED
        node.send_output("status", pa.array([ArmStatus.STOPPED]), output_metadata())

    def send_latest_command() -> None:
        """Publish the last command that the driver actually accepted."""
        if arm is None or latest_command_metadata is None:
            return
        executed_timestamp = arm.last_command_executed_timestamp_ns
        if executed_timestamp is None:
            return
        metadata = output_metadata(latest_command_metadata)
        metadata["executed_timestamp"] = executed_timestamp
        node.send_output(
            "latest_command",
            build_qpos_output(np.asarray(arm.last_command, dtype=np.float32)),
            metadata,
        )

    def send_position(position: np.ndarray, metadata: dict) -> bool:
        """Send one checked command and publish the final driver target."""
        nonlocal latest_command_metadata
        if not arm.send_position(position):
            return False
        executed_timestamp = arm.last_command_executed_timestamp_ns
        if executed_timestamp is None:
            raise RuntimeError(
                "driver accepted a command without an executed timestamp"
            )
        latest_command_metadata = dict(metadata)
        send_latest_command()
        return True

    for event in node:
        if event["type"] != "INPUT":
            continue

        event_id = event["id"]
        if event_id == "command":
            command = event["value"][0].as_py()
            if command == "start":
                if arm is not None:
                    arm.stop()  # Stop the existing session before replacing it
                latest_command_metadata = None
                arm = openarm_driver.SingleArmDriver(
                    name, config
                )  # Re-initialize the arm to ensure a fresh start
                arm.start()
                start_epoch += 1
                latest_command_metadata = startup_command_metadata(arm)
                align_state = (
                    AlignState(step_limit=args.align_delta_limit)
                    if args.align
                    else None
                )
                status = ArmStatus.STARTED
                node.send_output(
                    "status",
                    pa.array([status]),
                    output_metadata(event["metadata"]),
                )
            elif command == "stop":
                status = ArmStatus.STOPPED
                node.send_output(
                    "status",
                    pa.array([ArmStatus.STOPPED]),
                    output_metadata(event["metadata"]),
                )
                if arm is not None:
                    arm.stop()
                    arm = None  # Drop the instance to free resources
                latest_command_metadata = None
                align_state = None
        elif event_id == "request_position":
            if status is ArmStatus.STOPPED:
                continue
            current_position = arm.fetch_position(
                refresh=args.refresh_every_request,
            )
            node.send_output(
                "position",
                build_qpos_output(np.asarray(current_position, dtype=np.float32)),
                output_metadata(event["metadata"]),
            )
        elif event_id == "request_state":
            if status is ArmStatus.STOPPED:
                continue
            state = arm.fetch_state(refresh=args.refresh_every_request)
            metadata = output_metadata(event["metadata"])
            node.send_output("state", build_state_output(state), metadata)
            node.send_output(
                "position",
                build_qpos_output(np.asarray(state["qpos"], dtype=np.float32)),
                metadata,
            )
            send_latest_command()
        elif event_id == "move_position":
            if status is ArmStatus.STOPPED:
                continue
            if not command_epoch_matches(event["metadata"], start_epoch):
                print(
                    "Ignoring move_position with malformed or stale "
                    f"start_epoch: {event['metadata'].get('start_epoch')!r} "
                    f"(current={start_epoch})",
                    flush=True,
                )
                continue
            value = event["value"]
            if isinstance(value, pa.StructArray):
                names = value.type.names
                if "qpos" in names:
                    new_position = extract_values(value, "qpos")
                else:
                    new_position = np.array(
                        value.field("new_position"), dtype=np.float32
                    )
                # TODO: We use this for safety check later.
                # other_arm_position = value.field("other_arm_position")
            else:
                new_position = np.array(value, dtype=np.float32)
                # other_arm_position = None

            if status is ready_status:
                send_position(new_position, event["metadata"])
            elif status is ArmStatus.STARTED:
                is_aligned = _align(
                    arm,
                    align_state,
                    new_position,
                    name,
                    align_threshold,
                    lambda position: send_position(position, event["metadata"]),
                    trigger=args.align_trigger,
                )
                if is_aligned:
                    status = ArmStatus.ALIGNED
                    node.send_output(
                        "status",
                        pa.array([ArmStatus.ALIGNED]),
                        output_metadata(event["metadata"]),
                    )
    if arm is not None:
        if args.stop:
            arm.stop()
        else:
            arm.move_to_start_position()


if __name__ == "__main__":
    main()

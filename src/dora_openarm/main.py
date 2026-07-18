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
import enum
import dora
import openarm_driver
import os
import pathlib
import pyarrow as pa
import numpy as np
import time


DEFAULT_COMMAND_DT_S = 1.0 / 250.0
MAX_COMMAND_DT_S = 0.1


class ArmStatus(str, enum.Enum):
    """Arm control states."""

    STOPPED = "stopped"
    STARTED = "started"


class CommandShapeState:
    """State for velocity command shaping."""

    def __init__(
        self,
        position: np.ndarray | None = None,
        time_s: float | None = None,
    ):
        self.position = position
        self.time_s = time_s


def _load_command_limits(config) -> np.ndarray:
    """Load command velocity limits from the arm config."""
    velocity_limits = np.asarray(config.get_joint_delta_position_limits(), dtype=float)
    if np.any(velocity_limits <= 0.0):
        raise ValueError("Command velocity limits must be positive.")
    return velocity_limits


def _reset_shape_state(arm) -> CommandShapeState:
    position = np.asarray(arm.last_command, dtype=float)
    return CommandShapeState(
        position=position.copy(),
        time_s=time.monotonic(),
    )


def _shape_position(
    target: np.ndarray,
    state: CommandShapeState,
    velocity_limits: np.ndarray,
    default_dt_s: float,
) -> np.ndarray:
    """Apply per-command velocity limits without adding acceleration lag."""
    now_s = time.monotonic()
    target = np.asarray(target, dtype=float)
    if not np.all(np.isfinite(target)):
        raise RuntimeError("Received non-finite joint position command.")

    if state.position is None:
        state.position = target.copy()
    if target.shape != state.position.shape:
        raise ValueError(
            f"Command shape {target.shape} does not match arm shape {state.position.shape}."
        )
    if target.shape != velocity_limits.shape:
        raise ValueError(
            "Command shaping limits must have the same shape as the joint command."
        )

    dt_s = now_s - state.time_s if state.time_s is not None else default_dt_s
    if not np.isfinite(dt_s) or dt_s <= 0.0 or dt_s > MAX_COMMAND_DT_S:
        dt_s = default_dt_s

    delta = target - state.position
    max_delta = velocity_limits * dt_s
    shaped_position = state.position + np.clip(delta, -max_delta, max_delta)
    return shaped_position.astype(np.float32)


def _record_sent_command(
    arm,
    state: CommandShapeState,
) -> None:
    sent_position = np.asarray(arm.last_command, dtype=float)
    state.position = sent_position.copy()
    state.time_s = time.monotonic()


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
        help="Deprecated; command shaping is always active.",
    )
    parser.add_argument(
        "--align-threshold",
        default=0.1,
        help="Deprecated; kept for dataflow compatibility.",
        type=float,
    )
    parser.add_argument(
        "--control-hz",
        default=1.0 / DEFAULT_COMMAND_DT_S,
        help="Fallback command shaping rate in Hz (default: 250).",
        type=float,
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
    node = dora.Node()
    name = f"{args.side}_arm"
    config = openarm_driver.Config(args.config)
    velocity_limits = _load_command_limits(config)
    if args.control_hz <= 0.0:
        raise ValueError("--control-hz must be positive.")
    default_dt_s = 1.0 / args.control_hz
    arm = None
    shape_state = None
    if args.start_on_startup:
        arm = openarm_driver.SingleArmDriver(name, config)
        arm.start()
        shape_state = _reset_shape_state(arm)
        status = ArmStatus.STARTED
        node.send_output("status", pa.array([ArmStatus.STARTED]))
    else:
        status = ArmStatus.STOPPED
        node.send_output("status", pa.array([ArmStatus.STOPPED]))
    for event in node:
        if event["type"] != "INPUT":
            continue

        event_id = event["id"]
        if event_id == "command":
            command = event["value"][0].as_py()
            if command == "start":
                if arm is not None:
                    arm.stop()  # Stop the existing session before replacing it
                arm = openarm_driver.SingleArmDriver(
                    name, config
                )  # Re-initialize the arm to ensure a fresh start
                arm.start()
                shape_state = _reset_shape_state(arm)
                status = ArmStatus.STARTED
                node.send_output("status", pa.array([ArmStatus.STARTED]))
            elif command == "stop":
                status = ArmStatus.STOPPED
                node.send_output("status", pa.array([ArmStatus.STOPPED]))
                if arm is not None:
                    arm.stop()
                    arm = None  # Drop the instance to free resources
                shape_state = None
        elif event_id == "request_position":
            if status is ArmStatus.STOPPED:
                continue
            current_position = arm.fetch_position(
                refresh=args.refresh_every_request,
            )
            node.send_output(
                "position",
                build_qpos_output(np.asarray(current_position, dtype=np.float32)),
            )
        elif event_id == "request_state":
            if status is ArmStatus.STOPPED:
                continue
            state = arm.fetch_state(refresh=args.refresh_every_request)
            node.send_output("state", build_state_output(state))
        elif event_id == "move_position":
            if status is ArmStatus.STOPPED:
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

            if shape_state is None:
                shape_state = _reset_shape_state(arm)
            try:
                shaped_position = _shape_position(
                    new_position,
                    shape_state,
                    velocity_limits,
                    default_dt_s,
                )
                arm.send_position(shaped_position)
            except (RuntimeError, ValueError) as exc:
                print(f"[safety] {exc}")
                arm.stop()
                arm = None
                shape_state = None
                status = ArmStatus.STOPPED
                node.send_output("status", pa.array([ArmStatus.STOPPED]))
                continue
            _record_sent_command(arm, shape_state)
    if arm is not None:
        if args.stop:
            arm.stop()
        else:
            arm.move_to_start_position()


if __name__ == "__main__":
    main()

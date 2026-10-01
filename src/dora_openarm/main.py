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
import dora
import openarm_driver
import os
import pathlib
import time
import traceback
import pyarrow as pa
import numpy as np


class ArmStatus(str, enum.Enum):
    """Arm control states."""

    STOPPED = "stopped"
    ERROR = "error"
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

    # Accept the final target before reporting alignment complete.
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

# Counters openarm_can keeps per interface. They latch, so a consumer that
# wants "during this episode" has to difference them against a baseline of
# its own rather than expect them to fall back to zero.
BUS_TYPE = pa.struct(
    [
        # The only instantaneous field. A bus-off with no auto-restart shows
        # up here; one that auto-restarts only moves the counters.
        ("carrier", pa.bool_()),
        ("bus_off", pa.int64()),
        ("error_passive", pa.int64()),
        ("error_warning", pa.int64()),
        ("ack_error", pa.int64()),
        ("tx_overflow", pa.int64()),
        ("rx_overflow", pa.int64()),
        ("net_down", pa.int64()),
    ]
)

STATE_TYPE = pa.struct(
    [
        ("qpos", pa.list_(pa.float32())),
        ("qvel", pa.list_(pa.float32())),
        ("qtorque", pa.list_(pa.float32())),
        ("tmos", pa.list_(pa.int32())),
        ("trotor", pa.list_(pa.int32())),
        # One entry per motor, in the same order as qpos: the motor's own
        # status nibble by name, or "SILENT" when it has stopped answering.
        ("motor_status", pa.list_(pa.string())),
        ("bus", BUS_TYPE),
    ]
)


_EMPTY_BUS = {
    "carrier": True,
    "bus_off": 0,
    "error_passive": 0,
    "error_warning": 0,
    "ack_error": 0,
    "tx_overflow": 0,
    "rx_overflow": 0,
    "net_down": 0,
}


def build_qpos_output(qpos: np.ndarray) -> pa.Array:
    """Wrap a qpos array as a length-1 StructArray: [{"qpos": [...]}]."""
    return pa.array([{"qpos": qpos}], type=QPOS_TYPE)


def _bus_snapshot(bus: dict) -> dict:
    """Narrow openarm_driver's get_health() bus dict to what `state` publishes.

    get_health() reports every counter openarm_can tracks; `state` only ever
    surfaced the subset an operator acts on, so this keeps that surface
    unchanged rather than growing STATE_TYPE (and every UI reading it) every
    time openarm_driver's own diagnostics grow.
    """
    if not bus:
        return dict(_EMPTY_BUS)
    return {
        "carrier": bus["carrier"],
        "bus_off": bus["bus_off"],
        "error_passive": bus["error_passive"],
        "error_warning": bus["error_warning"],
        "ack_error": bus["ack_error"],
        "tx_overflow": bus["tx_overflow"],
        "rx_overflow": bus["rx_overflow"],
        "net_down": bus["write_net_down"],
    }


def build_state_output(state, health: tuple[list[str], dict]) -> pa.Array:
    """Wrap a state dict as a length-1 StructArray: [{"qpos": [...], ...}].

    `health` is `arm.get_health()`'s return value, forwarded as-is: this
    node reports what openarm_driver already knows and does not read
    openarm_can itself.
    """
    motor_status, bus = health
    bus = _bus_snapshot(bus)
    return pa.array(
        [
            {
                "qpos": state["qpos"],
                "qvel": state["qvel"],
                "qtorque": state["qtorque"],
                "tmos": state["tmos"],
                "trotor": state["trotor"],
                "motor_status": motor_status,
                "bus": bus,
            }
        ],
        type=STATE_TYPE,
    )


def extract_values(value: pa.Array, key: str) -> np.ndarray:
    """Read `key` from a length-1 StructArray, or a flat array as-is."""
    if pa.types.is_struct(value.type):
        value = value.field(key)
        if pa.types.is_list(value.type):
            value = value[0].values
    return np.array(value, dtype=np.float32)


def command_epoch_matches(metadata: dict, start_epoch: int) -> bool:
    """Accept legacy commands or commands addressed to the current start."""
    if "start_epoch" not in metadata:
        return True
    value = metadata["start_epoch"]
    return (
        isinstance(value, int) and not isinstance(value, bool) and value == start_epoch
    )


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
        "--can-interface",
        default=None,
        help=(
            "SocketCAN interface, overriding the config. Which interface an "
            "arm is on is a property of the machine rather than of the arm, "
            "so a host that names them differently can be handled without "
            "editing the config (default: the config's)."
        ),
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
        help="Maximum intermediate alignment target step [rad] (default: 0.001).",
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
        help="Refresh OpenArm on every publish_tick.",
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
    session_metadata = {}
    align_state = None
    status = ArmStatus.STOPPED
    ready_status = ArmStatus.ALIGNED if args.align else ArmStatus.STARTED

    def output_metadata(metadata: dict | None = None) -> dict:
        result = dict(metadata or {})
        result["start_epoch"] = start_epoch
        return result

    node.send_output("status", pa.array([status]), output_metadata())

    def report_error(message: str) -> None:
        nonlocal status, align_state
        status = ArmStatus.ERROR
        align_state = None
        print(f"{name}: {message}", flush=True)
        metadata = output_metadata(session_metadata)
        metadata["message"] = message
        node.send_output("status", pa.array([status]), metadata)

    def send_position(position: np.ndarray, metadata: dict) -> bool:
        """Cache source metadata only after the driver accepts the target."""
        nonlocal latest_command_metadata
        accepted = arm.send_position(position)
        if accepted:
            latest_command_metadata = dict(metadata)
        else:
            report_error(arm.safety_stop_reason or "Position command rejected")
        return accepted

    clean_exit = False
    try:
        for event in node:
            if event["type"] == "STOP":
                break
            if event["type"] != "INPUT":
                continue
            event_id = event["id"]
            try:
                if event_id == "command":
                    command = event["value"][0].as_py()
                    if command == "start":
                        session_metadata = dict(event["metadata"])
                        if arm is not None:
                            arm.stop(run_trajectory=status is not ArmStatus.ERROR)
                            arm = None
                        latest_command_metadata = None
                        align_state = None
                        arm = openarm_driver.SingleArmDriver(
                            name, config, can_interface=args.can_interface
                        )
                        if not arm.start():
                            report_error(
                                arm.safety_stop_reason
                                or "Startup trajectory interrupted"
                            )
                            continue
                        start_epoch += 1
                        executed_timestamp = arm.last_command_dispatch_timestamp_ns
                        latest_command_metadata = (
                            {"timestamp": executed_timestamp}
                            if executed_timestamp is not None
                            else None
                        )
                        align_state = (
                            AlignState(step_limit=args.align_delta_limit)
                            if args.align
                            else None
                        )
                        status = ArmStatus.STARTED
                        node.send_output(
                            "status",
                            pa.array([status]),
                            output_metadata(session_metadata),
                        )
                    elif command == "stop":
                        if arm is not None:
                            arm.stop(run_trajectory=status is not ArmStatus.ERROR)
                        arm = None
                        latest_command_metadata = None
                        align_state = None
                        status = ArmStatus.STOPPED
                        node.send_output(
                            "status",
                            pa.array([status]),
                            output_metadata({**session_metadata, **event["metadata"]}),
                        )
                elif status not in (ArmStatus.STARTED, ArmStatus.ALIGNED):
                    continue
                elif event_id == "publish_tick":
                    state = arm.fetch_state(refresh=args.refresh_every_request)
                    health = arm.get_health()
                    metadata = output_metadata(event["metadata"])
                    metadata.pop("timestamp", None)
                    metadata["observation_timestamp"] = time.time_ns()
                    node.send_output(
                        "state", build_state_output(state, health), metadata
                    )
                    if latest_command_metadata is not None:
                        metadata = output_metadata(latest_command_metadata)
                        metadata["executed_timestamp"] = (
                            arm.last_command_dispatch_timestamp_ns
                        )
                        node.send_output(
                            "latest_command",
                            build_qpos_output(arm.last_command),
                            metadata,
                        )
                elif event_id == "move_position":
                    if not command_epoch_matches(event["metadata"], start_epoch):
                        print(
                            "Ignoring move_position with malformed or stale "
                            f"start_epoch: {event['metadata'].get('start_epoch')!r} "
                            f"(current={start_epoch})",
                            flush=True,
                        )
                        continue
                    value = event["value"]
                    key = (
                        "qpos"
                        if pa.types.is_struct(value.type) and "qpos" in value.type.names
                        else "new_position"
                    )
                    new_position = extract_values(value, key)
                    if status is ready_status:
                        send_position(new_position, event["metadata"])
                    elif _align(
                        arm,
                        align_state,
                        new_position,
                        name,
                        align_threshold,
                        lambda position: send_position(position, event["metadata"]),
                        trigger=args.align_trigger,
                    ):
                        status = ArmStatus.ALIGNED
                        node.send_output(
                            "status",
                            pa.array([status]),
                            output_metadata(event["metadata"]),
                        )
            except Exception as error:
                traceback.print_exc()
                report_error(f"{event_id} failed: {error}")

        if arm is not None and status is not ArmStatus.ERROR and not args.stop:
            if not arm.move_to_start_position():
                report_error(
                    arm.safety_stop_reason or "Shutdown trajectory interrupted"
                )
        clean_exit = True
    except Exception as error:
        report_error(f"Node exiting after failure: {error}")
        raise
    finally:
        if arm is not None and (
            args.stop or status is ArmStatus.ERROR or not clean_exit
        ):
            try:
                arm.stop(run_trajectory=clean_exit and status is not ArmStatus.ERROR)
            except Exception as error:
                traceback.print_exc()
                report_error(f"Shutdown failed: {error}")
                raise


if __name__ == "__main__":
    main()

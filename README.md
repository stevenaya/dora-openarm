# dora-openarm

A [Dora](https://dora-rs.ai/) node that controls OpenArm.

## Usage

Use this node from a dora-rs dataflow configuration. For a full configuration
example, see
[enactic/dora-openarm-data-collection](https://github.com/enactic/dora-openarm-data-collection).

```yaml
nodes:
  # ...
  - id: follower-right
    build: pip install dora-openarm
    path: dora-openarm
    args: "--side right --align-trigger gripper"
    inputs:
      # Only the event ID is used. The event value is ignored.
      publish_tick: dora/timer/millis/33
      command: ui/arm_command
      move_position: leader/right_follower_position
    outputs:
      - state
      - latest_command
      - status

  - id: follower-left
    build: pip install dora-openarm
    path: dora-openarm
    args: "--side left --align-trigger gripper"
    inputs:
      # Only the event ID is used. The event value is ignored.
      publish_tick: dora/timer/millis/33
      command: ui/arm_command
      move_position: leader/left_follower_position
    outputs:
      - state
      - latest_command
      - status
  # ...
```

### Node arguments

| Argument | Description |
| --- | --- |
| `--side` | OpenArm side to control. Default: `right`. |
| `--config` | Path to the OpenArm configuration file. Default: `openarm_cell.yaml`. |
| `--can-interface` | SocketCAN interface to use, overriding the one in the configuration file. Default: the configuration file's value. |
| `--align-trigger` | Optional trigger for the initial alignment step. Supported value: `gripper`. |
| `--align-threshold` | Alignment threshold in radians. Default: `0.1`. |
| `--align-delta-limit` | Maximum joint delta per initial-alignment command in radians. Default: `0.001`. |
| `--[no-]align` | Whether to align to incoming position commands after the arm starts. Default: enabled. |
| `--[no-]stop` | Whether to stop the arm when the node exits. Default: controlled by the `STOP` environment variable, or `true` when it is unset. |
| `--[no-]refresh-every-request` | Whether to refresh OpenArm state on each `publish_tick`. Default: controlled by the `REFRESH` environment variable, or `true` when it is unset. |

### Inputs

| Input | Description |
| --- | --- |
| `publish_tick` | Reads one state snapshot and publishes `state` plus the cached `latest_command`, if present. Ignores the event value. |
| `command` | Explicit `start` / `stop`; node startup does not create a driver or enable motors. |
| `move_position` | Sends a new target position to the arm. The value may be a struct containing `qpos` (`[{"qpos": [...]}]`), a position array directly, or a legacy struct containing `new_position`. When initial alignment is enabled, this input drives the alignment until it completes. |

### Outputs

| Output | Description |
| --- | --- |
| `state` | Current arm state as a length-1 struct: `[{"qpos": [...], "qvel": [...], "qtorque": [...], "tmos": [...], "trotor": [...], "motor_status": [...], "bus": {...}}]`. `qpos`, `qvel`, and `qtorque` are float32 lists; `tmos` (MOS temperature) and `trotor` (rotor temperature) are int32 lists per motor, in °C. `motor_status` is a string list with one entry per motor in `qpos` order: the motor's own status name, or `SILENT` if it has stopped answering. `bus` is a struct describing the CAN interface: `carrier` (bool, whether the link is up) and the cumulative fault counters `bus_off`, `error_passive`, `error_warning`, `ack_error`, `tx_overflow`, `rx_overflow`, and `net_down` (int64). The counters only grow while the node runs, so compare against a baseline to see what happened during a given period. |
| `status` | Current control state as a string array: `stopped`, `started`, `aligned`, or `error` (with `metadata.message`). With alignment enabled, `aligned` is emitted once initial alignment completes. |
| `latest_command` | A length-1 `qpos` struct with the final driver-accepted target after safety clamping. Published on `publish_tick`, preserving the source `timestamp` and adding the driver's `executed_timestamp` in wall-clock nanoseconds and the current `start_epoch`. |

Each `publish_tick` reads hardware once. Consumers needing only position extract
`qpos` from `state`; there is no separate `position` output. Each state includes `observation_timestamp`: integer Unix
wall-clock nanoseconds captured after the snapshot is read. Request metadata is
preserved except for `timestamp`, which is removed so Dora supplies the output
message timestamp.

Every output includes a process-local `start_epoch`, starting at zero and
incrementing only after each successful explicit start.
Commands with an epoch must supply a non-boolean integer matching the current
session; malformed or mismatched epochs are ignored. Commands without an epoch
remain accepted for compatibility. The epoch resets on process restart and
does not detect reordered commands within a session.

Connect only `publish_tick` to the sampling timer. `move_position` sends the
target and updates the command cache; the tick never resends commands. Multiple
accepted commands between ticks are represented by the latest one only.

Repeated command snapshots preserve their original timestamps. The startup
trajectory's final dispatched command uses its dispatch time for both
timestamps; if startup dispatches nothing, the command cache starts empty.
Stop/start clears the previous session's cache. Rejected commands leave the
cache unchanged, and an alignment-completing command must be accepted before
the node publishes `aligned`. A command snapshot reports software dispatch,
not motor acknowledgement or physical arrival at the target.

### Startup and errors

Requires openarm-driver >= 0.5.1. A successful startup emits `started` with the
start request metadata. It confirms completion of the startup dispatch sequence,
not physical arrival. Optional `aligned` follows subsequent move commands.
A rejected startup emits `error`, not `started`, and does not advance the epoch.
Command/state-read errors also enter `error` and block further moves and sampling
until an explicit stop/start. The driver instance is retained: failed startup
or safety rejection does not imply motors are disabled.

`stop` emits `stopped` only after the driver's stop completes. A failed stop keeps
the instance and reports `error`. There is no automatic retry or recovery. Normal
stop may run a return trajectory; a latched safety stop skips it before disabling.
On error, exit also uses stop even with `--no-stop`. Stop is not a hardware E-stop.

More detail: [command lifecycle and safety](docs/command-lifecycle-and-safety.md).

## License

Licensed under the Apache License 2.0. See [LICENSE](LICENSE) for details.

Copyright 2026 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our [Code of Conduct](CODE_OF_CONDUCT.md).

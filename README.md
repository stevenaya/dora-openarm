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
| `--align-delta-limit` | Maximum intermediate alignment target step in radians. Default: `0.001`. The final target is submitted directly once within the alignment threshold, subject to driver safety checks. |
| `--[no-]align` | Whether to align to incoming position commands after the arm starts. Default: enabled. |
| `--[no-]stop` | Whether to stop the arm on normal exit. With `--no-stop`, run the configured startup trajectory instead and leave motors enabled on success. Faults and interrupted exits disable without a trajectory. Default: `STOP`, or `true` when unset. |
| `--[no-]refresh-every-request` | Whether to refresh OpenArm state on each `publish_tick`. Default: `REFRESH`, or `true` when unset. |

### Inputs

| Input | Description |
| --- | --- |
| `publish_tick` | Reads state once and publishes `state` and the cached `latest_command`, if present. The event value is ignored. |
| `command` | Explicit `start` / `stop`. Node startup does not create a driver or enable motors. |
| `move_position` | Sends a new target position to the arm. The value may be a struct containing `qpos` (`[{"qpos": [...]}]`), a position array directly, or a legacy struct containing `new_position`. When initial alignment is enabled, this input drives the alignment until it completes. |

### Outputs

| Output | Description |
| --- | --- |
| `state` | Current arm state as a length-1 struct: `[{"qpos": [...], "qvel": [...], "qtorque": [...], "tmos": [...], "trotor": [...], "motor_status": [...], "bus": {...}}]`. `qpos`, `qvel`, and `qtorque` are float32 lists; `tmos` (MOS temperature) and `trotor` (rotor temperature) are int32 lists per motor, in °C. `motor_status` is a string list with one entry per motor in `qpos` order: the motor's own status name, or `SILENT` if it has stopped answering. `bus` is a struct describing the CAN interface: `carrier` (bool, whether the link is up) and the cumulative fault counters `bus_off`, `error_passive`, `error_warning`, `ack_error`, `tx_overflow`, `rx_overflow`, and `net_down` (int64). The counters only grow while the node runs, so compare against a baseline to see what happened during a given period. |
| `status` | A string array: `stopped`, `started`, `aligned`, or `error`. Errors include `metadata.message`. |
| `latest_command` | A length-1 `qpos` struct with the latest driver-accepted target after safety clamping. Preserves source metadata and adds `executed_timestamp`, the driver's software dispatch time in Unix nanoseconds. This is not measured position or confirmation of physical arrival. |

Each `publish_tick` reads state once; position-only consumers use `state.qpos`.
Each observation includes `observation_timestamp`: integer Unix
wall-clock nanoseconds captured after the snapshot is read. Request metadata is
preserved except for `timestamp`, which is removed so Dora supplies the output
message timestamp.

Every output includes a process-local `start_epoch`, starting at zero and
incrementing only after each successful explicit start.
Commands with an epoch must supply a non-boolean integer matching the current
session; malformed or mismatched epochs are ignored. Commands without an epoch
remain accepted for compatibility. The epoch resets on process restart and
does not detect reordered commands within a session.

### Commands and errors

Requires the matching `openarm-driver >= 0.5.2` with `stop(run_trajectory=...)`.
`move_position` sends a target and caches its metadata only on acceptance.
Ticks never resend targets. Several accepted commands between ticks produce
only the latest snapshot, not a complete dispatch log. Repeated snapshots keep
their original times and chunk metadata. Startup's final dispatched target uses
its dispatch time as the source `timestamp`; an empty startup trajectory produces
no command snapshot. A new session clears the old cache.

`started` and `aligned` are published only after driver acceptance; `stopped`
only after stop succeeds. Intermediate alignment steps are limited per input
event, not per second. The gripper participates in those steps but not in the
final alignment threshold comparison.

Driver rejection or an input-handling exception enters `error`, blocks moves
and snapshots, and retains the driver. It does not automatically run a stop
trajectory, disable motors, or retry. Exceptions also print their traceback.
Explicit stop in this state skips the trajectory and attempts disable; explicit
start first stops the old instance without a trajectory, then creates a new one.
Failed stop retains the instance and reports `error`, not `stopped`.

On normal exit, `--stop` retains the driver's configured stop behavior. Faulted
or interrupted exits attempt stop without a trajectory, even with `--no-stop`.
A failed `--no-stop` return trajectory follows the same no-motion cleanup.
Cleanup errors are reported and propagated, not treated as confirmed disable. None of these
software operations guarantees a physical emergency stop.

Source `timestamp` can be a Dora `datetime` or an application-supplied value;
convert it to the same clock and units before comparing with `executed_timestamp`.

Migration: replace `request_state` / `request_position` / `request_command` with
`publish_tick`, replace `position` subscriptions with `state`, and remove
`--start-on-startup`. Send an explicit `command: start` instead. Old input names
are not aliases. Install the matching driver change before running this node.

## License

Licensed under the Apache License 2.0. See [LICENSE](LICENSE) for details.

Copyright 2026 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our [Code of Conduct](CODE_OF_CONDUCT.md).

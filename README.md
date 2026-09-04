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
      request_position: leader/right_follower_position
      move_position: leader/right_follower_position
    outputs:
      - position
      - latest_command
      - status

  - id: follower-left
    build: pip install dora-openarm
    path: dora-openarm
    args: "--side left --align-trigger gripper"
    inputs:
      # Only the event ID is used. The event value is ignored.
      request_position: leader/left_follower_position
      move_position: leader/left_follower_position
    outputs:
      - position
      - latest_command
      - status
  # ...
```

### Node arguments

| Argument | Description |
| --- | --- |
| `--side` | OpenArm side to control. Default: `right`. |
| `--config` | Path to the OpenArm configuration file. Default: `openarm_cell.yaml`. |
| `--align-trigger` | Optional trigger for the initial alignment step. Supported value: `gripper`. |
| `--align-threshold` | Alignment threshold in radians. Default: `0.1`. |
| `--align-delta-limit` | Maximum joint delta per initial-alignment command in radians. Default: `0.001`. |
| `--[no-]align` | Whether to align to incoming position commands after the arm starts. Default: enabled. |
| `--[no-]stop` | Whether to stop the arm when the node exits. Default: controlled by the `STOP` environment variable, or `true` when it is unset. |
| `--[no-]refresh-every-request` | Whether to refresh OpenArm state before each request. Default: controlled by the `REFRESH` environment variable, or `true` when it is unset. |

### Inputs

| Input | Description |
| --- | --- |
| `request_position` | Requests the current arm position. The event ID is used and the event value is ignored. |
| `request_state` | Requests one current arm-state snapshot and publishes `state`, its `qpos` as `position`, and the last valid `latest_command` snapshot. The event ID is used and the event value is ignored. |
| `move_position` | Sends a new target position to the arm. The value may be a struct containing `qpos` (`[{"qpos": [...]}]`), a position array directly, or a legacy struct containing `new_position`. A command without `start_epoch` is accepted for compatibility; a supplied epoch must match the current start. When initial alignment is enabled, this input drives the alignment until it completes. |

### Outputs

| Output | Description |
| --- | --- |
| `position` | Current arm position as a length-1 struct containing a float32 array: `[{"qpos": [...]}]`. |
| `state` | Current arm state as a length-1 struct with list fields: `[{"qpos": [...], "qvel": [...], "qtorque": [...], "tmos": [...], "trotor": [...]}]`. `qpos`, `qvel`, and `qtorque` are float32 lists; `tmos` (MOS temperature) and `trotor` (rotor temperature) are int32 lists per motor, in °C. |
| `latest_command` | Final joint command accepted by the driver after safety clamping, using the same `qpos` struct as `position`. Its original action `timestamp` is preserved and `executed_timestamp` records when the driver dispatched the command. For an internally generated startup trajectory, both timestamps identify its final dispatched command. Repeated request snapshots preserve both timestamps. |
| `status` | Current control state as a string array: `stopped`, `started`, or `aligned`. With alignment enabled, `aligned` is emitted once initial alignment completes. |

Every output carries a process-local `start_epoch`. It increments after each
successful `start` and distinguishes commands and observations from earlier
arm-enable sessions.

## License

Licensed under the Apache License 2.0. See [LICENSE](LICENSE) for details.

Copyright 2026 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our [Code of Conduct](CODE_OF_CONDUCT.md).

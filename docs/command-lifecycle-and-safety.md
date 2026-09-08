# Command lifecycle and safety

This document describes how `dora-openarm` handles joint commands, reports the
command accepted by the driver, associates metadata with arm sessions, and
applies startup and driver-level safety checks.

Exact joint limits and velocity limits are defined by the selected OpenArm
driver configuration.

## Position terminology

The control path contains three different joint-position values:

- A **requested command** is the `qpos` received on `move_position`.
- An **accepted command** is the final target produced by the driver's safety
  pipeline and dispatched to the motor interface. It can differ from the
  requested command when a non-fatal safety check clamps it.
- A **measured position** is read back from the arm and published as `position`
  or as the `qpos` field of `state`.

`latest_command` reports the accepted command. It is not a measurement and
does not indicate that the physical arm has reached the target.

## End-to-end command flow

```text
move_position
  -> stopped-state check
  -> start_epoch check
  -> input parsing
  -> initial alignment, when enabled and not complete
  -> driver safety checks
       -> hard rejection: dispatch nothing and leave the cache unchanged
       -> soft correction: dispatch the corrected target
       -> accepted as-is: dispatch the requested target
  -> cache source metadata for the accepted command

publish_tick
  -> stopped-state check
  -> fetch_state() once
  -> publish state and position from that snapshot
  -> publish latest_command with cached source and execution metadata, if valid
```

The driver updates `last_command` only after a command passes all hard safety
checks. On each `publish_tick`, the node publishes this driver value rather
than echoing the original input. As a result, `latest_command.qpos` includes any
joint-position or joint-velocity correction applied by the driver.

If the driver rejects a command, `send_position()` returns false. The node does
not update the cached command metadata. Subsequent ticks continue to publish
the previous accepted command, if one exists, with its original timestamps.

`publish_tick` is the only trigger for `latest_command`. A `move_position` event
applies the command and updates the cache without publishing a snapshot. If
several commands are accepted between ticks, only the latest is reported.
Consumers therefore receive a sampled command state, not a complete log of
every dispatch. The reporting rate is set by the tick source; a 4 ms timer
requests up to 250 snapshots per second per arm. Ticks do not resend cached
position targets to the motor interface.

## Session epochs

`start_epoch` is a process-local arm-enable generation number. It starts at
zero and increments after every successful `arm.start()`, including a start
performed by `--start-on-startup`.

The node adds the current epoch to every output. A `move_position` carrying a
`start_epoch` is accepted only when the value is a non-boolean integer equal to
the current epoch. A malformed or mismatched epoch is logged and ignored. This
prevents a queued command from an earlier arm-enable session from moving the
arm after a stop/start cycle.

For compatibility, commands without `start_epoch` are accepted. An upstream
node must therefore copy the epoch from a current arm output into its commands
to enable stale-session protection.

The epoch has two intentional limitations:

- It resets to zero when the `dora-openarm` process restarts.
- It does not detect reordered commands within the same epoch.

It is a session-generation guard, not a globally unique command identifier.

## Command timestamps

For a command originating from `move_position`, `latest_command` preserves the
input metadata, including its `timestamp` when present, and adds:

- `executed_timestamp`: wall-clock nanoseconds recorded immediately before the
  driver dispatches the accepted target to the motor interface.
- `start_epoch`: the current arm-enable generation. This overrides any copied
  input value with the driver-side source of truth.

These timestamps can be used to measure source-to-driver latency:

```text
source-to-driver latency = executed_timestamp - timestamp
```

This subtraction is meaningful only when both timestamps use the same clock
domain, or when the source and driver hosts have synchronized wall clocks.

`executed_timestamp` is a software dispatch timestamp. It does not represent a
motor acknowledgement, physical motion onset, or target-settling time.
The delay until the next tick is not included in this timestamp difference.
Command snapshots keep the command's metadata, not the triggering tick's
metadata; the tick metadata is used for `state` and `position` instead.

### Startup commands

The driver's startup trajectory has no external source event. If it dispatches
one or more commands, the final dispatch time is used as both `timestamp` and
`executed_timestamp` for the cached startup command.

The startup command becomes visible when a later `publish_tick` asks the node
to publish the driver's last valid command. If the configured startup
trajectory dispatches no command, there is no startup `latest_command`.

Republishing a command preserves its original timestamps. A tick does not
assign a new execution time.

## State snapshot consistency

A `publish_tick` performs one `fetch_state()` operation and derives both state
outputs from the same returned object:

```text
publish_tick -> one driver state read (CAN refresh when enabled)
  -> state     (qpos, qvel, qtorque, tmos, trotor)
  -> position  (the same qpos sample)
  -> latest_command snapshot, when one exists
```

This avoids a second CAN refresh and guarantees that `state.qpos` and
`position.qpos` belong to the same hardware sample. `latest_command` remains a
command snapshot and is intentionally independent of the measured state.

`request_position` remains available when only a position read is required; it
does not publish `latest_command`. `--refresh-every-request` controls CAN
refresh for both `publish_tick` events and position requests.

Dataflows using the former `request_state` input must rename it to
`publish_tick` and connect it to a timer or a tick-forwarding node:

```yaml
inputs:
  publish_tick: quittable-tick-arm/tick
```

There is no `request_state` or `tick` alias. Commands, startup, and alignment
status changes do not trigger snapshots.

## Initial alignment

Initial alignment is a node-level startup guard. It is created after each start
when `--align` is enabled and is not re-entered during normal control.

The first alignment target is initialized from the measured arm position. On
each eligible `move_position` input, every joint moves toward the incoming
target by at most `--align-delta-limit` radians:

```text
delta = clip(requested - alignment_target, -delta_limit, delta_limit)
alignment_target = alignment_target + delta
```

The step limit also applies to the gripper. Alignment completion compares only
the seven arm joints with `--align-threshold`; the gripper is excluded from the
completion test.

With `--align-trigger gripper`, alignment remains paused until the target
gripper angle satisfies the side-specific condition:

- Right arm: target gripper angle is greater than `-5 deg`.
- Left arm: target gripper angle is less than `5 deg`.

`--align-delta-limit` is a delta per input event, not a physical velocity in
radians per second. The resulting alignment speed therefore depends on the
incoming command frequency.

When the measured arm enters the alignment threshold, `_align()` submits the
final requested target through the same checked and caching path used during
normal control. The node publishes `aligned` only if the driver accepts that
final command; its command snapshot is published on the next tick. If it is
rejected, the node remains in `started` and leaves the command cache unchanged.

## Driver safety pipeline

Alignment commands and normal-control commands pass through the same driver
safety pipeline. The default checks run in this order:

1. **Joint-position limits** clamp each target to its configured valid range.
2. **Joint-delta limits** compare the corrected target with `last_command`. An
   excessive single-command jump latches a safety stop and rejects the command.
3. **Optional joint-velocity limits** clamp each joint to the distance allowed
   since the previous accepted command.

For velocity limiting, the driver uses:

```text
allowed_delta = configured_velocity_limit * min(elapsed_time, 0.04 s)
```

The 40 ms cap prevents a long input pause from authorizing one large movement
when commands resume.

### Soft correction

A non-fatal position or velocity clamp produces a corrected target. The driver
dispatches that corrected target, stores it as `last_command`, and the node
publishes it as `latest_command` on the next tick unless a newer command has
already been accepted.

### Hard rejection

A hard rejection dispatches no target and leaves `last_command` and its cached
metadata unchanged. Ticks may still publish the previous valid command; that
snapshot does not indicate a new dispatch or that the driver is accepting
commands. A joint-delta violation also latches the driver's safety-stop state.
Later position commands are ignored until a stop/start cycle creates a fresh
driver session and clears the latch.

## Stop behavior

On `stop`, the node:

1. Publishes `stopped` with the current epoch.
2. Runs the driver's configured stop behavior.
3. Releases the driver instance.
4. Clears cached alignment state and command metadata.

Commands generated internally by the driver's stop behavior are not published
as `latest_command`. While stopped, ticks produce no arm snapshots. The next
successful `start` creates a fresh driver instance and advances `start_epoch`.

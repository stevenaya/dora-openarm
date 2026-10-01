"""Offline lifecycle checks; neither Dora nor the driver connects to hardware."""

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import MagicMock, call

import numpy as np
import pyarrow as pa
import pytest


@pytest.fixture
def runtime(monkeypatch):
    """Provide a fake driver and run the real node event loop."""
    arm = MagicMock()
    arm.safety_stop_reason = None
    arm.startup_timestamp = None
    arm.last_command_dispatch_timestamp_ns = None
    arm.last_command = np.zeros(8)
    arm.start.return_value = True
    arm.send_position.return_value = True
    arm.move_to_start_position.return_value = True
    arm.fetch_position.return_value = np.zeros(8)
    arm.fetch_state.return_value = {
        key: np.zeros(8) for key in ("qpos", "qvel", "qtorque", "tmos", "trotor")
    }
    arm.get_health.return_value = (["ENABLED"] * 8, {})

    def start():
        arm.last_command_dispatch_timestamp_ns = arm.startup_timestamp
        return arm.start.return_value

    def send_position(position):
        if arm.send_position.return_value:
            arm.last_command = np.clip(position, -1, 1)
            arm.last_command_dispatch_timestamp_ns = 123
        return arm.send_position.return_value

    arm.start.side_effect = start
    arm.send_position.side_effect = send_position
    factory = MagicMock(return_value=arm)
    monkeypatch.setitem(
        sys.modules,
        "openarm_driver",
        SimpleNamespace(Config=lambda _: None, SingleArmDriver=factory),
    )
    path = Path(__file__).parents[1] / "src/dora_openarm/main.py"
    spec = importlib.util.spec_from_file_location("arm_under_test", path)
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    node = MagicMock()
    outputs, statuses = [], []

    def send_output(name, value, metadata):
        outputs.append((name, value.to_pylist(), metadata))
        if name == "status":
            statuses.append(value[0].as_py())

    node.send_output.side_effect = send_output
    monkeypatch.setattr(module.dora, "Node", lambda: node)

    def run(events, *args):
        node.__iter__.side_effect = lambda: iter(events)
        monkeypatch.setattr(sys, "argv", ["dora-openarm", "--no-align", *args])
        module.main()

    return SimpleNamespace(
        arm=arm,
        factory=factory,
        module=module,
        run=run,
        outputs=outputs,
        statuses=statuses,
    )


def event(name, value=None, **metadata):
    """Build one input without Dora's runtime."""
    return {
        "type": "INPUT",
        "id": name,
        "value": value if isinstance(value, pa.Array) else pa.array(value or []),
        "metadata": metadata,
    }


def test_startup_is_idle(runtime):
    """Node construction never enables an arm."""
    runtime.run([])
    assert runtime.statuses == ["stopped"]
    runtime.factory.assert_not_called()


def test_tick_samples_state_and_preserves_accepted_command(runtime, monkeypatch):
    """Read once per tick, retain source metadata, and ignore stale epochs."""
    clock = iter((100, 200, 300))
    monkeypatch.setattr(runtime.module.time, "time_ns", lambda: next(clock))
    tick = event("publish_tick", timestamp=10, trace_id="keep")
    runtime.run(
        [
            event("command", ["start"], episode_attempt_id="attempt-1"),
            tick,
            event(
                "move_position",
                [2.0] * 8,
                timestamp=50,
                chunk_id="new",
                blended_chunk_id="old",
            ),
            tick,
            tick,
            event("move_position", [0.0] * 8, start_epoch=0),
            event("command", ["stop"]),
            tick,
        ]
    )
    assert runtime.arm.fetch_state.call_count == 3
    assert runtime.arm.send_position.call_count == 1
    commands = [out for out in runtime.outputs if out[0] == "latest_command"]
    assert len(commands) == 2 and commands[0] == commands[1]
    assert commands[0][1] == [{"qpos": [1.0] * 8}]
    assert commands[0][2] == dict(
        timestamp=50,
        chunk_id="new",
        blended_chunk_id="old",
        executed_timestamp=123,
        start_epoch=1,
    )
    states = [out for out in runtime.outputs if out[0] == "state"]
    assert [out[2]["observation_timestamp"] for out in states] == [100, 200, 300]
    assert all(
        "timestamp" not in out[2] and out[2]["trace_id"] == "keep" for out in states
    )
    assert tick["metadata"] == {"timestamp": 10, "trace_id": "keep"}
    assert runtime.statuses == ["stopped", "started", "stopped"]
    runtime.arm.stop.assert_called_once_with(run_trajectory=True)


@pytest.mark.parametrize("timestamp", [None, 99])
def test_startup_command_snapshot(runtime, timestamp):
    """Only a dispatched startup command has a cached command snapshot."""
    runtime.arm.startup_timestamp = timestamp
    runtime.run([event("command", ["start"]), event("publish_tick")])
    commands = [out for out in runtime.outputs if out[0] == "latest_command"]
    assert len(commands) == int(timestamp is not None)
    if commands:
        assert commands[0][2] == dict(
            timestamp=99, executed_timestamp=99, start_epoch=1
        )


@pytest.mark.parametrize("accepted", [False, True])
def test_alignment_final_target_must_be_accepted(runtime, accepted):
    """The final alignment target uses the same acceptance/cache path."""
    runtime.arm.send_position.return_value = accepted
    runtime.run(
        [
            event("command", ["start"]),
            event("move_position", [0.0] * 8),
            event("publish_tick"),
            event("command", ["stop"]),
        ],
        "--align",
    )
    assert ("aligned" in runtime.statuses) == accepted
    assert ("error" in runtime.statuses) != accepted
    assert runtime.arm.send_position.call_count == 1
    assert any(out[0] == "latest_command" for out in runtime.outputs) == accepted
    runtime.arm.stop.assert_called_once_with(run_trajectory=accepted)


@pytest.mark.parametrize("failure", [False, RuntimeError("start failed")])
def test_start_failure_keeps_instance_and_epoch(runtime, failure):
    """A failed start remains explicitly stoppable without a return trajectory."""
    if failure is False:
        runtime.arm.start.return_value = False
    else:
        runtime.arm.start.side_effect = failure
    runtime.run(
        [
            event("command", ["start"], episode_attempt_id="attempt-1"),
            event("move_position", [0.0] * 8),
            event("publish_tick"),
            event("command", ["stop"]),
        ]
    )
    assert runtime.statuses == ["stopped", "error", "stopped"]
    error = next(
        out[2] for out in runtime.outputs if out[0] == "status" and out[1] == ["error"]
    )
    assert error["start_epoch"] == 0 and error["episode_attempt_id"] == "attempt-1"
    runtime.arm.send_position.assert_not_called()
    runtime.arm.fetch_state.assert_not_called()
    runtime.arm.stop.assert_called_once_with(run_trajectory=False)


@pytest.mark.parametrize("operation", ["parse", "move", "state"])
def test_runtime_exception_waits_for_explicit_stop(runtime, operation, capsys):
    """Catch input/driver faults once, retain control, and never auto-stop in-loop."""
    bad_input = event("move_position", pa.array([{"invalid": [0.0] * 8}]))
    if operation == "move":
        runtime.arm.send_position.side_effect = RuntimeError("send failed")
        bad_input = event("move_position", [0.0] * 8)
    elif operation == "state":
        runtime.arm.fetch_state.side_effect = RuntimeError("read failed")
        bad_input = event("publish_tick")

    def events():
        yield event("command", ["start"])
        yield bad_input
        assert runtime.statuses[-1] == "error"
        runtime.arm.stop.assert_not_called()
        yield event("move_position", [0.0] * 8)
        yield event("publish_tick")
        yield event("command", ["stop"])

    runtime.run(events())
    assert runtime.statuses == ["stopped", "started", "error", "stopped"]
    assert runtime.arm.send_position.call_count == int(operation == "move")
    assert runtime.arm.fetch_state.call_count == int(operation == "state")
    runtime.arm.stop.assert_called_once_with(run_trajectory=False)
    assert "Traceback" in capsys.readouterr().err


def test_failed_stop_can_be_retried_without_motion(runtime):
    """Do not discard the driver or publish stopped when stop raises."""
    runtime.arm.stop.side_effect = [RuntimeError("disable failed"), None]
    runtime.run(
        [
            event("command", ["start"]),
            event("command", ["stop"]),
            event("move_position", [0.0] * 8),
            event("command", ["stop"]),
        ]
    )
    assert runtime.statuses == ["stopped", "started", "error", "stopped"]
    assert runtime.arm.stop.call_args_list == [
        call(run_trajectory=True),
        call(run_trajectory=False),
    ]
    runtime.arm.send_position.assert_not_called()


def test_restart_after_error_clears_cached_command(runtime):
    """Restart disables the faulted instance before advancing the session."""
    runtime.run(
        [
            event("command", ["start"]),
            event("move_position", [0.1] * 8),
            event("publish_tick"),
            event("move_position", pa.array([{"invalid": []}])),
            event("command", ["start"]),
            event("publish_tick"),
            event("command", ["stop"]),
        ]
    )
    assert runtime.factory.call_count == 2
    assert runtime.arm.stop.call_args_list == [
        call(run_trajectory=False),
        call(run_trajectory=True),
    ]
    assert len([out for out in runtime.outputs if out[0] == "latest_command"]) == 1
    epochs = [out[2]["start_epoch"] for out in runtime.outputs if out[1] == ["started"]]
    assert epochs == [1, 2]


@pytest.mark.parametrize("result", [True, False, RuntimeError("return failed")])
def test_no_stop_exit_checks_return_trajectory(runtime, result):
    """Only a successful normal no-stop exit leaves the arm enabled."""
    if isinstance(result, Exception):
        runtime.arm.move_to_start_position.side_effect = result
        with pytest.raises(RuntimeError, match="return failed"):
            runtime.run([event("command", ["start"])], "--no-stop")
    else:
        runtime.arm.move_to_start_position.return_value = result
        runtime.run([event("command", ["start"])], "--no-stop")
    if result is True:
        runtime.arm.stop.assert_not_called()
    else:
        assert runtime.statuses[-1] == "error"
        runtime.arm.stop.assert_called_once_with(run_trajectory=False)


@pytest.mark.parametrize("exception", [KeyboardInterrupt, SystemExit, RuntimeError])
def test_abnormal_exit_disables_without_trajectory(runtime, exception):
    """Cleanup cannot turn an interrupted event stream into a return motion."""

    def events():
        yield event("command", ["start"])
        raise exception()

    with pytest.raises(exception):
        runtime.run(events(), "--no-stop")
    runtime.arm.stop.assert_called_once_with(run_trajectory=False)
    runtime.arm.move_to_start_position.assert_not_called()


def test_faulted_exit_disables_even_with_no_stop(runtime):
    """An already reported node fault overrides the normal no-stop behavior."""
    runtime.arm.send_position.return_value = False
    runtime.run(
        [
            event("command", ["start"]),
            event("move_position", [0.0] * 8),
        ],
        "--no-stop",
    )
    runtime.arm.stop.assert_called_once_with(run_trajectory=False)
    runtime.arm.move_to_start_position.assert_not_called()


def test_shutdown_failure_is_reported_and_propagated(runtime):
    """A failed exit disable cannot become a successful process exit."""
    runtime.arm.stop.side_effect = RuntimeError("disable failed")
    with pytest.raises(RuntimeError, match="disable failed"):
        runtime.run([event("command", ["start"])])
    assert runtime.statuses == ["stopped", "started", "error"]


def test_dora_stop_does_not_process_queued_moves(runtime):
    """A normal Dora stop follows normal shutdown policy and ends input handling."""
    runtime.run(
        [
            event("command", ["start"]),
            {"type": "STOP"},
            event("move_position", [0.1] * 8),
        ]
    )
    runtime.arm.send_position.assert_not_called()
    runtime.arm.stop.assert_called_once_with(run_trajectory=True)


@pytest.mark.parametrize(
    "value",
    [
        pa.array([0.1] * 8),
        pa.array([{"qpos": [0.1] * 8}]),
        pa.array([{"new_position": [0.1] * 8}]),
        pa.StructArray.from_arrays([pa.array([0.1] * 8)], names=["new_position"]),
    ],
)
def test_supported_command_shapes(runtime, value):
    """Keep canonical and legacy command formats working through one parser."""
    runtime.run([event("command", ["start"]), event("move_position", value)])
    assert "error" not in runtime.statuses
    np.testing.assert_allclose(runtime.arm.last_command, [0.1] * 8)

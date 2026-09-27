"""Offline arm lifecycle and snapshot contract checks (no CAN access)."""

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pyarrow as pa
import pytest


@pytest.fixture
def run_node(monkeypatch):
    """Run the event loop with a fake driver and no hardware access."""
    def run(events, *, started=True, stop_failures=0, accepted=True, align=False):
        trace, outputs, drivers = [], [], []

        class Driver:
            safety_stop_reason = None
            last_command = np.zeros(8, dtype=np.float32)
            last_command_dispatch_timestamp_ns = None

            def __init__(self, *args, **kwargs):
                drivers.append(self)
                self.stop_failures = stop_failures

            def start(self):
                trace.append("start")
                if not started:
                    self.safety_stop_reason = "startup rejected"
                return started

            def stop(self):
                trace.append("stop")
                if self.stop_failures:
                    self.stop_failures -= 1
                    raise RuntimeError("stop rejected")

            def send_position(self, position):
                trace.append("move")
                if not accepted:
                    self.safety_stop_reason = "delta too large"
                    return False
                self.last_command = np.clip(position, -1, 1)
                self.last_command_dispatch_timestamp_ns = 123
                return True

            def fetch_state(self, refresh):
                trace.append("read")
                return {
                    key: np.zeros(8)
                    for key in ("qpos", "qvel", "qtorque", "tmos", "trotor")
                }

            def fetch_position(self):
                return np.zeros(8)

            def get_health(self):
                return ["OK"] * 8, {}

        class Node(list):
            def send_output(self, name, value, metadata):
                outputs.append((name, value.to_pylist(), metadata))
                if name == "status":
                    trace.append(value[0].as_py())

        monkeypatch.setitem(
            sys.modules,
            "openarm_driver",
            SimpleNamespace(
                Config=lambda _: None,
                SingleArmDriver=Driver,
            ),
        )
        path = Path(__file__).parents[1] / "src/dora_openarm/main.py"
        spec = importlib.util.spec_from_file_location("arm_lifecycle_under_test", path)
        module = importlib.util.module_from_spec(spec)
        monkeypatch.setitem(sys.modules, spec.name, module)
        spec.loader.exec_module(module)
        monkeypatch.setattr(module.dora, "Node", lambda: Node(events))
        monkeypatch.setattr(
            sys, "argv", ["dora-openarm", "--align" if align else "--no-align"]
        )
        module.main()
        return outputs, trace, drivers

    return run


def event(name, value=None, **metadata):
    """Build an input event for the mock Dora node."""
    return {
        "type": "INPUT",
        "id": name,
        "value": pa.array(value or []),
        "metadata": metadata,
    }


def test_single_tick_reads_once_and_preserves_command_metadata(run_node):
    """Check idle startup, snapshot timing, clamped targets and epoch rejection."""
    outputs, trace, drivers = run_node([])
    assert trace == ["stopped"] and not drivers
    outputs, trace, _ = run_node(
        [
            event("command", ["start"], episode_attempt_id="attempt-1"),
            event("publish_tick", timestamp=10),
            event(
                "move_position",
                [2.0] * 8,
                timestamp=50,
                chunk_id="chunk-2",
                blended_chunk_id="chunk-1",
            ),
            event("publish_tick", timestamp=20),
            event("publish_tick", timestamp=30),
            event("move_position", [0.0] * 8, start_epoch=0),
            event("command", ["stop"]),
            event("publish_tick"),
        ]
    )
    assert trace.count("read") == 3 and trace.count("move") == 1
    assert trace[-2:] == ["stop", "stopped"]
    assert {name for name, _, _ in outputs} == {"status", "state", "latest_command"}
    commands = [out for out in outputs if out[0] == "latest_command"]
    assert len(commands) == 2 and commands[0] == commands[1]
    assert commands[0][1] == [{"qpos": [1.0] * 8}]
    assert commands[0][2] == dict(
        timestamp=50,
        chunk_id="chunk-2",
        blended_chunk_id="chunk-1",
        executed_timestamp=123,
        start_epoch=1,
    )
    for name, value, metadata in outputs:
        if name == "state":
            assert "observation_timestamp" in metadata and "timestamp" not in metadata
            assert "motor_status" in value[0] and "bus" in value[0]


@pytest.mark.parametrize("started,accepted", [(False, True), (True, False)])
def test_fault_blocks_moves_and_preserves_explicit_stop(run_node, started, accepted):
    """Keep a failed driver reachable for an explicit stop."""
    outputs, trace, drivers = run_node(
        [
            event("command", ["start"], episode_attempt_id="attempt-1"),
            event("move_position", [1.0] * 8),
            event("move_position", [1.0] * 8),
            event("publish_tick"),
            event("command", ["stop"]),
        ],
        started=started,
        accepted=accepted,
    )
    errors = [
        metadata
        for name, value, metadata in outputs
        if name == "status" and value == ["error"]
    ]
    assert len(errors) == 1 and errors[0]["episode_attempt_id"] == "attempt-1"
    assert errors[0]["start_epoch"] == int(started)
    assert trace.count("move") == int(started) and "read" not in trace
    assert len(drivers) == 1 and trace[-2:] == ["stop", "stopped"]
    if not started:
        assert "started" not in trace


@pytest.mark.parametrize("accepted", [False, True])
def test_alignment_reports_success_only_after_accepted_final_target(run_node, accepted):
    """Never emit aligned for a rejected final target."""
    _, trace, _ = run_node(
        [
            event("command", ["start"]),
            event("move_position", [0.0] * 8),
            event("command", ["stop"]),
        ],
        align=True,
        accepted=accepted,
    )
    assert ("aligned" in trace) == accepted
    assert ("error" in trace) != accepted
    assert trace.count("move") == 1


def test_failed_stop_is_not_reported_as_stopped_and_can_be_retried(run_node):
    """Retain the instance when stop fails without claiming success."""
    _, trace, drivers = run_node(
        [
            event("command", ["start"]),
            event("command", ["stop"]),
            event("move_position", [0.0] * 8),
            event("command", ["stop"]),
        ],
        stop_failures=1,
    )
    assert trace == ["stopped", "start", "started", "stop", "error", "stop", "stopped"]
    assert len(drivers) == 1

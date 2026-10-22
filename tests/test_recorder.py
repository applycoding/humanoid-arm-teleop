from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from backend import config
from backend.recorder import RecorderError, TrajectoryRecorder
from backend.safety import SafetyMode


def test_record_and_replay(repo_root, control_loop, registry) -> None:
    loop = control_loop
    t_ref = {"t": 0.0}
    loop.mono_clock = lambda: t_ref["t"]
    owner = registry.register(object())
    registry.request_control(owner.conn_id)
    owner.deadman_held = True
    owner.deadman_expiry_mono = 100.0
    loop.safety.mode = SafetyMode.HOLD

    loop.start_recording("test_traj")
    positions = []
    for i in range(5):
        loop.q_committed = np.array(config.Q_HOME, dtype=float) + 0.01 * i
        loop.recorder.record_tick(i * config.CONTROL_DT, loop.q_committed.tolist(), "HOLD")
        positions.append(loop.q_committed.copy())
    loop.stop_recording()

    path = repo_root / "recordings" / "test_traj.jsonl"
    lines = path.read_text(encoding="utf-8").strip().split("\n")
    assert len(lines) == 6
    header = json.loads(lines[0])
    assert header["schema_version"] == 1
    assert header["joint_names"] == loop.model.revolute_names

    loop.q_committed = np.array(config.Q_HOME, dtype=float)
    loop.start_replay("test_traj")
    assert loop.safety.mode == SafetyMode.REPLAYING
    recorded = [json.loads(ln)["joint_position"] for ln in lines[1:]]
    seen = []
    for _ in range(500):
        loop.tick()
        t_ref["t"] += config.CONTROL_DT
        seen.append(loop.q_committed.tolist())
        if loop.safety.mode != SafetyMode.REPLAYING:
            break
    for rec in recorded:
        assert any(np.allclose(rec, s, atol=1e-5) for s in seen)


def test_replay_urdf_mismatch(repo_root) -> None:
    urdf = repo_root / "arm" / "arm.urdf"
    rec = TrajectoryRecorder(repo_root / "recordings", urdf, ["joint_1"])
    path = repo_root / "recordings" / "bad.jsonl"
    path.write_text(
        '{"schema_version":1,"joint_names":["joint_1"],"urdf_sha256":"deadbeef","tick_dt":0.02}\n',
        encoding="utf-8",
    )
    rec2 = TrajectoryRecorder(repo_root / "recordings", urdf, ["joint_1"])
    with __import__("pytest").raises(RecorderError):
        rec2.load_replay("bad")

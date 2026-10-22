from __future__ import annotations

import numpy as np

from backend import config
from backend.safety import SafetyMode


def _setup_owner(loop, registry, t_ref: dict) -> int:
    owner = registry.register(object())
    registry.request_control(owner.conn_id)
    owner.deadman_held = True
    owner.deadman_expiry_mono = t_ref["t"] + 10.0
    loop.safety.pose_received_since_hold = True
    return owner.conn_id


def test_loop_converges_toward_target(control_loop, registry) -> None:
    loop = control_loop
    t_ref = {"t": 0.0}
    loop.mono_clock = lambda: t_ref["t"]
    _setup_owner(loop, registry, t_ref)
    poses = __import__("json").loads(
        (__import__("pathlib").Path(__file__).parent / "fixtures" / "poses.json").read_text()
    )
    tgt = poses["b"]
    loop.target_position = np.array(
        [tgt["position"]["x"], tgt["position"]["y"], tgt["position"]["z"]], dtype=float
    )
    loop.target_rotation = loop.kin.quat_to_matrix(
        tgt["orientation"]["x"],
        tgt["orientation"]["y"],
        tgt["orientation"]["z"],
        tgt["orientation"]["w"],
    )
    loop.safety.mode = SafetyMode.RUNNING
    err0 = np.linalg.norm(loop.kin.forward(loop.q_committed).position - loop.target_position)
    for _ in range(200):
        loop.tick()
        t_ref["t"] += config.CONTROL_DT
    err1 = np.linalg.norm(loop.kin.forward(loop.q_committed).position - loop.target_position)
    assert err1 < err0
    assert err1 < 0.01


def test_state_machine_sequence(control_loop, registry) -> None:
    loop = control_loop
    t_ref = {"t": 0.0}
    loop.mono_clock = lambda: t_ref["t"]
    assert loop.safety.mode == SafetyMode.HOLD
    _setup_owner(loop, registry, t_ref)
    loop.safety.pose_received_since_hold = False
    loop.tick()
    assert loop.safety.mode == SafetyMode.HOLD
    loop.safety.pose_received_since_hold = True
    loop.tick()
    assert loop.safety.mode == SafetyMode.RUNNING
    loop.engage_estop()
    assert loop.safety.mode == SafetyMode.ESTOP
    loop.reset_from_fault_or_estop()
    assert loop.safety.mode == SafetyMode.HOLD

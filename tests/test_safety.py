from __future__ import annotations

import json
from typing import Any

import numpy as np
import pytest

from backend import config
from backend.safety import SafetyMode, in_envelope


def _fk_ee(loop, q):
    fk = loop.kin.forward(q)
    return fk.position, fk.rotation


def test_safety_transitions(control_loop, registry) -> None:
    cases = [
        ("estop_from_running", {"estop": True}, SafetyMode.ESTOP),
        ("reset_from_estop", {"reset": True}, SafetyMode.HOLD),
    ]
    loop = control_loop
    mono = loop.mono_clock

    def advance(dt: float) -> None:
        mono.__func__  # type: ignore
        t = loop.mono_clock()
        loop.mono_clock = lambda: t + dt

    # estop from RUNNING
    owner = registry.register(object())
    registry.request_control(owner.conn_id)
    owner.deadman_held = True
    owner.deadman_expiry_mono = loop.mono_clock() + 1.0
    loop.safety.mode = SafetyMode.RUNNING
    loop.safety.pose_received_since_hold = True
    loop.engage_estop()
    assert loop.safety.mode == SafetyMode.ESTOP
    loop.reset_from_fault_or_estop()
    assert loop.safety.mode == SafetyMode.HOLD
    fk = loop.kin.forward(loop.q_committed)
    assert np.allclose(loop.target_position, fk.position)


def test_fault_on_nonfinite(control_loop, registry) -> None:
    loop = control_loop
    owner = registry.register(object())
    registry.request_control(owner.conn_id)
    owner.deadman_held = True
    owner.deadman_expiry_mono = loop.mono_clock() + 10.0
    loop.safety.mode = SafetyMode.RUNNING
    loop.safety.pose_received_since_hold = True
    loop.q_committed[0] = float("nan")
    loop.tick()
    assert loop.safety.fault
    loop.reset_from_fault_or_estop()
    assert loop.safety.mode == SafetyMode.HOLD


def test_deadman_expiry_to_hold(control_loop, registry) -> None:
    loop = control_loop
    t = 0.0

    def mono() -> float:
        return t

    loop.mono_clock = mono
    owner = registry.register(object())
    registry.request_control(owner.conn_id)
    owner.deadman_held = True
    owner.deadman_expiry_mono = 0.25
    loop.safety.mode = SafetyMode.RUNNING
    loop.safety.pose_received_since_hold = True
    t = 0.3
    loop.tick()
    assert loop.safety.mode == SafetyMode.HOLD


def test_invalid_pose_rejected(control_loop, registry) -> None:
    loop = control_loop
    owner = registry.register(object())
    registry.request_control(owner.conn_id)
    q_before = loop.q_committed.copy()
    loop.target_position = np.array([float("nan"), 0.0, 0.0])
    loop.tick()
    assert np.allclose(loop.q_committed, q_before)


def test_committed_invariant_after_reject(control_loop, registry) -> None:
    loop = control_loop
    owner = registry.register(object())
    registry.request_control(owner.conn_id)
    owner.deadman_held = True
    owner.deadman_expiry_mono = loop.mono_clock() + 10.0
    loop.safety.mode = SafetyMode.RUNNING
    loop.safety.pose_received_since_hold = True
    loop.target_position = np.array([5.0, 5.0, 5.0])
    q0 = loop.q_committed.copy()
    for _ in range(5):
        loop.tick()
    fk = loop.kin.forward(loop.q_committed)
    # An unreachable target must not drag the arm far or out of the safe envelope.
    assert np.linalg.norm(fk.position - loop.kin.forward(q0).position) < 0.5
    assert in_envelope(fk.position)

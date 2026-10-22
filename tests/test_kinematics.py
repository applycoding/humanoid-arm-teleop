from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pytest

from backend import config
from backend.kinematics import orientation_error_vec
from backend.safety import check_keepout, in_envelope


def _load_poses() -> dict:
    path = Path(__file__).parent / "fixtures" / "poses.json"
    return json.loads(path.read_text(encoding="utf-8"))


def _assert_pose(fk, expected: dict, pos_tol: float = 1e-6) -> None:
    assert abs(fk.position[0] - expected["position"]["x"]) < pos_tol
    assert abs(fk.position[1] - expected["position"]["y"]) < pos_tol
    assert abs(fk.position[2] - expected["position"]["z"]) < pos_tol
    o = expected["orientation"]
    qx, qy, qz, qw = fk.quat
    dot = abs(qx * o["x"] + qy * o["y"] + qz * o["z"] + qw * o["w"])
    assert dot > 1.0 - 1e-5 or dot < -1.0 + 1e-5


def test_fk_matches_fixtures(control_loop) -> None:
    poses = _load_poses()
    kin = control_loop.kin
    for key in ("zero", "a", "b", "c", "d"):
        q = np.array(poses[key]["joints"], dtype=float)
        fk = kin.forward(q)
        _assert_pose(fk, poses[key])


def test_jacobian_finite_difference(control_loop) -> None:
    poses = _load_poses()
    q = np.array(poses["b"]["joints"], dtype=float)
    kin = control_loop.kin
    j_ana = kin.jacobian(q)
    fk0 = kin.forward(q)
    p0, r0 = fk0.position, fk0.rotation
    j_num = np.zeros((6, 6))
    h = 1e-6
    for i in range(6):
        dq = np.zeros(6)
        dq[i] = h
        fk_p = kin.forward(q + dq)
        fk_m = kin.forward(q - dq)
        j_num[0:3, i] = (fk_p.position - fk_m.position) / (2 * h)
        r_delta = fk_p.rotation @ fk_m.rotation.T
        theta = math.acos(max(-1.0, min(1.0, (np.trace(r_delta) - 1.0) / 2.0)))
        if theta < 1e-8:
            j_num[3:6, i] = 0.0
        else:
            skew = (r_delta - r_delta.T) / (2 * math.sin(theta))
            axis = np.array([skew[2, 1], skew[0, 2], skew[1, 0]])
            j_num[3:6, i] = axis * theta / (2 * h)
    assert np.max(np.abs(j_ana - j_num)) < 1e-4


def test_ik_round_trip(control_loop) -> None:
    poses = _load_poses()
    kin = control_loop.kin
    seed = np.array(config.Q_HOME, dtype=float)
    for key in ("b", "c"):
        q_tgt = np.array(poses[key]["joints"], dtype=float)
        fk = kin.forward(q_tgt)
        q_sol, ok, _, pe, oe, _, _ = kin.solve_ik(seed, fk.position, fk.rotation)
        assert ok
        fk2 = kin.forward(q_sol)
        assert np.linalg.norm(fk2.position - fk.position) < 0.002
        e_o = orientation_error_vec(fk.rotation, fk2.rotation)
        assert np.linalg.norm(e_o) < 0.01


def test_near_pi_orientation_error_finite(control_loop) -> None:
    kin = control_loop.kin
    q = np.array([0.0, 0.6, 1.4, 0.0, math.pi - 0.001, 0.0])
    fk = kin.forward(q)
    r_target = fk.rotation @ kin.quat_to_matrix(0, 1, 0, 0)
    e_o = orientation_error_vec(r_target, fk.rotation)
    assert np.all(np.isfinite(e_o))
    _, ok, _, _, _, _, _ = kin.solve_ik(np.array(config.Q_HOME), fk.position, r_target)
    assert ok


def test_home_and_fixtures_safe(control_loop) -> None:
    poses = _load_poses()
    kin = control_loop.kin
    for key in ("a", "b", "c", "d"):
        q = np.array(poses[key]["joints"], dtype=float)
        fk = kin.forward(q)
        assert in_envelope(fk.position)
        assert not check_keepout(kin, q, config.DEFAULT_KEEPOUT_ZONES)


def test_damping_increases_near_singularity(control_loop) -> None:
    poses = _load_poses()
    qd = np.array(poses["d"]["joints"], dtype=float)
    kin = control_loop.kin
    fk_d = kin.forward(qd)
    s = np.linalg.svd(kin.jacobian(qd), compute_uv=False)
    assert float(s[-1]) < 0.01
    target_p = fk_d.position + np.array([0.02, 0.0, 0.0])
    _, _, _, _, _, lam, _ = kin.solve_ik(qd, target_p, fk_d.rotation)
    assert lam > 0.05


def test_joint_limit_active_set(control_loop) -> None:
    kin = control_loop.kin
    q = np.array([3.1416, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)
    fk = kin.forward(q)
    target_p = fk.position + np.array([0.5, 0.0, 0.0])
    for _ in range(50):
        q, _, _, _, _, _, _ = kin.solve_ik(q, target_p, fk.rotation)
        assert q[0] <= 3.1416 + 1e-5
    target_p = fk.position + np.array([-0.5, 0.0, 0.0])
    q_before = q[0]
    for _ in range(20):
        q, _, _, _, _, _, _ = kin.solve_ik(q, target_p, fk.rotation)
    assert q[0] < q_before

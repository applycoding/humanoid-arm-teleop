from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Literal

import numpy as np

from backend import config
from backend.config import BoxKeepOut, SphereKeepOut
from backend.kinematics import ArmKinematics


class SafetyMode(str, Enum):
    RUNNING = "RUNNING"
    HOLD = "HOLD"
    ESTOP = "ESTOP"
    FAULT = "FAULT"
    REPLAYING = "REPLAYING"


TargetStatus = Literal[
    "OK",
    "UNREACHABLE",
    "REJECTED_LIMITS",
    "REJECTED_KEEPOUT",
    "REJECTED_ENVELOPE",
]


@dataclass
class SafetyState:
    estop: bool = False
    fault: bool = False
    mode: SafetyMode = SafetyMode.HOLD
    deadman_ok: bool = False
    watchdog_ok: bool = True
    owner_connected: bool = False
    replaying: bool = False
    consecutive_overruns: int = 0
    pose_received_since_hold: bool = False


@dataclass
class TickContext:
    q_committed: np.ndarray
    q_velocity: np.ndarray
    target_position: np.ndarray
    target_rotation: np.ndarray
    safety: SafetyState
    target_status: TargetStatus = "OK"
    ik_converged: bool = False
    ik_iterations: int = 0
    ik_pos_err: float = 0.0
    ik_ori_err: float = 0.0
    ik_near_singularity: bool = False
    ik_lambda: float = 0.05


def _capsule_point_distance(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom < 1e-12:
        return float(np.linalg.norm(p - a))
    t = max(0.0, min(1.0, float(np.dot(p - a, ab) / denom)))
    closest = a + t * ab
    return float(np.linalg.norm(p - closest))


def _capsule_sphere_collision(
    a: np.ndarray, b: np.ndarray, radius: float, center: np.ndarray, sphere_r: float, padding: float
) -> bool:
    dist = _capsule_point_distance(center, a, b)
    return dist < (radius + sphere_r + padding)


def _capsule_box_collision(
    a: np.ndarray, b: np.ndarray, radius: float, bmin: np.ndarray, bmax: np.ndarray, padding: float
) -> bool:
    samples = [a, b, (a + b) / 2.0]
    for p in samples:
        if (
            bmin[0] - padding - radius <= p[0] <= bmax[0] + padding + radius
            and bmin[1] - padding - radius <= p[1] <= bmax[1] + padding + radius
            and bmin[2] - padding - radius <= p[2] <= bmax[2] + padding + radius
        ):
            return True
    return False


def check_keepout(kin: ArmKinematics, q: np.ndarray, zones: tuple[SphereKeepOut | BoxKeepOut, ...]) -> bool:
    segments = kin.link_capsule_endpoints(q)
    r = config.CAPSULE_RADIUS_M
    pad = config.KEEPOUT_CLEARANCE_M
    for a, b in segments:
        for zone in zones:
            if zone.kind == "sphere":
                c = np.array(zone.center, dtype=float)
                if _capsule_sphere_collision(a, b, r, c, zone.radius, pad):
                    return True
            else:
                bmin = np.array(zone.min_corner, dtype=float)
                bmax = np.array(zone.max_corner, dtype=float)
                if _capsule_box_collision(a, b, r, bmin, bmax, pad):
                    return True
    return False


def in_envelope(pos: np.ndarray) -> bool:
    mn = config.WORKSPACE_MIN
    mx = config.WORKSPACE_MAX
    return mn[0] <= pos[0] <= mx[0] and mn[1] <= pos[1] <= mx[1] and mn[2] <= pos[2] <= mx[2]


def velocity_bound(
    kin: ArmKinematics,
    q_from: np.ndarray,
    q_to: np.ndarray,
    dt: float,
    velocity_limits: list[float],
) -> np.ndarray:
    dq = q_to - q_from
    if dt <= 0:
        return q_from.copy()
    scale = 1.0
    for i, vlim in enumerate(velocity_limits):
        max_dq = abs(vlim * dt)
        if max_dq > 0 and abs(dq[i]) > max_dq:
            scale = min(scale, max_dq / abs(dq[i]))
    dq *= scale
    q_bounded = q_from + dq

    fk0 = kin.forward(q_from)
    fk1 = kin.forward(q_bounded)
    dp = fk1.position - fk0.position
    linear_speed = float(np.linalg.norm(dp)) / dt
    r_delta = fk1.rotation @ fk0.rotation.T
    ang = math.acos(max(-1.0, min(1.0, (np.trace(r_delta) - 1.0) / 2.0)))
    angular_speed = ang / dt

    lo, hi = 0.0, 1.0
    for _ in range(8):
        if linear_speed <= config.MAX_EE_LINEAR_SPEED_M_S and angular_speed <= config.MAX_EE_ANGULAR_SPEED_RAD_S:
            break
        scale *= 0.5
        dq = (q_to - q_from) * scale
        q_bounded = q_from + dq
        fk1 = kin.forward(q_bounded)
        dp = fk1.position - fk0.position
        linear_speed = float(np.linalg.norm(dp)) / dt
        r_delta = fk1.rotation @ fk0.rotation.T
        ang = math.acos(max(-1.0, min(1.0, (np.trace(r_delta) - 1.0) / 2.0)))
        angular_speed = ang / dt
    return q_bounded


def apply_joint_limit_clamp(q: np.ndarray, limits: list[tuple[float, float]]) -> tuple[np.ndarray, bool]:
    out = q.copy()
    rejected = False
    for i, (lo, hi) in enumerate(limits):
        if out[i] < lo:
            out[i] = lo
            rejected = True
        elif out[i] > hi:
            out[i] = hi
            rejected = True
    return out, rejected

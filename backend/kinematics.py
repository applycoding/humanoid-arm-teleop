from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from backend import config
from backend.urdf_model import JointDef, UrdfArmModel

if TYPE_CHECKING:
    pass


def _rpy_to_matrix(rpy: tuple[float, float, float]) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def _axis_angle_matrix(axis: np.ndarray, theta: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(theta), math.sin(theta)
    cc = 1.0 - c
    return np.array(
        [
            [c + x * x * cc, x * y * cc - z * s, x * z * cc + y * s],
            [y * x * cc + z * s, c + y * y * cc, y * z * cc - x * s],
            [z * x * cc - y * s, z * y * cc + x * s, c + z * z * cc],
        ],
        dtype=float,
    )


def _make_transform(xyz: tuple[float, float, float], rpy: tuple[float, float, float]) -> np.ndarray:
    t = np.eye(4, dtype=float)
    t[:3, :3] = _rpy_to_matrix(rpy)
    t[:3, 3] = xyz
    return t


def _rot_to_quat(r: np.ndarray) -> tuple[float, float, float, float]:
    m = r
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return x / n, y / n, z / n, w / n


def orientation_error_vec(r_target: np.ndarray, r_current: np.ndarray) -> np.ndarray:
    r_err = r_target @ r_current.T
    theta = math.acos(max(-1.0, min(1.0, (np.trace(r_err) - 1.0) / 2.0)))
    if theta < 1e-6:
        return 0.5 * np.array(
            [r_err[2, 1] - r_err[1, 2], r_err[0, 2] - r_err[2, 0], r_err[1, 0] - r_err[0, 1]],
            dtype=float,
        )
    if theta > math.pi - 1e-3:
        diag = np.diag(r_err) + 1.0
        k = int(np.argmax(diag))
        axis = np.zeros(3, dtype=float)
        axis[k] = math.sqrt(max(0.0, diag[k] / 2.0))
        for j in range(3):
            if j != k:
                axis[j] = (r_err[k, j] + r_err[j, k]) / (4.0 * axis[k])
        axis = axis / np.linalg.norm(axis)
        return axis * theta
    skew = (r_err - r_err.T) / (2.0 * math.sin(theta))
    return np.array([skew[2, 1], skew[0, 2], skew[1, 0]], dtype=float) * theta


@dataclass
class FkResult:
    position: np.ndarray
    rotation: np.ndarray
    quat: tuple[float, float, float, float]
    joint_origins: list[np.ndarray]
    joint_axes: list[np.ndarray]
    link_transforms: dict[str, np.ndarray]


class ArmKinematics:
    def __init__(self, model: UrdfArmModel) -> None:
        self.model = model
        self._revolute_joints = [j for j in model.joints if j.joint_type == "revolute"]
        self._chain = self._revolute_joints + [j for j in model.joints if j.joint_type == "fixed" and j.name == "fixed_tool0"]

    def forward(self, q: np.ndarray) -> FkResult:
        if len(q) != len(self._revolute_joints):
            raise ValueError("q length mismatch")
        t_world = np.eye(4, dtype=float)
        joint_origins: list[np.ndarray] = []
        joint_axes: list[np.ndarray] = []
        link_transforms: dict[str, np.ndarray] = {}

        qi = 0
        for joint in self.model.joints:
            if joint.joint_type == "revolute":
                t_joint = _make_transform(joint.origin_xyz, joint.origin_rpy)
                t_world = t_world @ t_joint
                origin = t_world[:3, 3].copy()
                axis_world = t_world[:3, :3] @ np.array(joint.axis, dtype=float)
                axis_world = axis_world / np.linalg.norm(axis_world)
                joint_origins.append(origin)
                joint_axes.append(axis_world)
                r_q = _axis_angle_matrix(np.array(joint.axis, dtype=float), float(q[qi]))
                t_rot = np.eye(4, dtype=float)
                t_rot[:3, :3] = r_q
                t_world = t_world @ t_rot
                link_transforms[joint.child] = t_world.copy()
                qi += 1
            elif joint.joint_type == "fixed":
                t_joint = _make_transform(joint.origin_xyz, joint.origin_rpy)
                t_world = t_world @ t_joint
                link_transforms[joint.child] = t_world.copy()

        pos = t_world[:3, 3].copy()
        rot = t_world[:3, :3].copy()
        quat = _rot_to_quat(rot)
        return FkResult(
            position=pos,
            rotation=rot,
            quat=quat,
            joint_origins=joint_origins,
            joint_axes=joint_axes,
            link_transforms=link_transforms,
        )

    def jacobian(self, q: np.ndarray) -> np.ndarray:
        fk = self.forward(q)
        p_e = fk.position
        j = np.zeros((6, len(self._revolute_joints)), dtype=float)
        for i in range(len(self._revolute_joints)):
            z_i = fk.joint_axes[i]
            p_i = fk.joint_origins[i]
            j[0:3, i] = np.cross(z_i, p_e - p_i)
            j[3:6, i] = z_i
        return j

    def pose_error(self, q: np.ndarray, p_target: np.ndarray, r_target: np.ndarray) -> tuple[np.ndarray, float, float]:
        fk = self.forward(q)
        e_p = p_target - fk.position
        e_o = orientation_error_vec(r_target, fk.rotation)
        pos_err = float(np.linalg.norm(e_p))
        ori_err = float(np.linalg.norm(e_o))
        l_scale = config.CHARACTERISTIC_LENGTH
        e = np.concatenate([e_p, l_scale * e_o])
        return e, pos_err, ori_err

    def solve_ik(
        self,
        q_seed: np.ndarray,
        p_target: np.ndarray,
        r_target: np.ndarray,
        max_iter: int = config.IK_MAX_ITERATIONS,
    ) -> tuple[np.ndarray, bool, int, float, float, float, bool]:
        q = q_seed.copy().astype(float)
        limits = [j.limit for j in self._revolute_joints]
        stall = 0
        last_lambda = 0.05
        max_lambda = 0.05
        near_sing = False

        for it in range(max_iter):
            fk = self.forward(q)
            e_p = p_target - fk.position
            e_o = orientation_error_vec(r_target, fk.rotation)
            pos_err = float(np.linalg.norm(e_p))
            ori_err = float(np.linalg.norm(e_o))
            if pos_err < config.IK_POS_TOL_M and ori_err < config.IK_ORI_TOL_RAD:
                return q, True, it + 1, pos_err, ori_err, max_lambda, near_sing

            j_full = self.jacobian(q)
            l_scale = config.CHARACTERISTIC_LENGTH
            j_scaled = j_full.copy()
            j_scaled[3:6, :] *= l_scale
            e = np.concatenate([e_p, l_scale * e_o])

            s = np.linalg.svd(j_full, compute_uv=False)
            sigma_min = float(s[-1]) if len(s) else 0.0
            if sigma_min < 0.01:
                near_sing = True
                lam = 0.05 + 2.0 * (0.01 - sigma_min)
            else:
                lam = 0.05
            last_lambda = lam
            max_lambda = max(max_lambda, lam)

            a = j_scaled @ j_scaled.T + (lam ** 2) * np.eye(6)
            dq = j_scaled.T @ np.linalg.solve(a, e)

            lim = limits
            for i in range(len(q)):
                if lim[i] is None:
                    continue
                lo, hi = lim[i].lower, lim[i].upper
                at_lo = q[i] <= lo + 1e-6 and dq[i] < 0
                at_hi = q[i] >= hi - 1e-6 and dq[i] > 0
                if at_lo or at_hi:
                    dq[i] = 0.0
                    j_scaled[:, i] = 0.0

            max_step = float(np.max(np.abs(dq)))
            if max_step > 0.5:
                dq *= 0.5 / max_step

            e_norm = float(np.linalg.norm(e))
            dq_try = dq
            accepted = False
            for _ in range(5):
                q_new = q + dq_try
                _, pe, oe = self.pose_error(q_new, p_target, r_target)
                e_new = math.sqrt(pe * pe + (config.CHARACTERISTIC_LENGTH * oe) ** 2)
                if e_new < e_norm:
                    q = q_new
                    accepted = True
                    stall = 0
                    break
                dq_try *= 0.5
            if not accepted:
                stall += 1
                if stall >= 3:
                    break

        fk = self.forward(q)
        e_p = p_target - fk.position
        e_o = orientation_error_vec(r_target, fk.rotation)
        pos_err = float(np.linalg.norm(e_p))
        ori_err = float(np.linalg.norm(e_o))
        converged = pos_err < config.IK_POS_TOL_M and ori_err < config.IK_ORI_TOL_RAD
        return q, converged, max_iter, pos_err, ori_err, max_lambda, near_sing

    def quat_to_matrix(self, qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
        n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if n < 1e-12:
            return np.eye(3)
        qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
        return np.array(
            [
                [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
            ],
            dtype=float,
        )

    def link_capsule_endpoints(self, q: np.ndarray) -> list[tuple[np.ndarray, np.ndarray]]:
        fk = self.forward(q)
        origins = fk.joint_origins
        segments: list[tuple[np.ndarray, np.ndarray]] = []
        for i in range(len(origins) - 1):
            segments.append((origins[i].copy(), origins[i + 1].copy()))
        if origins:
            segments.append((origins[-1].copy(), fk.position.copy()))
        return segments

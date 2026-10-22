from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

import numpy as np

from backend import config
from backend.clients import ClientRegistry
from backend.kinematics import ArmKinematics
from backend.recorder import RecorderError, TrajectoryRecorder
from backend.safety import (
    SafetyMode,
    SafetyState,
    apply_joint_limit_clamp,
    check_keepout,
    in_envelope,
    velocity_bound,
)
from backend.urdf_model import UrdfArmModel, load_urdf


@dataclass
class ControlLoop:
    model: UrdfArmModel
    kin: ArmKinematics
    registry: ClientRegistry
    recorder: TrajectoryRecorder
    mono_clock: Callable[[], float] = time.monotonic
    q_committed: np.ndarray = field(default_factory=lambda: np.array(config.Q_HOME, dtype=float))
    q_velocity: np.ndarray = field(default_factory=lambda: np.zeros(6))
    target_position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    target_rotation: np.ndarray = field(default_factory=lambda: np.eye(3))
    safety: SafetyState = field(default_factory=SafetyState)
    tick_count: int = 0
    sim_time: float = 0.0
    recording_t: float = 0.0
    is_recording: bool = False
    replay_joint_target: np.ndarray | None = None
    publish_callback: Callable[[dict], None] | None = None
    velocity_limits: list[float] = field(default_factory=list)
    joint_limits: list[tuple[float, float]] = field(default_factory=list)
    last_tick_duration_ms: float = 0.0

    @classmethod
    def create(cls, root: Path, registry: ClientRegistry | None = None) -> ControlLoop:
        urdf_path = root / config.URDF_PATH
        model = load_urdf(urdf_path)
        kin = ArmKinematics(model)
        rec = TrajectoryRecorder(root / "recordings", urdf_path, model.revolute_names)
        reg = registry or ClientRegistry()
        loop = cls(
            model=model,
            kin=kin,
            registry=reg,
            recorder=rec,
        )
        rev = [j for j in model.joints if j.joint_type == "revolute"]
        loop.velocity_limits = [j.limit.velocity for j in rev if j.limit]
        loop.joint_limits = [(j.limit.lower, j.limit.upper) for j in rev if j.limit]
        fk = kin.forward(loop.q_committed)
        loop.target_position = fk.position.copy()
        loop.target_rotation = fk.rotation.copy()
        loop.safety.mode = SafetyMode.HOLD
        return loop

    def sync_target_to_committed(self) -> None:
        fk = self.kin.forward(self.q_committed)
        self.target_position = fk.position.copy()
        self.target_rotation = fk.rotation.copy()

    def engage_estop(self) -> None:
        self.safety.estop = True
        self.safety.mode = SafetyMode.ESTOP
        self.recorder.cancel_replay()
        self.safety.replaying = False

    def reset_from_fault_or_estop(self) -> None:
        self.safety.estop = False
        self.safety.fault = False
        self.safety.mode = SafetyMode.HOLD
        self.safety.pose_received_since_hold = False
        self.sync_target_to_committed()
        self.recorder.cancel_replay()
        self.safety.replaying = False

    def owner_deadman_ok(self, now: float) -> bool:
        oid = self.registry.owner_id
        if oid is None:
            return False
        st = self.registry.get(oid)
        if st is None:
            return False
        return st.deadman_held and now <= st.deadman_expiry_mono

    def update_mode_transitions(self, deadman_ok: bool) -> None:
        if self.safety.estop:
            self.safety.mode = SafetyMode.ESTOP
            return
        if self.safety.fault:
            self.safety.mode = SafetyMode.FAULT
            return
        if self.safety.replaying or self.safety.mode == SafetyMode.REPLAYING:
            self.safety.mode = SafetyMode.REPLAYING
            return
        if not deadman_ok:
            self.safety.mode = SafetyMode.HOLD
            return
        if (
            self.safety.mode == SafetyMode.HOLD
            and deadman_ok
            and self.safety.pose_received_since_hold
        ):
            self.safety.mode = SafetyMode.RUNNING

    def tick(self, dt: float = config.CONTROL_DT, tick_work_ms: float | None = None) -> None:
        t_start = self.mono_clock()
        self.tick_count += 1
        self.sim_time += dt
        now = self.mono_clock()

        if tick_work_ms is not None:
            work = tick_work_ms
        else:
            work = (now - t_start) * 1000.0
        if work > config.TICK_OVERRUN_FAULT_MS:
            self.safety.consecutive_overruns += 1
            if self.safety.consecutive_overruns >= config.CONSECUTIVE_OVERRUNS_FOR_FAULT:
                self.safety.fault = True
        else:
            self.safety.consecutive_overruns = 0

        if not np.all(np.isfinite(self.q_committed)):
            self.safety.fault = True

        self.safety.owner_connected = self.registry.owner_connected()
        deadman_ok = self.owner_deadman_ok(now) if self.safety.mode != SafetyMode.REPLAYING else True
        self.safety.deadman_ok = deadman_ok
        self.safety.watchdog_ok = True

        target_status = "OK"
        ik_converged = False
        ik_iterations = 0
        ik_pos_err = 0.0
        ik_ori_err = 0.0
        ik_near_sing = False

        if self.safety.estop or self.safety.fault:
            self.q_velocity[:] = 0.0
            self._publish_state(target_status, ik_converged, ik_iterations, ik_pos_err, ik_ori_err, ik_near_sing)
            return

        if self.safety.mode == SafetyMode.REPLAYING:
            if self.replay_joint_target is None:
                nxt = self.recorder.next_replay_target()
                if nxt is None:
                    self.safety.replaying = False
                    self.safety.mode = SafetyMode.HOLD
                    self.recorder.cancel_replay()
                    self._publish_state("OK", False, 0, 0.0, 0.0, False)
                    return
                self.replay_joint_target = np.array(nxt, dtype=float)

            q_goal = self.replay_joint_target
            q_bounded = velocity_bound(self.kin, self.q_committed, q_goal, dt, self.velocity_limits)
            q_bounded, lim_rej = apply_joint_limit_clamp(q_bounded, self.joint_limits)
            if lim_rej:
                target_status = "REJECTED_LIMITS"
            fk_b = self.kin.forward(q_bounded)
            if not in_envelope(fk_b.position):
                target_status = "REJECTED_ENVELOPE"
                self._publish_state(target_status, True, 0, 0.0, 0.0, False)
                return
            if check_keepout(self.kin, q_bounded, config.DEFAULT_KEEPOUT_ZONES):
                target_status = "REJECTED_KEEPOUT"
                self._publish_state(target_status, True, 0, 0.0, 0.0, False)
                return
            if np.allclose(q_bounded, q_goal, atol=1e-6):
                self.replay_joint_target = None
            self._commit(q_bounded, dt, target_status, True, 0, 0.0, 0.0, False)
            return

        if not deadman_ok or not self.registry.owner_connected():
            self.safety.mode = SafetyMode.HOLD
            self.q_velocity[:] = 0.0
            self.update_mode_transitions(deadman_ok)
            self._publish_state("OK", False, 0, 0.0, 0.0, False)
            return

        self.update_mode_transitions(deadman_ok)
        if self.safety.mode != SafetyMode.RUNNING:
            self.q_velocity[:] = 0.0
            self._publish_state("OK", False, 0, 0.0, 0.0, False)
            return

        if not np.all(np.isfinite(self.q_committed)):
            self.safety.fault = True
            self._publish_state("OK", False, 0, 0.0, 0.0, False)
            return

        q_cand, converged, iters, pe, oe, lam, near_sing = self.kin.solve_ik(
            self.q_committed, self.target_position, self.target_rotation
        )
        ik_iterations = iters
        ik_pos_err = pe
        ik_ori_err = oe
        ik_near_sing = near_sing
        ik_converged = converged

        if not converged:
            target_status = "UNREACHABLE"
            self._publish_state(target_status, ik_converged, ik_iterations, ik_pos_err, ik_ori_err, ik_near_sing)
            return

        q_bounded = velocity_bound(self.kin, self.q_committed, q_cand, dt, self.velocity_limits)
        q_bounded, lim_rej = apply_joint_limit_clamp(q_bounded, self.joint_limits)
        if lim_rej:
            target_status = "REJECTED_LIMITS"

        fk_b = self.kin.forward(q_bounded)
        if not in_envelope(fk_b.position):
            self._publish_state("REJECTED_ENVELOPE", ik_converged, ik_iterations, ik_pos_err, ik_ori_err, ik_near_sing)
            return
        if check_keepout(self.kin, q_bounded, config.DEFAULT_KEEPOUT_ZONES):
            self._publish_state("REJECTED_KEEPOUT", ik_converged, ik_iterations, ik_pos_err, ik_ori_err, ik_near_sing)
            return

        self._commit(q_bounded, dt, target_status, ik_converged, ik_iterations, ik_pos_err, ik_ori_err, ik_near_sing)

    def _commit(
        self,
        q_bounded: np.ndarray,
        dt: float,
        target_status: str,
        ik_converged: bool,
        ik_iterations: int,
        ik_pos_err: float,
        ik_ori_err: float,
        ik_near_sing: bool,
    ) -> None:
        if not np.all(np.isfinite(q_bounded)):
            self.safety.fault = True
            return
        self.q_velocity = (q_bounded - self.q_committed) / dt
        self.q_committed = q_bounded.copy()
        self._publish_state(target_status, ik_converged, ik_iterations, ik_pos_err, ik_ori_err, ik_near_sing)
        if self.is_recording:
            self.recording_t += dt
            self.recorder.record_tick(
                self.recording_t,
                self.q_committed.tolist(),
                self.safety.mode.value,
            )

    def _publish_state(
        self,
        target_status: str,
        ik_converged: bool,
        ik_iterations: int,
        ik_pos_err: float,
        ik_ori_err: float,
        ik_near_sing: bool,
    ) -> None:
        if self.publish_callback is None:
            return
        stamp = self.mono_clock()
        fk = self.kin.forward(self.q_committed)
        qx, qy, qz, qw = fk.quat
        payload = {
            "joint_states": {
                "topic": "/joint_states",
                "stamp": stamp,
                "name": self.model.revolute_names,
                "position": self.q_committed.tolist(),
                "velocity": self.q_velocity.tolist(),
            },
            "ee_pose": {
                "topic": "/ee_pose",
                "stamp": stamp,
                "position": {"x": float(fk.position[0]), "y": float(fk.position[1]), "z": float(fk.position[2])},
                "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
                "frame_id": "base_link",
            },
            "ik_status": {
                "topic": "/ik_status",
                "stamp": stamp,
                "converged": ik_converged,
                "iterations": ik_iterations,
                "position_error_m": ik_pos_err,
                "orientation_error_rad": ik_ori_err,
                "near_singularity": ik_near_sing,
                "target_status": target_status,
            },
            "safety_state": {
                "topic": "/safety_state",
                "stamp": stamp,
                "estop": self.safety.estop,
                "deadman_ok": self.safety.deadman_ok,
                "watchdog_ok": self.safety.watchdog_ok,
                "mode": self.safety.mode.value,
                "owner_connected": self.safety.owner_connected,
                "replaying": self.safety.replaying,
            },
        }
        self.publish_callback(payload)

    def start_recording(self, name: str) -> None:
        self.recorder.start(name)
        self.is_recording = True
        self.recording_t = 0.0

    def stop_recording(self) -> None:
        self.recorder.stop()
        self.is_recording = False

    def start_replay(self, name: str) -> None:
        if self.safety.mode != SafetyMode.HOLD:
            raise RecorderError("replay requires HOLD")
        self.recorder.load_replay(name)
        self.safety.replaying = True
        self.safety.mode = SafetyMode.REPLAYING
        self.replay_joint_target = None

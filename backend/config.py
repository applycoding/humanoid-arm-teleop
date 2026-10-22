from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

CONTROL_HZ = 50
CONTROL_DT = 1.0 / CONTROL_HZ

DEADMAN_LEASE_MS = 250
TICK_OVERRUN_FAULT_MS = 40
CONSECUTIVE_OVERRUNS_FOR_FAULT = 3

CHARACTERISTIC_LENGTH = 0.3

IK_MAX_ITERATIONS = 50
IK_POS_TOL_M = 0.002
IK_ORI_TOL_RAD = 0.01

MAX_EE_LINEAR_SPEED_M_S = 1.0
MAX_EE_ANGULAR_SPEED_RAD_S = 2.0

CAPSULE_RADIUS_M = 0.04
KEEPOUT_CLEARANCE_M = 0.03

WORKSPACE_MIN = (-0.8, -0.8, 0.0)
WORKSPACE_MAX = (0.8, 0.8, 1.2)

Q_HOME = [0.0, 0.6, 1.4, 0.0, 0.9, 0.0]

URDF_PATH = "arm/arm.urdf"

RECORDING_MAX_BYTES = 10 * 1024 * 1024


@dataclass(frozen=True)
class SphereKeepOut:
    kind: Literal["sphere"] = "sphere"
    center: tuple[float, float, float] = (0.30, -0.45, 0.65)
    radius: float = 0.12


@dataclass(frozen=True)
class BoxKeepOut:
    kind: Literal["box"] = "box"
    min_corner: tuple[float, float, float] = (0.35, -0.25, 0.0)
    max_corner: tuple[float, float, float] = (0.65, 0.25, 0.08)


DEFAULT_KEEPOUT_ZONES: tuple[SphereKeepOut | BoxKeepOut, ...] = (
    SphereKeepOut(),
    BoxKeepOut(),
)

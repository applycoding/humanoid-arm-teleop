from __future__ import annotations

import re
from typing import Annotated, Literal, Union

from pydantic import BaseModel, Field, field_validator

RECORD_NAME_RE = re.compile(r"^[a-zA-Z0-9_-]{1,64}$")


class Vec3(BaseModel):
    x: float
    y: float
    z: float


class Quat(BaseModel):
    x: float
    y: float
    z: float
    w: float


class CmdControlRequest(BaseModel):
    topic: Literal["/cmd_control_request"]
    seq: int


class CmdControlAck(BaseModel):
    topic: Literal["/cmd_control_ack"] = "/cmd_control_ack"
    seq: int
    stamp: float
    granted: bool


class CmdPose(BaseModel):
    topic: Literal["/cmd_pose"]
    seq: int
    stamp: float = 0.0
    position: Vec3
    orientation: Quat
    frame_id: str

    @field_validator("frame_id")
    @classmethod
    def frame_must_be_base(cls, v: str) -> str:
        if v != "base_link":
            raise ValueError("frame_id must be base_link")
        return v


class CmdDeadmanHeartbeat(BaseModel):
    topic: Literal["/cmd_deadman_heartbeat"]
    seq: int
    held: bool


class CmdEstop(BaseModel):
    topic: Literal["/cmd_estop"]
    seq: int
    engaged: bool


class CmdReset(BaseModel):
    topic: Literal["/cmd_reset"]
    seq: int


class CmdRecord(BaseModel):
    topic: Literal["/cmd_record"]
    seq: int
    action: Literal["start", "stop", "replay", "cancel_replay"]
    name: str

    @field_validator("name")
    @classmethod
    def valid_name(cls, v: str) -> str:
        if not RECORD_NAME_RE.match(v):
            raise ValueError("invalid recording name")
        return v


class JointStates(BaseModel):
    topic: Literal["/joint_states"] = "/joint_states"
    seq: int
    stamp: float
    name: list[str]
    position: list[float]
    velocity: list[float]


class EePose(BaseModel):
    topic: Literal["/ee_pose"] = "/ee_pose"
    seq: int
    stamp: float
    position: Vec3
    orientation: Quat
    frame_id: str = "base_link"


class IkStatus(BaseModel):
    topic: Literal["/ik_status"] = "/ik_status"
    seq: int
    stamp: float
    converged: bool
    iterations: int
    position_error_m: float
    orientation_error_rad: float
    near_singularity: bool
    target_status: str


class SafetyStateMsg(BaseModel):
    topic: Literal["/safety_state"] = "/safety_state"
    seq: int
    stamp: float
    estop: bool
    deadman_ok: bool
    watchdog_ok: bool
    mode: str
    owner_connected: bool
    replaying: bool


class WsError(BaseModel):
    topic: Literal["/ws_error"] = "/ws_error"
    seq: int
    stamp: float
    reason: str


InboundMessage = Annotated[
    Union[CmdControlRequest, CmdPose, CmdDeadmanHeartbeat, CmdEstop, CmdReset, CmdRecord],
    Field(discriminator="topic"),
]


def parse_inbound(data: dict) -> InboundMessage:
    topic = data.get("topic")
    if topic == "/cmd_control_request":
        return CmdControlRequest.model_validate(data)
    if topic == "/cmd_pose":
        return CmdPose.model_validate(data)
    if topic == "/cmd_deadman_heartbeat":
        return CmdDeadmanHeartbeat.model_validate(data)
    if topic == "/cmd_estop":
        return CmdEstop.model_validate(data)
    if topic == "/cmd_reset":
        return CmdReset.model_validate(data)
    if topic == "/cmd_record":
        return CmdRecord.model_validate(data)
    raise ValueError("unknown_topic")


def normalize_quaternion(q: Quat) -> Quat:
    n = (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) ** 0.5
    if n < 1e-6 or not all(map(_finite, (q.x, q.y, q.z, q.w))):
        raise ValueError("invalid quaternion")
    return Quat(x=q.x / n, y=q.y / n, z=q.z / n, w=q.w / n)


def _finite(x: float) -> bool:
    return x == x and abs(x) != float("inf")

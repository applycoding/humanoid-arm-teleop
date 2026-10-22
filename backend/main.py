from __future__ import annotations

import asyncio
import json
import time
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from backend import config
from backend.clients import ClientRegistry, ConnectionState
from backend.control_loop import ControlLoop
import numpy as np

from backend.messages import (
    CmdControlAck,
    CmdEstop,
    CmdPose,
    WsError,
    normalize_quaternion,
    parse_inbound,
)
from backend.safety import SafetyMode
from backend.recorder import RecorderError

ROOT = Path(__file__).resolve().parent.parent


class AppState:
    def __init__(self) -> None:
        self.registry = ClientRegistry()
        self.loop = ControlLoop.create(ROOT, self.registry)
        self.loop_task: asyncio.Task | None = None
        self._out_seq = 0

    def next_seq(self) -> int:
        self._out_seq += 1
        return self._out_seq


state = AppState()


def _broadcast(payload: dict) -> None:
    seq = state.next_seq()
    stamp = time.monotonic()
    for conn in state.registry.all_connections():
        for topic, body in payload.items():
            msg = dict(body)
            msg["seq"] = seq
            msg["stamp"] = stamp
            conn.mailboxes[topic] = msg


state.loop.publish_callback = _broadcast


async def _control_task() -> None:
    while True:
        await asyncio.sleep(config.CONTROL_DT)
        state.loop.tick(config.CONTROL_DT)


async def _flush_connection(conn: ConnectionState) -> None:
    ws = conn.websocket
    while conn.mailboxes:
        for topic in list(conn.mailboxes.keys()):
            msg = conn.mailboxes.pop(topic, None)
            if msg is None:
                continue
            try:
                await ws.send_json(msg)
            except Exception:
                return


@asynccontextmanager
async def lifespan(app: FastAPI):
    state.loop_task = asyncio.create_task(_control_task())
    yield
    if state.loop_task:
        state.loop_task.cancel()
        try:
            await state.loop_task
        except asyncio.CancelledError:
            pass


app = FastAPI(lifespan=lifespan)


@app.get("/arm/arm.urdf")
def get_urdf():
    return FileResponse(ROOT / "arm" / "arm.urdf", media_type="application/xml")


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    conn = state.registry.register(websocket)
    flush_task = asyncio.create_task(_connection_flush_loop(conn))

    try:
        while True:
            raw = await websocket.receive_text()
            try:
                data = json.loads(raw)
                await handle_message(conn, data)
            except Exception:
                await send_error(conn, "invalid_message")
    except WebSocketDisconnect:
        pass
    finally:
        flush_task.cancel()
        was_owner = state.registry.is_owner(conn.conn_id)
        state.registry.release_owner(conn.conn_id)
        state.registry.unregister(conn.conn_id)
        if was_owner:
            state.loop.safety.mode = SafetyMode.HOLD


async def _connection_flush_loop(conn: ConnectionState) -> None:
    while True:
        if conn.mailboxes:
            await _flush_connection(conn)
        await asyncio.sleep(0.001)


async def send_error(conn: ConnectionState, reason: str) -> None:
    msg = WsError(seq=state.next_seq(), stamp=time.monotonic(), reason=reason)
    await conn.websocket.send_json(msg.model_dump())


async def handle_message(conn: ConnectionState, data: dict) -> None:
    topic = data.get("topic")
    if topic is None:
        await send_error(conn, "invalid_message")
        return
    try:
        msg = parse_inbound(data)
    except Exception:
        await send_error(conn, "invalid_message")
        return

    now = time.monotonic()
    loop = state.loop

    if isinstance(msg, CmdEstop):
        if msg.engaged:
            loop.engage_estop()
        return

    if topic == "/cmd_control_request":
        granted = state.registry.request_control(conn.conn_id)
        ack = CmdControlAck(seq=msg.seq, stamp=now, granted=granted)
        await conn.websocket.send_json(ack.model_dump())
        return

    if not state.registry.is_owner(conn.conn_id):
        if topic != "/cmd_estop":
            await send_error(conn, "not_owner")
        return

    if topic == "/cmd_deadman_heartbeat":
        conn.deadman_held = msg.held
        if msg.held:
            conn.deadman_expiry_mono = now + config.DEADMAN_LEASE_MS / 1000.0
        else:
            conn.deadman_expiry_mono = 0.0
        return

    if topic == "/cmd_reset":
        loop.reset_from_fault_or_estop()
        return

    if topic == "/cmd_pose":
        try:
            if not all(
                map(
                    lambda v: v == v and abs(v) != float("inf"),
                    (
                        msg.position.x,
                        msg.position.y,
                        msg.position.z,
                        msg.orientation.x,
                        msg.orientation.y,
                        msg.orientation.z,
                        msg.orientation.w,
                    ),
                )
            ):
                raise ValueError("non-finite")
            qn = normalize_quaternion(msg.orientation)
            loop.target_position = np.array(
                [msg.position.x, msg.position.y, msg.position.z], dtype=float
            )
            loop.target_rotation = loop.kin.quat_to_matrix(qn.x, qn.y, qn.z, qn.w)
            loop.safety.pose_received_since_hold = True
        except Exception:
            await send_error(conn, "invalid_message")
        return

    if topic == "/cmd_record":
        try:
            if msg.action == "start":
                loop.start_recording(msg.name)
            elif msg.action == "stop":
                loop.stop_recording()
            elif msg.action == "replay":
                loop.start_replay(msg.name)
            elif msg.action == "cancel_replay":
                loop.recorder.cancel_replay()
                loop.safety.replaying = False
                loop.safety.mode = SafetyMode.HOLD
        except RecorderError as exc:
            await conn.websocket.send_json(
                {"topic": "/ws_error", "seq": state.next_seq(), "stamp": now, "reason": str(exc)}
            )
        return


app.mount("/", StaticFiles(directory=str(ROOT / "web"), html=True), name="static")


def get_app() -> FastAPI:
    return app

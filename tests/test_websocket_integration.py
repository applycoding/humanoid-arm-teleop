from __future__ import annotations

import json
from pathlib import Path

from fastapi.testclient import TestClient

from backend.main import app


def test_observer_cannot_pose() -> None:
    client = TestClient(app)
    with client.websocket_connect("/ws") as ws_owner:
        ws_owner.send_json({"topic": "/cmd_control_request", "seq": 1})
        ack = ws_owner.receive_json()
        assert ack["granted"] is True
    with client.websocket_connect("/ws") as ws_obs:
        ws_obs.send_json({"topic": "/cmd_pose", "seq": 2, "position": {"x": 0.4, "y": 0, "z": 0.3}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}, "frame_id": "base_link"})
        err = ws_obs.receive_json()
        assert err["topic"] == "/ws_error"
        assert err["reason"] == "not_owner"


def test_observer_estop_accepted() -> None:
    client = TestClient(app)
    with client.websocket_connect("/ws") as ws_owner:
        ws_owner.send_json({"topic": "/cmd_control_request", "seq": 1})
    with client.websocket_connect("/ws") as ws_obs:
        ws_obs.send_json({"topic": "/cmd_estop", "seq": 3, "engaged": True})

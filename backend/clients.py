from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
from typing import Any


@dataclass
class ConnectionState:
    conn_id: int
    websocket: Any
    is_owner: bool = False
    deadman_held: bool = False
    deadman_expiry_mono: float = 0.0
    outbound_seq: int = 0
    mailboxes: dict[str, dict] = field(default_factory=dict)
    send_task: asyncio.Task | None = None


class ClientRegistry:
    def __init__(self) -> None:
        self._connections: dict[int, ConnectionState] = {}
        self._owner_id: int | None = None
        self._next_id = 1

    def register(self, websocket: Any) -> ConnectionState:
        cid = self._next_id
        self._next_id += 1
        state = ConnectionState(conn_id=cid, websocket=websocket)
        self._connections[cid] = state
        return state

    def unregister(self, conn_id: int) -> None:
        if conn_id in self._connections:
            del self._connections[conn_id]
        if self._owner_id == conn_id:
            self._owner_id = None

    @property
    def owner_id(self) -> int | None:
        return self._owner_id

    def request_control(self, conn_id: int) -> bool:
        if self._owner_id is not None and self._owner_id != conn_id:
            return False
        self._owner_id = conn_id
        st = self._connections.get(conn_id)
        if st:
            st.is_owner = True
        return True

    def release_owner(self, conn_id: int) -> None:
        if self._owner_id == conn_id:
            self._owner_id = None
        st = self._connections.get(conn_id)
        if st:
            st.is_owner = False
            st.deadman_held = False

    def is_owner(self, conn_id: int) -> bool:
        return self._owner_id == conn_id

    def all_connections(self) -> list[ConnectionState]:
        return list(self._connections.values())

    def get(self, conn_id: int) -> ConnectionState | None:
        return self._connections.get(conn_id)

    def owner_connected(self) -> bool:
        return self._owner_id is not None and self._owner_id in self._connections

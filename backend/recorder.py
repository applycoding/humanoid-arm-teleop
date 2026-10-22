from __future__ import annotations

import hashlib
import json
import re
from pathlib import Path
from typing import Callable

from backend import config
from backend.safety import SafetyMode

FILENAME_RE = re.compile(r"^[a-zA-Z0-9_-]{1,64}\.jsonl$")


class RecorderError(Exception):
    pass


class TrajectoryRecorder:
    def __init__(self, recordings_dir: Path, urdf_path: Path, joint_names: list[str]) -> None:
        self.recordings_dir = recordings_dir
        self.urdf_path = urdf_path
        self.joint_names = joint_names
        self._file = None
        self._bytes_written = 0
        self._t0 = 0.0
        self._current_name: str | None = None

        self.replay_samples: list[dict] = []
        self.replay_index = 0
        self.replay_active = False
        self.replay_error: str | None = None

    def urdf_sha256(self) -> str:
        data = self.urdf_path.read_bytes()
        return hashlib.sha256(data).hexdigest()

    def start(self, name: str) -> None:
        if not re.match(r"^[a-zA-Z0-9_-]{1,64}$", name):
            raise RecorderError("invalid name")
        path = self.recordings_dir / f"{name}.jsonl"
        header = {
            "schema_version": 1,
            "joint_names": self.joint_names,
            "urdf_sha256": self.urdf_sha256(),
            "tick_dt": config.CONTROL_DT,
        }
        self._file = path.open("w", encoding="utf-8")
        line = json.dumps(header) + "\n"
        self._file.write(line)
        self._bytes_written = len(line.encode("utf-8"))
        self._t0 = 0.0
        self._current_name = name

    def stop(self) -> None:
        if self._file:
            self._file.close()
            self._file = None
        self._current_name = None

    def record_tick(self, t: float, joint_position: list[float], safety_mode: str) -> None:
        if not self._file:
            return
        row = {"t": t, "joint_position": joint_position, "safety_mode": safety_mode}
        line = json.dumps(row) + "\n"
        enc = line.encode("utf-8")
        if self._bytes_written + len(enc) > config.RECORDING_MAX_BYTES:
            self.stop()
            return
        self._file.write(line)
        self._bytes_written += len(enc)

    def load_replay(self, name: str) -> None:
        path = self.recordings_dir / f"{name}.jsonl"
        if not path.exists():
            raise RecorderError(f"recording {name} not found")
        lines = path.read_text(encoding="utf-8").strip().split("\n")
        if not lines:
            raise RecorderError("empty recording")
        header = json.loads(lines[0])
        if header.get("urdf_sha256") != self.urdf_sha256():
            raise RecorderError("urdf_sha256 mismatch")
        self.replay_samples = [json.loads(ln) for ln in lines[1:]]
        self.replay_index = 0
        self.replay_active = True
        self.replay_error = None

    def cancel_replay(self) -> None:
        self.replay_active = False
        self.replay_index = 0
        self.replay_samples = []

    def next_replay_target(self) -> list[float] | None:
        if not self.replay_active or self.replay_index >= len(self.replay_samples):
            return None
        sample = self.replay_samples[self.replay_index]
        self.replay_index += 1
        return list(sample["joint_position"])

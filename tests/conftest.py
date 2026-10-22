from __future__ import annotations

from pathlib import Path

import pytest

from backend.clients import ClientRegistry
from backend.control_loop import ControlLoop


@pytest.fixture
def repo_root() -> Path:
    return Path(__file__).resolve().parent.parent


@pytest.fixture
def registry() -> ClientRegistry:
    return ClientRegistry()


@pytest.fixture
def control_loop(repo_root: Path, registry: ClientRegistry) -> ControlLoop:
    clock = {"t": 0.0}

    def mono() -> float:
        return clock["t"]

    loop = ControlLoop.create(repo_root, registry)
    loop.mono_clock = mono
    loop.publish_callback = None
    return loop

#!/usr/bin/env python3
"""One-shot FK fixture generator. Run from repo root: python scripts/generate_poses_fixture.py"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from backend.control_loop import ControlLoop
from backend.clients import ClientRegistry

FIXTURES = {
    "zero": [0, 0, 0, 0, 0, 0],
    "a": [0.0, 0.6, 1.4, 0.0, 0.9, 0.0],
    "b": [0.5, 0.3, 1.0, 0.8, 0.5, 0.2],
    "c": [-0.7, 0.7, 1.0, -0.3, 0.8, 1.0],
    "d": [0.0, 1.5708, 0.0, 0.0, 0.0, 0.0],
}


def main() -> None:
    root = Path(__file__).resolve().parent.parent
    loop = ControlLoop.create(root, ClientRegistry())
    out = {}
    for key, joints in FIXTURES.items():
        fk = loop.kin.forward(np.array(joints, dtype=float))
        qx, qy, qz, qw = fk.quat
        out[key] = {
            "joints": joints,
            "position": {
                "x": float(fk.position[0]),
                "y": float(fk.position[1]),
                "z": float(fk.position[2]),
            },
            "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
        }
    path = root / "tests" / "fixtures" / "poses.json"
    path.write_text(json.dumps(out, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {path}")


if __name__ == "__main__":
    main()

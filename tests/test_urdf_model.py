from __future__ import annotations

from pathlib import Path

import pytest

from backend.urdf_model import UrdfParseError, load_urdf


def test_arm_urdf_joint_table(repo_root: Path) -> None:
    model = load_urdf(repo_root / "arm" / "arm.urdf")
    rev = [j for j in model.joints if j.joint_type == "revolute"]
    fixed = [j for j in model.joints if j.joint_type == "fixed"]
    assert len(rev) == 6
    assert len(fixed) == 1
    assert fixed[0].name == "fixed_tool0"
    names = [j.name for j in rev]
    assert names == [
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
        "joint_6",
    ]
    j1 = rev[0]
    assert j1.parent == "base_link" and j1.child == "shoulder_link"
    assert j1.origin_xyz == (0.0, 0.0, 0.15)
    assert j1.axis == (0.0, 0.0, 1.0)
    assert j1.limit is not None
    assert abs(j1.limit.lower - (-3.1416)) < 1e-4
    assert abs(j1.limit.upper - 3.1416) < 1e-4
    assert j1.limit.velocity == 2.0


def test_rejects_unsupported_features(repo_root: Path, tmp_path: Path) -> None:
    base = (repo_root / "arm" / "arm.urdf").read_text(encoding="utf-8")

    bad_p = base.replace('type="revolute"', 'type="prismatic"', 1)
    (tmp_path / "prismatic.urdf").write_text(bad_p, encoding="utf-8")
    with pytest.raises(UrdfParseError):
        load_urdf(tmp_path / "prismatic.urdf")

    bad = base.replace('type="revolute"', 'type="continuous"', 1)
    (tmp_path / "bad.urdf").write_text(bad, encoding="utf-8")
    with pytest.raises(UrdfParseError):
        load_urdf(tmp_path / "bad.urdf")

    bad2 = base.replace("<axis xyz=\"0 0 1\"/>", "<axis xyz=\"0 0 1\"/><mimic joint=\"j\"/>", 1)
    (tmp_path / "mimic.urdf").write_text(bad2, encoding="utf-8")
    with pytest.raises(UrdfParseError):
        load_urdf(tmp_path / "mimic.urdf")

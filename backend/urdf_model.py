from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Literal


@dataclass(frozen=True)
class JointLimit:
    lower: float
    upper: float
    velocity: float


@dataclass(frozen=True)
class JointDef:
    name: str
    joint_type: Literal["revolute", "fixed"]
    parent: str
    child: str
    origin_xyz: tuple[float, float, float]
    origin_rpy: tuple[float, float, float]
    axis: tuple[float, float, float]
    limit: JointLimit | None


@dataclass
class UrdfArmModel:
    joints: list[JointDef]
    revolute_names: list[str]
    ee_link: str

    @property
    def n_revolute(self) -> int:
        return len(self.revolute_names)


class UrdfParseError(ValueError):
    pass


def _parse_vec3(text: str, default: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> tuple[float, float, float]:
    if not text or not text.strip():
        return default
    parts = text.split()
    if len(parts) != 3:
        raise UrdfParseError(f"expected 3 floats, got {text!r}")
    return float(parts[0]), float(parts[1]), float(parts[2])


def _origin(elem: ET.Element | None) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    if elem is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    xyz = _parse_vec3(elem.get("xyz", "0 0 0"))
    rpy = _parse_vec3(elem.get("rpy", "0 0 0"))
    return xyz, rpy


def load_urdf(path: str | Path) -> UrdfArmModel:
    tree = ET.parse(path)
    root = tree.getroot()
    if root.tag != "robot":
        raise UrdfParseError("root element must be <robot>")

    children_count: dict[str, int] = {}
    joints: list[JointDef] = []

    for joint_elem in root.findall("joint"):
        if joint_elem.find("mimic") is not None:
            raise UrdfParseError("mimic joints are not supported")

        name = joint_elem.get("name")
        if not name:
            raise UrdfParseError("joint missing name")
        jtype = joint_elem.get("type")
        if jtype not in ("revolute", "fixed", "continuous", "prismatic"):
            raise UrdfParseError(f"unsupported joint type {jtype!r}")
        if jtype in ("continuous", "prismatic"):
            raise UrdfParseError(f"joint type {jtype!r} is not supported")

        parent = joint_elem.find("parent")
        child = joint_elem.find("child")
        if parent is None or child is None:
            raise UrdfParseError(f"joint {name} missing parent or child")
        parent_link = parent.get("link")
        child_link = child.get("link")
        if not parent_link or not child_link:
            raise UrdfParseError(f"joint {name} missing link names")

        children_count[parent_link] = children_count.get(parent_link, 0) + 1
        if children_count[parent_link] > 1:
            raise UrdfParseError(f"branching link {parent_link!r} is not supported")

        origin_elem = joint_elem.find("origin")
        xyz, rpy = _origin(origin_elem)

        axis_elem = joint_elem.find("axis")
        axis = _parse_vec3(axis_elem.get("xyz", "1 0 0") if axis_elem is not None else "1 0 0")

        limit: JointLimit | None = None
        limit_elem = joint_elem.find("limit")
        if jtype == "revolute":
            if limit_elem is None:
                raise UrdfParseError(f"revolute joint {name} missing limit")
            limit = JointLimit(
                lower=float(limit_elem.get("lower", "0")),
                upper=float(limit_elem.get("upper", "0")),
                velocity=float(limit_elem.get("velocity", "1.0")),
            )

        joints.append(
            JointDef(
                name=name,
                joint_type=jtype,
                parent=parent_link,
                child=child_link,
                origin_xyz=xyz,
                origin_rpy=rpy,
                axis=axis,
                limit=limit,
            )
        )

    revolute = [j for j in joints if j.joint_type == "revolute"]
    fixed = [j for j in joints if j.joint_type == "fixed"]
    revolute_names = [j.name for j in revolute]

    ee_link = "tool0"
    for j in fixed:
        if j.name == "fixed_tool0":
            ee_link = j.child
            break

    return UrdfArmModel(joints=joints, revolute_names=revolute_names, ee_link=ee_link)


def link_capsule_segments(model: UrdfArmModel) -> list[tuple[str, str, str]]:
    """Return (parent_link, child_link, child_joint_name) for each revolute link segment."""
    segs: list[tuple[str, str, str]] = []
    for j in model.joints:
        if j.joint_type == "revolute":
            segs.append((j.parent, j.child, j.name))
    return segs

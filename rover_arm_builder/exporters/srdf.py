"""MoveIt SRDF generation. Collision disabling is conservative: adjacent links only."""

from __future__ import annotations

import xml.etree.ElementTree as ET

from ..core.assembly import FlatRobot
from ..core.model import Actuation, ControlRole, FrameKind, JointType, Module, ModuleKind, Project
from .urdf import fmt


def arm_chain(robot: FlatRobot, base: str, tip: str) -> list[str]:
    """Return movable joint names from base to tip, or raise ValueError."""
    by_child = {j.child: j for j in robot.joints}
    names: list[str] = []
    current = tip
    while current != base:
        joint = by_child.get(current)
        if joint is None:
            raise ValueError(f"{tip} is not downstream of {base}.")
        if joint.type is not JointType.FIXED:
            names.append(joint.name)
        current = joint.parent
    return list(reversed(names))


def _arm_modules(project: Project, module_ids: list[str]) -> list[Module]:
    return [m for m in project.modules if m.id in module_ids and m.kind is ModuleKind.ARM]


def robot_to_srdf(project: Project, robot: FlatRobot, module_ids: list[str]) -> str:
    root = ET.Element("robot", {"name": robot.name})

    for module in _arm_modules(project, module_ids):
        tool = next((f for f in module.frames if f.kind is FrameKind.TOOL), None)
        if tool is None:
            continue
        base_link = next(l for l in module.links if l.id == module.root_link_id)
        base = f"{module.prefix}{base_link.name}"
        tip = f"{module.prefix}{tool.name}"
        group_name = f"{module.prefix}group".replace("__", "_")
        group = ET.SubElement(root, "group", {"name": group_name})
        ET.SubElement(group, "chain", {"base_link": base, "tip_link": tip})
        chain = set(arm_chain(robot, base, tip))
        by_id = {j.id: f"{module.prefix}{j.name}" for j in module.joints}
        for pose_name in sorted(module.poses):
            state = ET.SubElement(root, "group_state", {"name": pose_name, "group": group_name})
            for joint_id, value in sorted(module.poses[pose_name].items(), key=lambda kv: by_id[kv[0]]):
                if by_id[joint_id] in chain:
                    ET.SubElement(state, "joint", {"name": by_id[joint_id], "value": fmt(value)})

    ET.SubElement(root, "virtual_joint", {
        "name": "parked_base", "type": "fixed", "parent_frame": "world", "child_link": robot.root,
    })
    for joint in sorted(robot.joints, key=lambda j: j.name):
        if joint.actuation is Actuation.PASSIVE:
            ET.SubElement(root, "passive_joint", {"name": joint.name})

    colliding = {l.name for l in robot.links if l.collisions}
    pairs = sorted(
        {tuple(sorted((j.parent, j.child))) for j in robot.joints if j.parent in colliding and j.child in colliding}
    )
    for first, second in pairs:
        ET.SubElement(root, "disable_collisions", {"link1": first, "link2": second, "reason": "Adjacent"})

    ET.indent(root, space="  ")
    return '<?xml version="1.0"?>\n' + ET.tostring(root, encoding="unicode") + "\n"


def arm_joint_names(project: Project, robot: FlatRobot, module_ids: list[str]) -> list[str]:
    return [
        j.name for j in sorted(robot.joints, key=lambda j: j.name)
        if j.role is ControlRole.ARM and j.type is not JointType.FIXED
    ]

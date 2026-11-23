"""Plain URDF serialization with deterministic ordering and number formatting."""

from __future__ import annotations

import xml.etree.ElementTree as ET

from ..core.assembly import FlatJoint, FlatLink, FlatRobot
from ..core.model import Box, Cylinder, GeometryElement, JointType, Mesh, Project, Sphere
from ..core.transforms import Transform, normalized
from ..core.validate import Target, ValidationReport, validate_project

POSE_SNAP = 1e-12


class ExportBlocked(RuntimeError):
    """Export was refused. `issues` holds the actionable reasons."""

    def __init__(self, target: Target, issues: list) -> None:
        self.target = target
        self.issues = issues
        details = "; ".join(i.message for i in issues) or "The model could not be flattened."
        super().__init__(f"Cannot export for {target.value}: {details}")


def fmt(value: float) -> str:
    text = f"{value:.12g}"
    return "0" if float(text) == 0.0 else text


def fmt_pose(value: float) -> str:
    return fmt(0.0 if abs(value) < POSE_SNAP else value)


def _origin(parent: ET.Element, pose: Transform) -> None:
    ET.SubElement(parent, "origin", {
        "xyz": " ".join(fmt_pose(v) for v in pose.t),
        "rpy": " ".join(fmt_pose(v) for v in pose.rpy()),
    })


def mesh_filename(asset: str) -> str:
    return f"{asset}.stl"


def _geometry(parent: ET.Element, item: GeometryElement, package: str) -> None:
    geometry = ET.SubElement(parent, "geometry")
    shape = item.shape
    if isinstance(shape, Box):
        ET.SubElement(geometry, "box", {"size": f"{fmt(shape.x)} {fmt(shape.y)} {fmt(shape.z)}"})
    elif isinstance(shape, Cylinder):
        ET.SubElement(geometry, "cylinder", {"radius": fmt(shape.radius), "length": fmt(shape.length)})
    elif isinstance(shape, Sphere):
        ET.SubElement(geometry, "sphere", {"radius": fmt(shape.radius)})
    elif isinstance(shape, Mesh):
        ET.SubElement(geometry, "mesh", {"filename": f"package://{package}/meshes/{mesh_filename(shape.asset)}"})


def _link(root: ET.Element, link: FlatLink, package: str) -> None:
    node = ET.SubElement(root, "link", {"name": link.name})
    if link.mass is not None:
        inertial = ET.SubElement(node, "inertial")
        _origin(inertial, Transform(t=link.mass.com))
        ET.SubElement(inertial, "mass", {"value": fmt(link.mass.mass)})
        i = link.mass.inertia
        ET.SubElement(inertial, "inertia", {
            "ixx": fmt(i.ixx), "ixy": fmt(i.ixy), "ixz": fmt(i.ixz),
            "iyy": fmt(i.iyy), "iyz": fmt(i.iyz), "izz": fmt(i.izz),
        })
    for visual in sorted(link.visuals, key=lambda v: v.name):
        element = ET.SubElement(node, "visual", {"name": visual.name})
        _origin(element, visual.origin)
        _geometry(element, visual, package)
        if visual.material is not None:
            ET.SubElement(element, "material", {"name": visual.material.name})
    for collision in sorted(link.collisions, key=lambda v: v.name):
        element = ET.SubElement(node, "collision", {"name": collision.name})
        _origin(element, collision.origin)
        _geometry(element, collision, package)


def _joint(root: ET.Element, joint: FlatJoint) -> None:
    node = ET.SubElement(root, "joint", {"name": joint.name, "type": joint.type.value})
    _origin(node, joint.origin)
    ET.SubElement(node, "parent", {"link": joint.parent})
    ET.SubElement(node, "child", {"link": joint.child})
    if joint.type is JointType.FIXED:
        return
    ET.SubElement(node, "axis", {"xyz": " ".join(fmt_pose(v) for v in normalized(joint.axis))})
    limit = joint.limit
    if limit is not None and limit.effort is not None and limit.velocity is not None:
        attrs = {}
        if joint.type is not JointType.CONTINUOUS:
            attrs = {"lower": fmt(limit.lower), "upper": fmt(limit.upper)}
        attrs.update({"effort": fmt(limit.effort), "velocity": fmt(limit.velocity)})
        ET.SubElement(node, "limit", attrs)
    if joint.damping or joint.friction:
        ET.SubElement(node, "dynamics", {"damping": fmt(joint.damping), "friction": fmt(joint.friction)})


def robot_to_urdf(robot: FlatRobot, package: str) -> str:
    root = ET.Element("robot", {"name": robot.name})
    materials = {
        v.material.name: v.material for l in robot.links for v in l.visuals if v.material is not None
    }
    for name in sorted(materials):
        node = ET.SubElement(root, "material", {"name": name})
        ET.SubElement(node, "color", {"rgba": " ".join(fmt(v) for v in materials[name].rgba)})
    for link in sorted(robot.links, key=lambda l: l.name):
        _link(root, link, package)
    for joint in sorted(robot.joints, key=lambda j: j.name):
        _joint(root, joint)
    ET.indent(root, space="  ")
    return '<?xml version="1.0"?>\n' + ET.tostring(root, encoding="unicode") + "\n"


def export_urdf(
    project: Project,
    package: str,
    target: Target = Target.VISUALIZATION,
    module_ids: list[str] | None = None,
    name: str | None = None,
) -> tuple[str, ValidationReport]:
    """Validate, then serialize. Raises ExportBlocked instead of writing a bad model."""
    report = validate_project(project, module_ids, name)
    if not report.ok(target):
        raise ExportBlocked(target, report.blockers(target))
    return robot_to_urdf(report.robot, package), report

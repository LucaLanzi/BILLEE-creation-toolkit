"""Actionable validation of projects and flattened robots."""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from enum import Enum

from .assembly import FlatRobot, flatten
from .mass import inertia_problems
from .model import (
    Actuation,
    Component,
    ControlRole,
    GeometryElement,
    JointType,
    Link,
    Mesh,
    Module,
    Project,
    resolve_mass,
    shape_dimensions,
    shape_half_extents,
)
from .transforms import AffineKind, Transform, classify_linear, norm

NAME_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


class Severity(Enum):
    ERROR = "error"
    WARNING = "warning"
    INFO = "info"


class Target(Enum):
    VISUALIZATION = "visualization"
    PLANNING = "planning"
    PHYSICS = "physics"


ALL_TARGETS = frozenset(Target)
PHYSICS_ONLY = frozenset({Target.PHYSICS})
NO_TARGETS: frozenset[Target] = frozenset()


@dataclass(frozen=True)
class Issue:
    severity: Severity
    code: str
    entity_id: str
    message: str
    action: str
    blocks: frozenset[Target] = NO_TARGETS


@dataclass
class ValidationReport:
    issues: list[Issue] = field(default_factory=list)
    robot: FlatRobot | None = None

    def add(
        self,
        code: str,
        entity_id: str,
        message: str,
        action: str,
        blocks: frozenset[Target] = ALL_TARGETS,
        severity: Severity | None = None,
    ) -> None:
        level = severity or (Severity.ERROR if blocks else Severity.WARNING)
        self.issues.append(Issue(level, code, entity_id, message, action, blocks))

    def blockers(self, target: Target) -> list[Issue]:
        return [i for i in self.issues if target in i.blocks]

    def ok(self, target: Target) -> bool:
        return self.robot is not None and not self.blockers(target)

    def codes(self) -> set[str]:
        return {i.code for i in self.issues}


def _check_module_graph(module: Module, report: ValidationReport) -> None:
    link_ids = [l.id for l in module.links]
    known = set(link_ids)
    if len(known) != len(link_ids):
        report.add("duplicate_id", module.id, f"Module {module.name} has repeated link IDs.",
                   "Duplicate the module again to regenerate IDs.")
    if module.root_link_id not in known:
        report.add("missing_root", module.id, f"Module {module.name} has no valid root link.",
                   "Choose a root link for this module.")
        return

    parents: dict[str, list] = {}
    references_ok = True
    for joint in module.joints:
        for side, ref in (("parent", joint.parent_id), ("child", joint.child_id)):
            if ref not in known:
                references_ok = False
                report.add("missing_reference", joint.id,
                           f"Joint {joint.name} points to a {side} link that no longer exists.",
                           f"Select {joint.name} and choose a {side} link.")
        parents.setdefault(joint.child_id, []).append(joint)

    for frame in module.frames:
        if frame.link_id not in known:
            references_ok = False
            report.add("missing_reference", frame.id,
                       f"Frame {frame.name} is owned by a link that no longer exists.",
                       f"Select {frame.name} and choose an owner link.")
    if not references_ok:
        return

    names = {l.id: l.name for l in module.links}
    closed = False
    for child, incoming in parents.items():
        if len(incoming) > 1:
            closed = True
            joined = ", ".join(sorted(j.name for j in incoming))
            report.add("closed_loop", child,
                       f"Link {names[child]} has several parent joints ({joined}). Standard URDF only "
                       "describes trees, so this closed mechanical loop cannot be exported as authored.",
                       "Keep one parent joint and model the mechanism as a tree. "
                       "The loop constraint is not removed for you.")
    if module.root_link_id in parents:
        closed = True
        report.add("root_has_parent", module.root_link_id,
                   f"Root link {names[module.root_link_id]} is the child of a joint.",
                   "Choose a different root link or remove that joint.")
    if closed:
        return

    for link_id in link_ids:
        seen: set[str] = set()
        current = link_id
        while current != module.root_link_id:
            if current in seen:
                report.add("joint_cycle", link_id, f"Link {names[link_id]} is part of a joint cycle.",
                           "Select the link and remove one joint from the cycle.")
                break
            seen.add(current)
            if current not in parents:
                report.add("disconnected_link", link_id,
                           f"Link {names[link_id]} is not connected to root {names[module.root_link_id]}.",
                           "Select the link and add a joint to its parent.")
                break
            current = parents[current][0].parent_id


def _check_mounts(project: Project, chosen: set[str], report: ValidationReport) -> None:
    parent_of: dict[str, str] = {}
    for mount in project.mounts:
        if mount.parent_module_id not in chosen or mount.child_module_id not in chosen:
            continue
        parent = project.module(mount.parent_module_id)
        child = project.module(mount.child_module_id)
        if mount.parent_module_id == mount.child_module_id:
            report.add("mount_self", mount.id, f"Module {parent.name} is mounted to itself.",
                       "Choose a different parent module.")
            continue
        if not any(f.id == mount.parent_frame_id for f in parent.frames):
            report.add("missing_reference", mount.id,
                       f"The mount's frame on {parent.name} no longer exists.", "Choose a mounting frame.")
        if not any(f.id == mount.child_frame_id for f in child.frames):
            report.add("missing_reference", mount.id,
                       f"The mount's frame on {child.name} no longer exists.", "Choose a mounting frame.")
        if mount.child_module_id in parent_of:
            report.add("duplicate_mount", mount.id, f"Module {child.name} is mounted more than once.",
                       "Detach one of its mounts.")
        parent_of[mount.child_module_id] = mount.parent_module_id

    for start in parent_of:
        seen = {start}
        current = parent_of[start]
        while current in parent_of:
            if current in seen:
                report.add("mount_cycle", start, "These module mounts form a cycle.",
                           "Detach one of the modules in the cycle.")
                break
            seen.add(current)
            current = parent_of[current]


def _check_transform(pose: Transform, entity: str, label: str, report: ValidationReport) -> None:
    finite = all(math.isfinite(v) for row in pose.R for v in row) and all(math.isfinite(v) for v in pose.t)
    if not finite or classify_linear(pose.R).kind is not AffineKind.RIGID:
        report.add("invalid_transform", entity, f"{label} is not a rigid transform.",
                   "Open its pose and re-enter position and rotation.")


def _check_geometry(item: GeometryElement, report: ValidationReport) -> None:
    _check_transform(item.origin, item.id, f"Geometry {item.name}", report)
    if isinstance(item.shape, Mesh):
        if not item.shape.triangles:
            report.add("empty_mesh", item.id, f"Mesh {item.name} has no triangles.",
                       "Assign geometry or remove this element.")
    elif not all(math.isfinite(v) and v > 0.0 for v in shape_dimensions(item.shape)):
        report.add("invalid_shape", item.id, f"Geometry {item.name} needs positive, finite dimensions.",
                   "Open its shape and correct the dimensions.")


def _check_component_overlap(link: Link, a: Component, b: Component, report: ValidationReport) -> None:
    if a.overlap_confirmed and b.overlap_confirmed:
        return
    if not (a.mass and b.mass and a.mass.shape and b.mass.shape):
        return
    if isinstance(a.mass.shape, Mesh) or isinstance(b.mass.shape, Mesh):
        return
    ha = shape_half_extents(a.mass.shape, a.mass.origin.R)
    hb = shape_half_extents(b.mass.shape, b.mass.origin.R)
    margin = 1e-6  # Parts that only touch are not an overlap, even with single-precision input.
    if all(abs(a.mass.origin.t[i] - b.mass.origin.t[i]) < ha[i] + hb[i] - margin for i in range(3)):
        report.add("mass_overlap", b.id,
                   f"In link {link.name}, the mass shapes of {a.name} and {b.name} may overlap "
                   "(approximate bounding-box check), which can count the same material twice.",
                   "Adjust the shapes or confirm that both components are intended.",
                   blocks=NO_TARGETS)


def _check_masses(module: Module, report: ValidationReport) -> None:
    for link in module.links:
        for component in link.components:
            for item in [*component.visuals, *component.collisions]:
                _check_geometry(item, report)
            if component.mass is None:
                continue
            _check_transform(component.mass.origin, component.id, f"Mass frame of {component.name}", report)
            try:
                body = resolve_mass(component.mass)
            except ValueError as exc:
                report.add("invalid_mass", component.id, f"{component.name}: {exc}",
                           "Open the component's mass settings.", blocks=PHYSICS_ONLY)
                continue
            for problem in inertia_problems(body.mass, body.inertia):
                report.add("invalid_inertia", component.id, f"{component.name}: {problem}",
                           "Open the component's mass settings.", blocks=PHYSICS_ONLY)
        for index, first in enumerate(link.components):
            for second in link.components[index + 1:]:
                _check_component_overlap(link, first, second, report)


def _check_names(robot: FlatRobot, report: ValidationReport) -> None:
    if not NAME_RE.match(robot.name):
        report.add("invalid_name", robot.name, f"Robot name {robot.name!r} is not a valid identifier.",
                   "Use letters, digits, and underscores, starting with a letter or underscore.")
    for kind, items in (("link", robot.links), ("joint", robot.joints)):
        seen: dict[str, str] = {}
        for item in items:
            if not NAME_RE.match(item.name):
                report.add("invalid_name", item.source_id, f"{kind.capitalize()} name {item.name!r} is invalid.",
                           "Use letters, digits, and underscores, starting with a letter or underscore.")
            if item.name in seen:
                report.add("duplicate_name", item.source_id,
                           f"{kind.capitalize()} name {item.name} is used more than once.",
                           "Rename it or give its module a unique prefix.")
            seen[item.name] = item.source_id

    colors: dict[str, tuple] = {}
    for link in robot.links:
        for visual in link.visuals:
            if visual.material is None:
                continue
            known = colors.setdefault(visual.material.name, visual.material.rgba)
            if known != visual.material.rgba:
                report.add("material_conflict", visual.id,
                           f"Material {visual.material.name} has two different colors.",
                           "Rename one of the materials.")


def _check_joint(joint, report: ValidationReport) -> None:
    _check_transform(joint.origin, joint.source_id, f"Joint {joint.name} origin", report)
    moving = joint.type is not JointType.FIXED

    if moving and joint.actuation is Actuation.FIXED:
        report.add("actuation_mismatch", joint.source_id,
                   f"{joint.name} moves but is classified as fixed.",
                   f"Select {joint.name} and set it to actuated or passive.")
    if not moving and joint.actuation is not Actuation.FIXED:
        report.add("actuation_mismatch", joint.source_id,
                   f"{joint.name} is a fixed joint but is classified as {joint.actuation.value}.",
                   f"Select {joint.name} and set its actuation to fixed.")
    if not moving:
        return

    if not all(math.isfinite(v) for v in joint.axis) or norm(joint.axis) < 1e-9:
        report.add("zero_axis", joint.source_id, f"{joint.name} needs a nonzero joint axis.",
                   f"Select {joint.name} and open its axis.")
    if joint.actuation is Actuation.PASSIVE and joint.role in (
        ControlRole.ARM, ControlRole.GRIPPER, ControlRole.BASE_DRIVE, ControlRole.STEERING
    ):
        report.add("role_mismatch", joint.source_id,
                   f"{joint.name} is passive but assigned to {joint.role.value} control.",
                   f"Select {joint.name} and correct its actuation or control role.", blocks=NO_TARGETS)
    if joint.actuation is Actuation.ACTUATED and joint.role is ControlRole.PASSIVE_SUSPENSION:
        report.add("role_mismatch", joint.source_id,
                   f"{joint.name} is actuated but assigned to passive suspension.",
                   f"Select {joint.name} and correct its actuation or control role.", blocks=NO_TARGETS)

    limit = joint.limit
    for label, value in (
        ("damping", joint.damping), ("friction", joint.friction),
    ):
        if not math.isfinite(value) or value < 0.0:
            report.add("invalid_dynamics", joint.source_id, f"{joint.name} {label} must be zero or positive.",
                       f"Select {joint.name} and open its dynamics.")

    if joint.type in (JointType.REVOLUTE, JointType.PRISMATIC):
        missing = []
        if limit is None or limit.lower is None:
            missing.append("lower limit")
        if limit is None or limit.upper is None:
            missing.append("upper limit")
        if limit is None or limit.velocity is None:
            missing.append("velocity limit")
        if limit is None or limit.effort is None:
            missing.append("effort limit")
        if missing:
            report.add("missing_limit", joint.source_id, f"{joint.name} needs: {', '.join(missing)}.",
                       f"Select {joint.name} and open its joint limits.")
    elif joint.actuation is Actuation.ACTUATED and (
        limit is None or limit.velocity is None or limit.effort is None
    ):
        report.add("missing_limit", joint.source_id,
                   f"{joint.name} is actuated and needs velocity and effort limits for simulation.",
                   f"Select {joint.name} and open its joint limits.", blocks=PHYSICS_ONLY)

    if limit is None:
        return
    values = [v for v in (limit.lower, limit.upper, limit.velocity, limit.effort) if v is not None]
    if not all(math.isfinite(v) for v in values):
        report.add("invalid_limit", joint.source_id, f"{joint.name} has a non-finite limit.",
                   f"Select {joint.name} and open its joint limits.")
        return
    if limit.lower is not None and limit.upper is not None and limit.lower > limit.upper:
        report.add("invalid_limit", joint.source_id, f"{joint.name} lower limit is above its upper limit.",
                   f"Select {joint.name} and open its joint limits.")
    if (limit.velocity is not None and limit.velocity < 0.0) or (limit.effort is not None and limit.effort < 0.0):
        report.add("invalid_limit", joint.source_id, f"{joint.name} velocity and effort limits cannot be negative.",
                   f"Select {joint.name} and open its joint limits.")


def _check_robot(robot: FlatRobot, report: ValidationReport) -> None:
    _check_names(robot, report)
    for joint in robot.joints:
        _check_joint(joint, report)

    children = {j.child for j in robot.joints}
    roots = sorted(l.name for l in robot.links if l.name not in children)
    if len(roots) != 1:
        report.add("not_connected", robot.name,
                   f"The export must have one root link, but it has {len(roots)}: {', '.join(roots)}.",
                   "Mount the modules together or export one module.")

    by_name = {l.name: l for l in robot.links}
    for link in robot.links:
        if link.has_body and link.mass is None:
            report.add("missing_inertia", link.source_id, f"Link {link.name} has no usable mass properties.",
                       "Open the link's mass settings.", blocks=PHYSICS_ONLY)
        elif link.mass is not None:
            for problem in inertia_problems(link.mass.mass, link.mass.inertia):
                report.add("invalid_inertia", link.source_id, f"Link {link.name}: {problem}",
                           "Open the link's mass settings.", blocks=PHYSICS_ONLY)
        elif link.visuals or link.collisions:
            report.add("missing_inertia", link.source_id,
                       f"Link {link.name} has geometry but no mass. It can be displayed but not simulated.",
                       "Open the link's mass settings.", blocks=PHYSICS_ONLY)
    for joint in robot.joints:
        child = by_name.get(joint.child)
        if joint.type is not JointType.FIXED and child is not None and child.mass is None:
            report.add("massless_moving_link", joint.source_id,
                       f"{joint.name} moves link {joint.child}, which has no mass.",
                       f"Add mass to {joint.child}.", blocks=PHYSICS_ONLY)


def validate_project(
    project: Project, module_ids: list[str] | None = None, name: str | None = None
) -> ValidationReport:
    """Validate the selected modules. `report.robot` is set when flattening is possible."""
    report = ValidationReport()
    selected = [m for m in project.modules if module_ids is None or m.id in module_ids]
    if module_ids is not None:
        known = {m.id for m in project.modules}
        for module_id in module_ids:
            if module_id not in known:
                report.add("missing_reference", module_id, "A selected module no longer exists.",
                           "Choose the modules to export again.")
    if not selected:
        report.add("empty_selection", project.name, "No modules are selected for export.",
                   "Add or select a module.")
        return report

    for module in selected:
        _check_module_graph(module, report)
    _check_mounts(project, {m.id for m in selected}, report)
    if any(i.severity is Severity.ERROR for i in report.issues):
        return report

    for module in selected:
        _check_masses(module, report)
    report.robot = flatten(project, module_ids, name)
    _check_robot(report.robot, report)
    return report

"""Flatten selected modules and their mounts into one exportable robot."""

from __future__ import annotations

from dataclasses import dataclass, field, replace

from .mass import MassProperties, combine
from .model import (
    Actuation,
    AttachmentFrame,
    ControlRole,
    GeometryElement,
    Joint,
    JointLimit,
    JointType,
    Module,
    Project,
    resolve_mass,
)
from .transforms import Transform, Vec3


class ModelError(ValueError):
    """The model's references or topology cannot be resolved."""


@dataclass
class FlatLink:
    name: str
    source_id: str
    mass: MassProperties | None = None
    visuals: list[GeometryElement] = field(default_factory=list)
    collisions: list[GeometryElement] = field(default_factory=list)
    has_body: bool = False


@dataclass
class FlatJoint:
    name: str
    source_id: str
    type: JointType
    parent: str
    child: str
    origin: Transform
    axis: Vec3 = (0.0, 0.0, 1.0)
    limit: JointLimit | None = None
    damping: float = 0.0
    friction: float = 0.0
    actuation: Actuation = Actuation.FIXED
    role: ControlRole = ControlRole.UNASSIGNED


@dataclass
class FlatRobot:
    name: str
    links: list[FlatLink] = field(default_factory=list)
    joints: list[FlatJoint] = field(default_factory=list)

    def link(self, name: str) -> FlatLink:
        return next(item for item in self.links if item.name == name)

    def joint(self, name: str) -> FlatJoint:
        return next(item for item in self.joints if item.name == name)

    @property
    def root(self) -> str:
        children = {j.child for j in self.joints}
        roots = sorted(l.name for l in self.links if l.name not in children)
        if len(roots) != 1:
            raise ModelError(f"Expected one root link, found {len(roots)}: {roots}.")
        return roots[0]


def link_pose_in_module(module: Module, link_id: str) -> Transform:
    """Return T_root_link at the zero configuration."""
    by_child: dict[str, Joint] = {j.child_id: j for j in module.joints}
    pose = Transform()
    current = link_id
    visited: set[str] = set()
    while current != module.root_link_id:
        if current in visited:
            raise ModelError(f"Module {module.name!r} contains a joint cycle.")
        visited.add(current)
        joint = by_child.get(current)
        if joint is None:
            raise ModelError(f"Link {current!r} is not connected to the root of {module.name!r}.")
        pose = joint.origin @ pose
        current = joint.parent_id
    return pose


def _find_frame(module: Module, frame_id: str) -> AttachmentFrame:
    for frame in module.frames:
        if frame.id == frame_id:
            return frame
    raise ModelError(f"Frame {frame_id!r} does not belong to module {module.name!r}.")


def frame_pose_in_root(module: Module, frame_id: str) -> Transform:
    """Return T_root_frame at the zero configuration."""
    frame = _find_frame(module, frame_id)
    return link_pose_in_module(module, frame.link_id) @ frame.pose


def _flatten_module(module: Module, robot: FlatRobot) -> None:
    names = {link.id: f"{module.prefix}{link.name}" for link in module.links}
    for link in module.links:
        contributors = []
        flat = FlatLink(names[link.id], link.id)
        for component in link.components:
            flat.visuals.extend(
                replace(v, material=replace(v.material, name=f"{module.prefix}{v.material.name}"))
                if v.material is not None
                else v
                for v in component.visuals
            )
            flat.collisions.extend(component.collisions)
            if component.mass is not None:
                flat.has_body = True
                try:
                    contributors.append(resolve_mass(component.mass))
                except ValueError:
                    continue  # Reported by validation, which blocks export.
        if contributors:
            try:
                flat.mass = combine(contributors)
            except ValueError:
                flat.mass = None  # Reported by validation, which blocks export.
        robot.links.append(flat)

    for joint in module.joints:
        robot.joints.append(
            FlatJoint(
                name=f"{module.prefix}{joint.name}",
                source_id=joint.id,
                type=joint.type,
                parent=names[joint.parent_id],
                child=names[joint.child_id],
                origin=joint.origin,
                axis=joint.axis,
                limit=joint.limit,
                damping=joint.damping,
                friction=joint.friction,
                actuation=joint.actuation,
                role=joint.role,
            )
        )

    for frame in module.frames:
        frame_link = f"{module.prefix}{frame.name}"
        robot.links.append(FlatLink(frame_link, frame.id))
        robot.joints.append(
            FlatJoint(
                name=f"{frame_link}_joint",
                source_id=frame.id,
                type=JointType.FIXED,
                parent=names[frame.link_id],
                child=frame_link,
                origin=frame.pose,
            )
        )


def flatten(project: Project, module_ids: list[str] | None = None, name: str | None = None) -> FlatRobot:
    """Build one robot from the selected modules. Mounts outside the selection are ignored."""
    selected = [m for m in project.modules if module_ids is None or m.id in module_ids]
    robot = FlatRobot(name or project.name)
    for module in selected:
        _flatten_module(module, robot)

    chosen = {m.id for m in selected}
    for mount in project.mounts:
        if mount.parent_module_id not in chosen or mount.child_module_id not in chosen:
            continue
        parent = project.module(mount.parent_module_id)
        child = project.module(mount.child_module_id)
        parent_frame = _find_frame(parent, mount.parent_frame_id)
        parent_link = next(l for l in parent.links if l.id == parent_frame.link_id)
        child_root = next(l for l in child.links if l.id == child.root_link_id)
        T_root_child_frame = frame_pose_in_root(child, mount.child_frame_id)
        origin = parent_frame.pose @ mount.offset @ T_root_child_frame.inverse()
        robot.joints.append(
            FlatJoint(
                name=f"{child.prefix}mount_joint",
                source_id=mount.id,
                type=JointType.FIXED,
                parent=f"{parent.prefix}{parent_link.name}",
                child=f"{child.prefix}{child_root.name}",
                origin=origin,
            )
        )
    return robot

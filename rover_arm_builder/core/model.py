"""Robot intermediate representation. All stored values are SI.

Identity lives in stable `id` fields. Names are only used to produce exported
identifiers, so a rename never breaks a reference.
"""

from __future__ import annotations

import math
import uuid
from dataclasses import dataclass, field, replace
from enum import Enum
from typing import Callable, Union

from .mass import (
    Inertia,
    MassContributor,
    box_inertia,
    box_volume,
    cylinder_inertia,
    cylinder_volume,
    sphere_inertia,
    sphere_volume,
)
from .transforms import Mat3, Transform, Vec3

SCHEMA_VERSION = 1


class JointType(Enum):
    FIXED = "fixed"
    REVOLUTE = "revolute"
    CONTINUOUS = "continuous"
    PRISMATIC = "prismatic"


class Actuation(Enum):
    ACTUATED = "actuated"
    PASSIVE = "passive"
    FIXED = "fixed"


class ControlRole(Enum):
    ARM = "arm"
    BASE_DRIVE = "base_drive"
    STEERING = "steering"
    PASSIVE_SUSPENSION = "passive_suspension"
    OTHER = "other"
    UNASSIGNED = "unassigned"


class MassMode(Enum):
    MEASURED = "measured"
    ESTIMATED = "estimated"
    DENSITY = "density"
    MANUAL = "manual"


class ModuleKind(Enum):
    ROVER = "rover"
    ARM = "arm"
    ACCESSORY = "accessory"


class FrameKind(Enum):
    MOUNT = "mount"
    TOOL = "tool"
    REFERENCE = "reference"


@dataclass(frozen=True)
class Box:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class Cylinder:
    """Circular cylinder with its axis along local Z."""

    radius: float
    length: float


@dataclass(frozen=True)
class Sphere:
    radius: float


@dataclass(frozen=True)
class Mesh:
    """Triangles in meters, expressed in the owning element's frame."""

    asset: str
    triangles: tuple[tuple[Vec3, Vec3, Vec3], ...]


Shape = Union[Box, Cylinder, Sphere, Mesh]


def shape_dimensions(shape: Shape) -> tuple[float, ...]:
    if isinstance(shape, Box):
        return (shape.x, shape.y, shape.z)
    if isinstance(shape, Cylinder):
        return (shape.radius, shape.length)
    if isinstance(shape, Sphere):
        return (shape.radius,)
    return ()


def shape_volume(shape: Shape) -> float:
    if isinstance(shape, Box):
        return box_volume(shape.x, shape.y, shape.z)
    if isinstance(shape, Cylinder):
        return cylinder_volume(shape.radius, shape.length)
    if isinstance(shape, Sphere):
        return sphere_volume(shape.radius)
    raise ValueError("Mesh volume is not estimated. Use a primitive distribution or manual inertia.")


def shape_inertia(shape: Shape, mass: float) -> Inertia:
    if isinstance(shape, Box):
        return box_inertia(mass, shape.x, shape.y, shape.z)
    if isinstance(shape, Cylinder):
        return cylinder_inertia(mass, shape.radius, shape.length)
    if isinstance(shape, Sphere):
        return sphere_inertia(mass, shape.radius)
    raise ValueError("Mesh inertia is not estimated. Use a primitive distribution or manual inertia.")


def shape_half_extents(shape: Shape, R: Mat3) -> Vec3:
    """Half-widths of the axis-aligned box around a primitive rotated by R."""
    if isinstance(shape, Box):
        h = (shape.x / 2, shape.y / 2, shape.z / 2)
        return tuple(sum(abs(R[i][j]) * h[j] for j in range(3)) for i in range(3))  # type: ignore[return-value]
    if isinstance(shape, Cylinder):
        out = []
        for i in range(3):
            a = min(1.0, abs(R[i][2]))
            if a > 1.0 - 1e-6:  # Single-precision rotations would otherwise inflate the radius term.
                a = 1.0
            out.append(shape.length / 2 * a + shape.radius * math.sqrt(1.0 - a * a))
        return (out[0], out[1], out[2])
    if isinstance(shape, Sphere):
        return (shape.radius, shape.radius, shape.radius)
    raise ValueError("Mesh extents are not supported for this check.")


@dataclass(frozen=True)
class Material:
    name: str
    rgba: tuple[float, float, float, float]


@dataclass
class GeometryElement:
    """A visual or collision shape. Geometry never contributes mass."""

    id: str
    name: str
    shape: Shape
    origin: Transform = field(default_factory=Transform)
    material: Material | None = None


@dataclass
class MassSpec:
    """Mass source for one component. `origin` is the body frame in the link frame."""

    mode: MassMode
    origin: Transform = field(default_factory=Transform)
    shape: Shape | None = None
    mass: float | None = None
    density: float | None = None
    manual_inertia: Inertia | None = None
    assumption: str = "Uniform-density solid"


def resolve_mass(spec: MassSpec) -> MassContributor:
    """Compute a contributor from a spec, or raise ValueError with the reason."""
    if spec.mode is MassMode.MANUAL:
        if spec.mass is None or spec.manual_inertia is None:
            raise ValueError("Manual mass mode needs both a mass and an inertia tensor.")
        return MassContributor(spec.mass, spec.origin.t, spec.manual_inertia, spec.origin.R)
    if spec.shape is None:
        raise ValueError("This mass mode needs a distribution shape.")
    if spec.mode is MassMode.DENSITY:
        if spec.density is None:
            raise ValueError("Density mode needs a density in kg/m^3.")
        if not math.isfinite(spec.density) or spec.density <= 0.0:
            raise ValueError(f"Density must be positive and finite, got {spec.density!r}.")
        mass = spec.density * shape_volume(spec.shape)
    else:
        if spec.mass is None:
            raise ValueError("This mass mode needs a mass in kg.")
        mass = spec.mass
    return MassContributor(mass, spec.origin.t, shape_inertia(spec.shape, mass), spec.origin.R)


@dataclass
class Component:
    """A physical part. It has at most one mass owner: its `mass` spec."""

    id: str
    name: str
    mass: MassSpec | None = None
    visuals: list[GeometryElement] = field(default_factory=list)
    collisions: list[GeometryElement] = field(default_factory=list)
    overlap_confirmed: bool = False


@dataclass
class Link:
    id: str
    name: str
    components: list[Component] = field(default_factory=list)

    @property
    def is_frame_only(self) -> bool:
        return not any(c.mass or c.visuals or c.collisions for c in self.components)


@dataclass
class JointLimit:
    lower: float | None = None
    upper: float | None = None
    velocity: float | None = None
    effort: float | None = None


@dataclass
class Joint:
    """`origin` is T_P_J. The child link frame equals the joint frame at zero."""

    id: str
    name: str
    type: JointType
    parent_id: str
    child_id: str
    origin: Transform = field(default_factory=Transform)
    axis: Vec3 = (0.0, 0.0, 1.0)
    limit: JointLimit | None = None
    damping: float = 0.0
    friction: float = 0.0
    actuation: Actuation = Actuation.FIXED
    role: ControlRole = ControlRole.UNASSIGNED


@dataclass
class AttachmentFrame:
    id: str
    name: str
    link_id: str
    pose: Transform = field(default_factory=Transform)
    kind: FrameKind = FrameKind.MOUNT


@dataclass
class Module:
    id: str
    name: str
    kind: ModuleKind
    prefix: str
    root_link_id: str
    links: list[Link] = field(default_factory=list)
    joints: list[Joint] = field(default_factory=list)
    frames: list[AttachmentFrame] = field(default_factory=list)
    poses: dict[str, dict[str, float]] = field(default_factory=dict)


@dataclass
class Mount:
    """Fixed attachment of a child module's frame to a parent module's frame."""

    id: str
    parent_module_id: str
    parent_frame_id: str
    child_module_id: str
    child_frame_id: str
    offset: Transform = field(default_factory=Transform)


@dataclass
class Project:
    name: str
    modules: list[Module] = field(default_factory=list)
    mounts: list[Mount] = field(default_factory=list)
    schema_version: int = SCHEMA_VERSION

    def module(self, module_id: str) -> Module | None:
        return next((m for m in self.modules if m.id == module_id), None)


def new_id() -> str:
    return uuid.uuid4().hex


def instantiate_module(
    source: Module, prefix: str, name: str | None = None, make_id: Callable[[], str] = new_id
) -> Module:
    """Copy a module with fresh IDs and a new prefix. The source is not modified."""
    ids: dict[str, str] = {}

    def fresh(old: str) -> str:
        if old not in ids:
            ids[old] = make_id()
        return ids[old]

    def copy_geometry(item: GeometryElement) -> GeometryElement:
        return replace(item, id=fresh(item.id))

    links = [
        Link(
            id=fresh(link.id),
            name=link.name,
            components=[
                replace(
                    c,
                    id=fresh(c.id),
                    mass=replace(c.mass) if c.mass else None,
                    visuals=[copy_geometry(v) for v in c.visuals],
                    collisions=[copy_geometry(v) for v in c.collisions],
                )
                for c in link.components
            ],
        )
        for link in source.links
    ]
    joints = [
        replace(
            j,
            id=fresh(j.id),
            parent_id=fresh(j.parent_id),
            child_id=fresh(j.child_id),
            limit=replace(j.limit) if j.limit else None,
        )
        for j in source.joints
    ]
    frames = [replace(f, id=fresh(f.id), link_id=fresh(f.link_id)) for f in source.frames]
    poses = {
        pose: {fresh(joint_id): value for joint_id, value in values.items()}
        for pose, values in source.poses.items()
    }
    return Module(
        id=make_id(),
        name=name or source.name,
        kind=source.kind,
        prefix=prefix,
        root_link_id=fresh(source.root_link_id),
        links=links,
        joints=joints,
        frames=frames,
        poses=poses,
    )

"""Demonstration robot. Every dimension, mass, and limit is illustrative, not measured.

The suspension is a passive pivoting front axle. With the rigid rear axle it gives three
support points, so the chassis cannot pitch freely. Two independent side rockers without a
differential would let it pitch until the joint limits, and a differential is a closed
mechanical loop that standard URDF cannot describe.
"""

from __future__ import annotations

import math

from .model import (
    Actuation,
    AttachmentFrame,
    Box,
    Component,
    ControlRole,
    Cylinder,
    FrameKind,
    GeometryElement,
    Joint,
    JointLimit,
    JointType,
    Link,
    MassMode,
    MassSpec,
    Material,
    Module,
    ModuleKind,
    Mount,
    Project,
    Shape,
    Sphere,
)
from .transforms import Transform

BODY = Material("body_gray", (0.45, 0.47, 0.5, 1.0))
DARK = Material("wheel_black", (0.08, 0.08, 0.09, 1.0))
ACCENT = Material("arm_orange", (0.9, 0.45, 0.1, 1.0))
AXLE_Y = (math.pi / 2, 0.0, 0.0)

AXLE_X = 0.22
AXLE_Z = -0.11
MOTOR_Y = 0.21
WHEEL_Y = 0.27


def _part(key: str, name: str, shape: Shape, mass: float, xyz, rpy=(0.0, 0.0, 0.0), material=BODY) -> Component:
    pose = Transform.from_xyz_rpy(xyz, rpy)
    return Component(
        id=f"cmp_{key}",
        name=name,
        mass=MassSpec(MassMode.ESTIMATED, origin=pose, shape=shape, mass=mass,
                      assumption="Illustrative uniform-density solid"),
        visuals=[GeometryElement(f"vis_{key}", f"{name}_visual", shape, pose, material)],
        collisions=[GeometryElement(f"col_{key}", f"{name}_collision", shape, pose)],
    )


def _motor(side: str, sign: float, end: str, x: float, z: float) -> Component:
    """A motor housing, owned by the member that carries it rather than by the wheel."""
    return _part(f"{side}_{end}_motor", f"{side}_{end}_motor_housing", Cylinder(0.03, 0.06), 0.25,
                 (x, sign * MOTOR_Y, z), AXLE_Y, DARK)


def _wheel(side: str, sign: float, end: str, parent_id: str, x: float, z: float) -> tuple[Link, Joint]:
    wheel_id = f"lnk_{side}_{end}_wheel"
    link = Link(wheel_id, f"{side}_{end}_wheel", [
        _part(f"{side}_{end}_wheel", f"{side}_{end}_wheel_body", Cylinder(0.08, 0.05), 0.5,
              (0.0, 0.0, 0.0), AXLE_Y, DARK)
    ])
    joint = Joint(
        id=f"jnt_{side}_{end}_wheel",
        name=f"{side}_{end}_wheel_joint",
        type=JointType.CONTINUOUS,
        parent_id=parent_id,
        child_id=wheel_id,
        origin=Transform.from_xyz_rpy((x, sign * WHEEL_Y, z)),
        axis=(0.0, 1.0, 0.0),
        limit=JointLimit(velocity=10.0, effort=5.0),
        damping=0.05,
        actuation=Actuation.ACTUATED,
        role=ControlRole.BASE_DRIVE,
    )
    return link, joint


def build_rover() -> Module:
    chassis_parts = [
        _part("chassis", "chassis_body", Box(0.6, 0.4, 0.15), 12.0, (0.0, 0.0, 0.0)),
        _part("antenna", "antenna_mast", Cylinder(0.008, 0.25), 0.1, (-0.22, 0.14, 0.2), material=DARK),
    ]
    axle_parts = [_part("front_axle", "front_axle_beam", Box(0.04, 0.36, 0.02), 0.6, (0.0, 0.0, 0.0))]
    links: list[Link] = []
    joints: list[Joint] = []

    for side, sign in (("left", 1.0), ("right", -1.0)):
        chassis_parts.append(_motor(side, sign, "rear", -AXLE_X, AXLE_Z))
        axle_parts.append(_motor(side, sign, "front", 0.0, 0.0))
        rear_link, rear_joint = _wheel(side, sign, "rear", "lnk_chassis", -AXLE_X, AXLE_Z)
        front_link, front_joint = _wheel(side, sign, "front", "lnk_front_axle", 0.0, 0.0)
        links.extend((rear_link, front_link))
        joints.extend((rear_joint, front_joint))

    links.insert(0, Link("lnk_chassis", "chassis", chassis_parts))
    links.insert(1, Link("lnk_front_axle", "front_axle", axle_parts))
    joints.insert(0, Joint(
        id="jnt_front_axle",
        name="front_axle_joint",
        type=JointType.REVOLUTE,
        parent_id="lnk_chassis",
        child_id="lnk_front_axle",
        origin=Transform.from_xyz_rpy((AXLE_X, 0.0, AXLE_Z)),
        axis=(1.0, 0.0, 0.0),
        limit=JointLimit(lower=-0.12, upper=0.12, velocity=2.0, effort=50.0),
        damping=2.0,
        friction=0.1,
        actuation=Actuation.PASSIVE,
        role=ControlRole.PASSIVE_SUSPENSION,
    ))

    return Module(
        id="mod_rover",
        name="Demo Rover",
        kind=ModuleKind.ROVER,
        prefix="rover_",
        root_link_id="lnk_chassis",
        links=links,
        joints=joints,
        frames=[AttachmentFrame("frm_arm_mount", "arm_mount", "lnk_chassis",
                                Transform.from_xyz_rpy((0.1, 0.0, 0.075)), FrameKind.MOUNT)],
    )


def build_arm() -> Module:
    links = [
        Link("lnk_arm_base", "base", [
            _part("arm_base", "base_body", Cylinder(0.05, 0.06), 0.8, (0.0, 0.0, 0.03), material=ACCENT)
        ]),
        Link("lnk_upper", "upper_link", [
            _part("upper", "upper_body", Box(0.05, 0.05, 0.25), 0.7, (0.0, 0.0, 0.125), material=ACCENT)
        ]),
        Link("lnk_fore", "forearm_link", [
            _part("fore", "forearm_body", Box(0.04, 0.04, 0.2), 0.4, (0.0, 0.0, 0.1), material=ACCENT),
            _part("wrist", "wrist_ball", Sphere(0.025), 0.05, (0.0, 0.0, 0.225), material=DARK),
        ]),
    ]
    joints = [
        Joint("jnt_shoulder", "shoulder_yaw_joint", JointType.REVOLUTE, "lnk_arm_base", "lnk_upper",
              Transform.from_xyz_rpy((0.0, 0.0, 0.06)), (0.0, 0.0, 1.0),
              JointLimit(-2.6, 2.6, 1.0, 15.0), damping=0.2, actuation=Actuation.ACTUATED, role=ControlRole.ARM),
        Joint("jnt_elbow", "elbow_pitch_joint", JointType.REVOLUTE, "lnk_upper", "lnk_fore",
              Transform.from_xyz_rpy((0.0, 0.0, 0.25)), (0.0, 1.0, 0.0),
              JointLimit(-2.0, 2.0, 1.0, 10.0), damping=0.1, actuation=Actuation.ACTUATED, role=ControlRole.ARM),
    ]
    frames = [
        AttachmentFrame("frm_arm_base_mount", "base_mount", "lnk_arm_base", Transform(), FrameKind.MOUNT),
        AttachmentFrame("frm_tool", "tool_frame", "lnk_fore", Transform.from_xyz_rpy((0.0, 0.0, 0.25)), FrameKind.TOOL),
    ]
    return Module(
        id="mod_arm",
        name="Demo Arm",
        kind=ModuleKind.ARM,
        prefix="arm_",
        root_link_id="lnk_arm_base",
        links=links,
        joints=joints,
        frames=frames,
        poses={
            "home": {"jnt_shoulder": 0.0, "jnt_elbow": 0.0},
            "stow": {"jnt_shoulder": 0.0, "jnt_elbow": 1.5},
        },
    )


def build_demo_project() -> Project:
    return Project(
        name="demo_rover_arm",
        modules=[build_rover(), build_arm()],
        mounts=[Mount("mnt_arm", "mod_rover", "frm_arm_mount", "mod_arm", "frm_arm_base_mount")],
    )

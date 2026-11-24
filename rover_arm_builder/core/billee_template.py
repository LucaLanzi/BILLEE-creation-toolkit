"""A six-wheel starting point that matches the BILLEE drivetrain layout.

Track width, wheel radius, wheel stations, and camera pose follow billee-software-2027.
Chassis shapes and masses are simple placeholders, so replace them with measured values.
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
    Project,
    Shape,
)
from .transforms import Transform

BODY = Material("body_white", (0.82, 0.83, 0.85, 1.0))
FRAME = Material("frame_gray", (0.25, 0.26, 0.28, 1.0))
WHEEL = Material("wheel_black", (0.04, 0.04, 0.05, 1.0))
AXLE_Y = (math.pi / 2, 0.0, 0.0)

WHEEL_RADIUS = 0.11
WHEEL_WIDTH = 0.08
TRACK = 0.67
STATIONS = (("front", 0.39), ("mid", 0.0), ("rear", -0.5))
CAMERA_POSE = (0.3624, 0.0, 0.3424)


def _part(key: str, shape: Shape, mass: float, xyz, rpy=(0.0, 0.0, 0.0), material: Material = BODY) -> Component:
    pose = Transform.from_xyz_rpy(xyz, rpy)
    return Component(
        id=f"cmp_{key}",
        name=key,
        mass=MassSpec(MassMode.ESTIMATED, origin=pose, shape=shape, mass=mass,
                      assumption="Placeholder uniform-density solid"),
        visuals=[GeometryElement(f"vis_{key}", f"{key}_visual", shape, pose, material)],
        collisions=[GeometryElement(f"col_{key}", f"{key}_collision", shape, pose)],
    )


def build_billee_rover() -> Module:
    base = Link("lnk_base", "base", [
        _part("chassis", Box(0.9, 0.5, 0.2), 35.0, (-0.05, 0.0, 0.2)),
        _part("left_rail", Box(1.0, 0.05, 0.04), 2.0, (-0.055, 0.25, 0.08), material=FRAME),
        _part("right_rail", Box(1.0, 0.05, 0.04), 2.0, (-0.055, -0.25, 0.08), material=FRAME),
        _part("camera_mast", Cylinder(0.012, 0.0424), 0.1, (CAMERA_POSE[0], 0.0, 0.3212), material=FRAME),
    ])
    links = [base]
    joints = []
    for side, sign in (("left", 1.0), ("right", -1.0)):
        for station, x in STATIONS:
            name = f"{side}_{station}_wheel"
            links.append(Link(f"lnk_{name}", name, [
                _part(f"{name}_body", Cylinder(WHEEL_RADIUS, WHEEL_WIDTH), 1.5, (0.0, 0.0, 0.0), AXLE_Y, WHEEL)
            ]))
            joints.append(Joint(
                id=f"jnt_{name}",
                name=f"{name}_joint",
                type=JointType.CONTINUOUS,
                parent_id="lnk_base",
                child_id=f"lnk_{name}",
                origin=Transform.from_xyz_rpy((x, sign * TRACK / 2.0, 0.0)),
                axis=(0.0, 1.0, 0.0),
                limit=JointLimit(velocity=30.0, effort=60.0),
                damping=0.05,
                actuation=Actuation.ACTUATED,
                role=ControlRole.BASE_DRIVE,
            ))
    frames = [
        AttachmentFrame("frm_camera", "camera_link", "lnk_base", Transform.from_xyz_rpy(CAMERA_POSE), FrameKind.REFERENCE),
        AttachmentFrame("frm_arm_mount", "arm_mount", "lnk_base", Transform.from_xyz_rpy((0.05, 0.0, 0.3)), FrameKind.MOUNT),
    ]
    return Module("mod_billee", "BILLEE Rover", ModuleKind.ROVER, "rover_", "lnk_base", links, joints, frames)


def build_billee_project() -> Project:
    return Project(name="billee", modules=[build_billee_rover()])

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
    DriveKind,
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
    MotorDrive,
    Mount,
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


def build_billee_arm() -> Module:
    """Five axes plus an end effector. Shapes, masses, and limits are placeholders.

    Axes 1 to 3 are P17 or P20 CAN motors. Axes 4, 5, and the end effector are goBILDA 5203
    19.2:1 motors on RoboClaws. CAN IDs and RoboClaw addresses stay unset until they are known.
    """
    color = Material("arm_orange", (0.9, 0.45, 0.1, 1.0))
    links = [
        Link("lnk_arm_base", "arm_base", [_part("arm_base_body", Cylinder(0.06, 0.08), 1.0, (0, 0, 0.04), material=color)]),
        Link("lnk_arm_1", "arm_link_1", [_part("arm_link_1_body", Cylinder(0.05, 0.08), 0.8, (0, 0, 0.04), material=color)]),
        Link("lnk_arm_2", "arm_link_2", [_part("arm_link_2_body", Box(0.05, 0.05, 0.3), 1.0, (0, 0, 0.15), material=color)]),
        Link("lnk_arm_3", "arm_link_3", [_part("arm_link_3_body", Box(0.045, 0.045, 0.25), 0.7, (0, 0, 0.125), material=color)]),
        Link("lnk_arm_4", "arm_link_4", [_part("arm_link_4_body", Cylinder(0.025, 0.06), 0.25, (0, 0, 0.03), material=FRAME)]),
        Link("lnk_arm_5", "arm_link_5", [_part("arm_link_5_body", Box(0.04, 0.04, 0.06), 0.2, (0, 0, 0.03), material=color)]),
        Link("lnk_gripper", "arm_gripper", [_part("arm_gripper_body", Box(0.02, 0.06, 0.05), 0.08, (0, 0, 0.025), material=FRAME)]),
    ]
    can = MotorDrive(DriveKind.CAN, "P20 or P17")
    usb = MotorDrive(DriveKind.ROBOCLAW, "goBILDA 5203 19.2:1", qppr=538)
    table = [
        ("joint_arm_1", "lnk_arm_base", "lnk_arm_1", 0.08, (0.0, 0.0, 1.0), -3.0, 3.0, 40.0, can, ControlRole.ARM),
        ("joint_arm_2", "lnk_arm_1", "lnk_arm_2", 0.08, (0.0, 1.0, 0.0), -1.57, 1.57, 40.0, can, ControlRole.ARM),
        ("joint_arm_3", "lnk_arm_2", "lnk_arm_3", 0.30, (0.0, 1.0, 0.0), -2.3, 2.3, 20.0, can, ControlRole.ARM),
        ("joint_arm_4", "lnk_arm_3", "lnk_arm_4", 0.25, (0.0, 0.0, 1.0), -3.0, 3.0, 2.0, usb, ControlRole.ARM),
        ("joint_arm_5", "lnk_arm_4", "lnk_arm_5", 0.06, (0.0, 1.0, 0.0), -1.8, 1.8, 2.0, usb, ControlRole.ARM),
        ("joint_gripper", "lnk_arm_5", "lnk_gripper", 0.06, (1.0, 0.0, 0.0), -0.6, 0.6, 2.0, usb, ControlRole.GRIPPER),
    ]
    joints = [
        Joint(name, name, JointType.REVOLUTE, parent, child, Transform.from_xyz_rpy((0.0, 0.0, z)), axis,
              JointLimit(lower, upper, 1.0, effort), damping=0.5, friction=0.05,
              actuation=Actuation.ACTUATED, role=role, drive=drive)
        for name, parent, child, z, axis, lower, upper, effort, drive, role in table
    ]
    frames = [
        AttachmentFrame("frm_arm_plate", "arm_mount_plate", "lnk_arm_base", Transform(), FrameKind.MOUNT),
        AttachmentFrame("frm_arm_tool", "arm_tool", "lnk_arm_5", Transform.from_xyz_rpy((0.0, 0.0, 0.11)), FrameKind.TOOL),
    ]
    poses = {
        "home": {f"joint_arm_{n}": 0.0 for n in range(1, 6)},
        "stow": {"joint_arm_1": 0.0, "joint_arm_2": -0.5, "joint_arm_3": 2.0, "joint_arm_4": 0.0, "joint_arm_5": 0.8},
    }
    return Module("mod_billee_arm", "BILLEE Arm", ModuleKind.ARM, "", "lnk_arm_base", links, joints, frames, poses)


def build_billee_project() -> Project:
    return Project(name="billee", modules=[build_billee_rover()])


def build_billee_assembly() -> Project:
    """The rover and arm as independent modules, joined through their mounting frames."""
    return Project(
        name="billee",
        modules=[build_billee_rover(), build_billee_arm()],
        mounts=[Mount("mnt_billee_arm", "mod_billee", "frm_arm_mount", "mod_billee_arm", "frm_arm_plate")],
    )

"""Generate a drop-in replacement for billee-software-2027's robot_description package.

Only the robot's geometry is generated. The software stack's wiring (CAN node IDs, the
ros2_control hardware switch, Gazebo sensors and plugins, controller settings, packaging)
is copied byte-for-byte from that package, except for values that depend on the new model:

- urdf/robot_core.urdf.xacro  is generated.
- config/controllers.yaml     gets the new wheel_separation and wheel_radius.
- urdf/camera.urdf.xacro      gets a new pose only if the model has a camera_link frame.
- urdf/robot_gz.urdf.xacro    drops color blocks for links the new model does not have.

The source package is only ever read.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
import shutil
import tempfile
from dataclasses import dataclass, field, replace
from pathlib import Path

from ..core.assembly import FlatJoint, FlatLink, FlatRobot
from ..core.model import (
    Actuation,
    ControlRole,
    Cylinder,
    GeometryElement,
    JointType,
    Mesh,
    ModuleKind,
    Project,
    Sphere,
)
from ..core.transforms import Transform, dot, mat_vec, normalized, scale, sub
from ..core.validate import Issue, Severity, Target, ValidationReport, validate_project
from .billee_arm import PACKAGE as ARM_PACKAGE
from .billee_arm import ArmPlan, analyze_arm, build_moveit_files, real_hardware_gaps, simulation_block
from .package import PackageConflict
from .stl import mesh_to_stl_bytes
from .urdf import ExportBlocked, fmt, mesh_filename, robot_to_urdf

PACKAGE = "robot_description"
BUNDLED = Path(__file__).resolve().parent / "billee_wiring" / PACKAGE
AXIS_TOLERANCE = math.cos(math.radians(10.0))
RADIUS_TOLERANCE = 0.002
REQUIRED_WIRING = (
    "package.xml", "setup.py", "config/controllers.yaml", "urdf/robot.urdf.xacro", "urdf/robot_core.urdf.xacro",
    "urdf/robot_gz.urdf.xacro", "urdf/ros2_control.urdf.xacro", "urdf/camera.urdf.xacro",
)
# Wheel numbering in billee-software-2027: left runs rear to front, right runs front to rear.
WHEEL_SLOTS = {
    ("left", "front"): ("joint_wheel_l3", "wheel_3"),
    ("left", "mid"): ("joint_wheel_l2", "wheel_2"),
    ("left", "rear"): ("joint_wheel_l1", "wheel_1"),
    ("right", "front"): ("joint_wheel_r1", "wheel_r_3"),
    ("right", "mid"): ("joint_wheel_r2", "wheel_r_2"),
    ("right", "rear"): ("joint_wheel_r3", "wheel_r_1"),
}
RESERVED_LINKS = {"base_footprint", "base_link", "camera_link", "camera_link_optical"}
RESERVED_JOINTS = {"dummy_joint", "camera_joint", "camera_optical_joint"}


class WiringError(ValueError):
    """The robot_description package to copy wiring from is not usable."""


@dataclass
class WheelAssignment:
    source_joint: str
    source_link: str
    joint: str
    link: str
    position: str
    flip_axis: bool


@dataclass
class DropinPlan:
    report: ValidationReport
    issues: list[Issue] = field(default_factory=list)
    wheels: dict[str, WheelAssignment] = field(default_factory=dict)
    radius: float = 0.0
    separation: float = 0.0
    origin_shift: tuple[float, float, float] = (0.0, 0.0, 0.0)
    frozen: list[str] = field(default_factory=list)
    camera_link: str | None = None
    camera_pose: Transform | None = None
    arm: ArmPlan | None = None
    real_arm_gaps: list[str] = field(default_factory=list)

    @property
    def errors(self) -> list[Issue]:
        return [i for i in self.issues if i.severity is Severity.ERROR]

    @property
    def ready(self) -> bool:
        return not self.errors and self.report.ok(Target.PHYSICS)

    def error(self, code: str, entity: str, message: str, action: str) -> None:
        self.issues.append(Issue(Severity.ERROR, code, entity, message, action))

    def warn(self, code: str, entity: str, message: str, action: str) -> None:
        self.issues.append(Issue(Severity.WARNING, code, entity, message, action))


def _poses(robot: FlatRobot) -> dict[str, Transform]:
    by_child = {j.child: j for j in robot.joints}

    def pose(name: str) -> Transform:
        joint = by_child.get(name)
        return Transform() if joint is None else pose(joint.parent) @ joint.origin

    return {link.name: pose(link.name) for link in robot.links}


def _wheel_center(link: FlatLink, pose: Transform):
    if link.mass is not None:
        return pose.apply(link.mass.com)
    items = link.collisions or link.visuals
    return pose.apply(items[0].origin.t) if items else pose.t


def _wheel_radius(link: FlatLink, axis) -> float | None:
    """Largest rolling radius about the joint axis, from the wheel's own shapes."""
    radii = []
    for item in [*link.collisions, *link.visuals]:
        if isinstance(item.shape, Cylinder):
            cylinder_axis = mat_vec(item.origin.R, (0.0, 0.0, 1.0))
            if abs(dot(cylinder_axis, axis)) >= AXIS_TOLERANCE:
                radii.append(item.shape.radius)
        elif isinstance(item.shape, Sphere):
            radii.append(item.shape.radius)
        elif isinstance(item.shape, Mesh):
            distance = 0.0
            for triangle in item.shape.triangles:
                for vertex in triangle:
                    point = item.origin.apply(vertex)
                    offset = sub(point, scale(axis, dot(point, axis)))
                    distance = max(distance, math.sqrt(dot(offset, offset)))
            if distance > 0.0:
                radii.append(distance)
    return max(radii) if radii else None


def analyze(project: Project, report: ValidationReport | None = None) -> DropinPlan:
    """Work out how this model maps onto the BILLEE software stack, or explain why it cannot."""
    report = report or validate_project(project)
    plan = DropinPlan(report)
    robot = report.robot
    if robot is None or "not_connected" in report.codes():
        plan.error("billee_structure", project.name, "The robot must be one connected model before it can replace robot_description.",
                   "Fix the structural issues listed above, or mount the modules together.")
        return plan

    root = robot.root
    root_link = robot.link(root)
    if root_link.mass is None:
        plan.error("billee_root_mass", root_link.source_id,
                   f"Root link {root} needs mass because it becomes base_link in the rover software.",
                   "Open the link's mass settings.")

    drive = [j for j in robot.joints if j.role is ControlRole.BASE_DRIVE and j.type is not JointType.FIXED]
    if len(drive) != 6:
        plan.error("billee_wheel_count", root_link.source_id,
                   f"The BILLEE drivetrain has six driven wheels, but this model has {len(drive)}.",
                   "Set exactly six wheel joints to Rotates freely, Actuated, and Base drive.")
        return plan

    poses = _poses(robot)
    measured = []
    for joint in sorted(drive, key=lambda j: j.name):
        if joint.type is not JointType.CONTINUOUS or joint.actuation is not Actuation.ACTUATED:
            plan.error("billee_wheel_joint", joint.source_id, f"{joint.name} must be a continuous, actuated joint.",
                       f"Select {joint.name} and set Rotates freely and Actuated.")
            continue
        link = robot.link(joint.child)
        axis = normalized(joint.axis)
        world_axis = mat_vec(poses[joint.child].R, axis)
        if abs(world_axis[1]) < AXIS_TOLERANCE:
            plan.error("billee_wheel_axis", joint.source_id,
                       f"{joint.name} must spin about the rover's left-right (Y) axis.",
                       f"Select {joint.name} and correct its axis.")
            continue
        radius = _wheel_radius(link, axis)
        if radius is None:
            plan.error("billee_wheel_shape", joint.source_id,
                       f"Could not measure the wheel radius of {joint.child}.",
                       "Give the wheel a cylinder or mesh shape centered on its joint axis.")
            continue
        measured.append((joint, link, _wheel_center(link, poses[joint.child]), radius, world_axis[1] < 0.0))
    if plan.errors or len(measured) != 6:
        return plan

    left = sorted((m for m in measured if m[2][1] > 0.0), key=lambda m: -m[2][0])
    right = sorted((m for m in measured if m[2][1] < 0.0), key=lambda m: -m[2][0])
    if len(left) != 3 or len(right) != 3:
        plan.error("billee_wheel_sides", root_link.source_id,
                   f"Expected three wheels on each side, but found {len(left)} left (+Y) and {len(right)} right (-Y).",
                   "Place three wheels on each side of the rover, with X forward and Y left.")
        return plan
    for side in (left, right):
        if side[0][2][0] - side[1][2][0] < 0.01 or side[1][2][0] - side[2][2][0] < 0.01:
            plan.error("billee_wheel_stations", side[0][0].source_id,
                       "Wheels on the same side need different front, middle, and rear X positions.",
                       "Move the wheels apart along X.")
            return plan

    radii = [m[3] for m in measured]
    if max(radii) - min(radii) > RADIUS_TOLERANCE:
        plan.error("billee_wheel_radius", root_link.source_id,
                   f"All wheels must share one radius for the drive controller, but they range from "
                   f"{min(radii):.4f} to {max(radii):.4f} m.", "Make the wheel radii match.")
        return plan
    for name, side in (("left", left), ("right", right)):
        spread = max(m[2][1] for m in side) - min(m[2][1] for m in side)
        if spread > 0.01:
            plan.warn("billee_wheel_track", side[0][0].source_id,
                      f"The {name} wheels differ by {spread * 1000:.0f} mm in Y. The controller uses their average.",
                      "Line up the wheels on this side.")

    for side_name, side in (("left", left), ("right", right)):
        for station, item in zip(("front", "mid", "rear"), side):
            joint, link, _center, _radius, flip = item
            new_joint, new_link = WHEEL_SLOTS[(side_name, station)]
            plan.wheels[joint.name] = WheelAssignment(joint.name, link.name, new_joint, new_link,
                                                      f"{station} {side_name}", flip)

    plan.radius = float(f"{sum(radii) / 6.0:.7g}")
    plan.separation = float(f"{sum(m[2][1] for m in left) / 3.0 - sum(m[2][1] for m in right) / 3.0:.7g}")
    center_x = (left[1][2][0] + right[1][2][0]) / 2.0
    center_y = (left[1][2][1] + right[1][2][1]) / 2.0
    center_z = sum(m[2][2] for m in measured) / 6.0
    plan.origin_shift = (-center_x, -center_y, -center_z)

    for joint in sorted(robot.joints, key=lambda j: j.name):
        if joint.role in (ControlRole.ARM, ControlRole.GRIPPER) and any(m.kind is ModuleKind.ARM for m in project.modules):
            continue  # The arm stays movable. billee_arm_moveit_config controls it.
        if joint.type is not JointType.FIXED and joint.name not in plan.wheels:
            plan.frozen.append(joint.name)
            plan.warn("billee_frozen_joint", joint.source_id,
                      f"{joint.name} is exported as fixed at its zero pose, because the rover software has no "
                      "controller for it.", "No action needed unless this joint must move in the rover software.")

    cameras = [l for l in robot.links if (l.name == "camera_link" or l.name.endswith("_camera_link")) and not l.has_body]
    if len(cameras) > 1:
        plan.error("billee_camera", cameras[1].source_id, "The model has more than one camera_link frame.",
                   "Keep one frame named camera_link.")
    elif cameras:
        if any(j.parent == cameras[0].name for j in robot.joints):
            plan.error("billee_camera", cameras[0].source_id, "The camera_link frame cannot have child links.",
                       "Move those links to another parent.")
        plan.camera_link = cameras[0].name
        plan.camera_pose = Transform(t=plan.origin_shift) @ poses[cameras[0].name]
    else:
        plan.warn("billee_camera", root_link.source_id,
                  "No camera_link frame found, so the ZED camera keeps its existing mounting pose.",
                  "Add a frame named camera_link where the ZED is mounted.")

    renamed = {root: "base_link", **{w.source_link: w.link for w in plan.wheels.values()}}
    for link in robot.links:
        if link.name not in renamed and link.name != plan.camera_link and (
                link.name in RESERVED_LINKS or link.name in {s[1] for s in WHEEL_SLOTS.values()}):
            plan.error("billee_reserved_name", link.source_id, f"Link name {link.name} is reserved by the rover software.",
                       "Rename the link or give its module a prefix.")
    reserved = RESERVED_JOINTS | {s[0] for s in WHEEL_SLOTS.values()}
    for joint in robot.joints:
        if joint.name not in plan.wheels and joint.name in reserved:
            plan.error("billee_reserved_name", joint.source_id, f"Joint name {joint.name} is reserved by the rover software.",
                       "Rename the joint or give its module a prefix.")

    if not plan.errors and any(m.kind is ModuleKind.ARM for m in project.modules):
        rewritten = rewrite_robot(robot, plan)
        plan.arm = analyze_arm(project, rewritten)
        plan.issues.extend(plan.arm.issues)
        if plan.arm.ok:
            plan.real_arm_gaps = real_hardware_gaps(rewritten, plan.arm)
            if plan.real_arm_gaps:
                plan.warn("arm_real_hardware", plan.arm.arm_joints[0] and rewritten.joint(plan.arm.arm_joints[0]).source_id,
                          f"Simulation and planning are ready. Real arm motors stay off: {len(plan.real_arm_gaps)} item(s) open.",
                          "See robot_description_export.md for the list.")
    return plan


def rewrite_robot(robot: FlatRobot, plan: DropinPlan) -> FlatRobot:
    """Apply the plan: standard frames, standard wheel names, and frozen non-drive joints."""
    root = robot.root
    shift = Transform(t=plan.origin_shift)
    names = {root: "base_link", **{w.source_link: w.link for w in plan.wheels.values()}}
    output = FlatRobot("robot", [FlatLink("base_footprint", robot.link(root).source_id)], [
        FlatJoint("dummy_joint", robot.link(root).source_id, JointType.FIXED, "base_footprint", "base_link",
                  Transform(t=(0.0, 0.0, plan.radius))),
    ])

    def moved(item: GeometryElement) -> GeometryElement:
        return replace(item, origin=shift @ item.origin)

    for link in robot.links:
        if link.name == plan.camera_link:
            continue
        copy = replace(link, name=names.get(link.name, link.name))
        if link.name == root:
            copy.visuals = [moved(v) for v in link.visuals]
            copy.collisions = [moved(c) for c in link.collisions]
            if link.mass is not None:
                copy.mass = replace(link.mass, com=shift.apply(link.mass.com))
        output.links.append(copy)

    for joint in robot.joints:
        if joint.child == plan.camera_link:
            continue
        copy = replace(joint, parent=names.get(joint.parent, joint.parent), child=names.get(joint.child, joint.child))
        if joint.parent == root:
            copy.origin = shift @ joint.origin
        wheel = plan.wheels.get(joint.name)
        if wheel is not None:
            copy.name = wheel.joint
            if wheel.flip_axis:
                copy.axis = scale(joint.axis, -1.0)
        elif joint.name in plan.frozen:
            copy = replace(copy, type=JointType.FIXED, limit=None, damping=0.0, friction=0.0, actuation=Actuation.FIXED)
        output.joints.append(copy)
    return output


def _is_wiring(path: Path) -> bool:
    return all((path / item).is_file() for item in REQUIRED_WIRING)


def find_wiring(explicit: str | os.PathLike | None = None) -> tuple[Path, str]:
    """Prefer a local billee-software-2027 checkout, so the current CAN IDs are always used."""
    candidates = []
    if explicit:
        candidates.append(Path(explicit).expanduser())
    if os.environ.get("BILLEE_SOFTWARE_DIR"):
        candidates.append(Path(os.environ["BILLEE_SOFTWARE_DIR"]).expanduser())
    candidates.append(Path.home() / "billee-software-2027")
    for candidate in candidates:
        for path in (candidate / "ros2_ws" / "src" / PACKAGE, candidate):
            if _is_wiring(path):
                return path, "local checkout"
    if explicit:
        raise WiringError(f"No robot_description package found in {explicit}.")
    return BUNDLED, "bundled snapshot (commit e18b1f7)"


def _read_wiring(path: Path) -> dict[str, bytes]:
    return {
        item.relative_to(path).as_posix(): item.read_bytes()
        for item in sorted(path.rglob("*"))
        if item.is_file() and "__pycache__" not in item.parts and item.suffix != ".pyc"
    }


def _patch_number(text: str, key: str, value: float) -> str:
    patched, count = re.subn(rf"(\b{key}:\s*)[-+0-9.eE]+", lambda m: m.group(1) + fmt(value), text, count=1)
    if count != 1:
        raise WiringError(f"config/controllers.yaml has no {key} value to update.")
    return patched


def _core_xacro(robot: FlatRobot, arm: ArmPlan | None = None) -> str:
    opening = '<robot name="robot">\n'
    urdf = robot_to_urdf(robot, PACKAGE)
    if opening not in urdf:
        raise WiringError("Could not prepare robot_core.urdf.xacro.")
    if arm is not None:
        urdf = urdf.replace("</robot>", simulation_block(arm) + "</robot>")
    # camera.urdf.xacro refers to the silver material by name, so the core must define it.
    replacement = ('<robot name="robot" xmlns:xacro="http://www.ros.org/wiki/xacro">\n'
                   '  <material name="silver">\n    <color rgba="0.7 0.7 0.7 1" />\n  </material>\n')
    return urdf.replace(opening, replacement, 1)


def build_files(project: Project, wiring: str | os.PathLike | None = None) -> tuple[dict[str, bytes], DropinPlan, str]:
    """Return the complete package as {path: bytes}. Nothing is written."""
    plan = analyze(project)
    if not plan.ready:
        blockers = plan.errors + plan.report.blockers(Target.PHYSICS)
        raise ExportBlocked(Target.PHYSICS, blockers)
    source, label = find_wiring(wiring)
    files = _read_wiring(source)
    missing = [item for item in REQUIRED_WIRING if item not in files]
    if missing:
        raise WiringError(f"The robot_description package is missing: {', '.join(missing)}.")

    control = files["urdf/ros2_control.urdf.xacro"].decode("utf-8")
    absent = [s[0] for s in WHEEL_SLOTS.values() if f'name="{s[0]}"' not in control]
    if absent:
        raise WiringError(f"ros2_control.urdf.xacro does not define: {', '.join(absent)}.")

    robot = rewrite_robot(plan.report.robot, plan)
    files["urdf/robot_core.urdf.xacro"] = _core_xacro(robot, plan.arm).encode("utf-8")

    controllers = files["config/controllers.yaml"].decode("utf-8")
    controllers = _patch_number(controllers, "wheel_separation", plan.separation)
    controllers = _patch_number(controllers, "wheel_radius", plan.radius)
    files["config/controllers.yaml"] = controllers.encode("utf-8")

    known = {l.name for l in robot.links} | {"camera_link", "camera_link_optical"}
    gazebo = files["urdf/robot_gz.urdf.xacro"].decode("utf-8")
    gazebo = re.sub(r'[ \t]*<gazebo reference="([^"]+)">.*?</gazebo>\n?',
                    lambda m: m.group(0) if m.group(1) in known else "", gazebo, flags=re.S)
    files["urdf/robot_gz.urdf.xacro"] = gazebo.encode("utf-8")

    if plan.camera_pose is not None:
        camera = files["urdf/camera.urdf.xacro"].decode("utf-8")
        xyz = " ".join(fmt(0.0 if abs(v) < 1e-12 else v) for v in plan.camera_pose.t)
        rpy = " ".join(fmt(0.0 if abs(v) < 1e-12 else v) for v in plan.camera_pose.rpy())
        camera, count = re.subn(r'(<joint name="camera_joint".*?<origin )xyz="[^"]*" rpy="[^"]*"',
                                lambda m: f'{m.group(1)}xyz="{xyz}" rpy="{rpy}"', camera, count=1, flags=re.S)
        if count != 1:
            raise WiringError("camera.urdf.xacro has no camera_joint origin to update.")
        files["urdf/camera.urdf.xacro"] = camera.encode("utf-8")

    for link in robot.links:
        for item in [*link.visuals, *link.collisions]:
            if isinstance(item.shape, Mesh):
                files[f"meshes/{mesh_filename(item.shape.asset)}"] = mesh_to_stl_bytes(item.shape)
    return dict(sorted(files.items())), plan, label


def _digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _report(plan: DropinPlan, label: str) -> str:
    lines = [
        "# robot_description export", "",
        "Replace `billee-software-2027/ros2_ws/src/robot_description` with the `robot_description` folder, then:", "",
        "```sh", "cd ros2_ws", "pixi run build --packages-select robot_description", "```", "",
        f"- Software wiring source: {label}",
        f"- Wheel radius: {fmt(plan.radius)} m",
        f"- Wheel separation: {fmt(plan.separation)} m", "",
        "| Wheel | Your joint | Rover joint |", "|---|---|---|",
    ]
    for wheel in sorted(plan.wheels.values(), key=lambda w: w.joint):
        lines.append(f"| {wheel.position} | `{wheel.source_joint}` | `{wheel.joint}` |")
    if plan.frozen:
        lines += ["", "Exported as fixed, because the rover software has no controller for them:", ""]
        lines += [f"- `{name}`" for name in plan.frozen]
    warnings = [i for i in plan.issues if i.severity is Severity.WARNING and i.code != "billee_frozen_joint"]
    if warnings:
        lines += ["", "Warnings:", ""] + [f"- {i.message}" for i in warnings]
    if plan.arm is not None:
        lines += ["", "## Arm", "",
                  f"`{ARM_PACKAGE}` is a separate package. Copy it into `ros2_ws/src/` beside `robot_description`.",
                  "It needs MoveIt 2 for Humble, which is not in the current software environment.", "",
                  f"- Planning group `arm`: {', '.join(plan.arm.arm_joints)}",
                  f"- End effector group `gripper`: {', '.join(plan.arm.gripper_joints) or 'none'}"]
        for name, joints in plan.arm.controllers.items():
            lines.append(f"- `{name}`: {', '.join(joints)}")
        if plan.real_arm_gaps:
            lines += ["", "Real arm motors stay off until these are resolved:", ""]
            lines += [f"- {gap}" for gap in plan.real_arm_gaps]
    return "\n".join(lines) + "\n"


@dataclass
class DropinResult:
    path: Path
    report_path: Path
    plan: DropinPlan
    wiring: str
    moveit_path: Path | None = None


def _check_destination(parent: Path, name: str) -> None:
    destination = parent / name
    manifest_path = parent / f".{name}.manifest.json"
    if not destination.exists():
        return
    if not manifest_path.is_file():
        raise PackageConflict(f"{destination} already exists and was not generated here. Choose another folder, "
                              "then drag the result over the existing package.")
    recorded = json.loads(manifest_path.read_text(encoding="utf-8"))["files"]
    current = {p.relative_to(destination).as_posix(): _digest(p.read_bytes())
               for p in destination.rglob("*") if p.is_file()}
    changed = sorted(k for k in set(recorded) | set(current) if recorded.get(k) != current.get(k))
    if changed:
        raise PackageConflict("These files in the previous export were changed: " + ", ".join(changed[:8]))


def _install_tree(parent: Path, name: str, files: dict[str, bytes]) -> Path:
    """Build in staging, then swap into place. An unchanged earlier export is replaced."""
    destination = parent / name
    staging = Path(tempfile.mkdtemp(prefix=f".{name}.staging-", dir=parent))
    try:
        built = staging / name
        for relative, data in files.items():
            path = built / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
        previous = None
        if destination.exists():
            previous = staging / "previous"
            os.replace(destination, previous)
        try:
            os.replace(built, destination)
        except OSError:
            if previous is not None:
                os.replace(previous, destination)
            raise
        manifest = {"generator": "rover_arm_builder", "files": {k: _digest(v) for k, v in files.items()}}
        (parent / f".{name}.manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n",
                                                       encoding="utf-8", newline="\n")
    finally:
        shutil.rmtree(staging, ignore_errors=True)
    return destination


def write_dropin(project: Project, output: str | os.PathLike, wiring: str | os.PathLike | None = None) -> DropinResult:
    """Write robot_description, plus billee_arm_moveit_config when the project has an arm."""
    files, plan, label = build_files(project, wiring)
    moveit = None
    if plan.arm is not None:
        moveit = build_moveit_files(rewrite_robot(plan.report.robot, plan), plan.arm)
    parent = Path(output).expanduser()
    _check_destination(parent, PACKAGE)
    if moveit is not None:
        _check_destination(parent, ARM_PACKAGE)
    parent.mkdir(parents=True, exist_ok=True)
    destination = _install_tree(parent, PACKAGE, files)
    moveit_path = _install_tree(parent, ARM_PACKAGE, moveit) if moveit is not None else None
    report_path = parent / f"{PACKAGE}_export.md"
    report_path.write_text(_report(plan, label), encoding="utf-8", newline="\n")
    return DropinResult(destination, report_path, plan, label, moveit_path)


def write_arm_only(project: Project, output: str | os.PathLike) -> tuple[Path, ArmPlan]:
    """Write a standalone MoveIt package for the arm module, with its own model and mock hardware."""
    arms = [m for m in project.modules if m.kind is ModuleKind.ARM]
    if len(arms) != 1:
        raise ValueError("Standalone arm export needs exactly one arm module.")
    report = validate_project(project, [arms[0].id], "billee_arm")
    if not report.ok(Target.PHYSICS):
        raise ExportBlocked(Target.PHYSICS, report.blockers(Target.PHYSICS))
    plan = analyze_arm(Project("billee_arm", [arms[0]]), report.robot)
    if not plan.ok:
        raise ExportBlocked(Target.PLANNING, [i for i in plan.issues if i.severity is Severity.ERROR])
    parent = Path(output).expanduser()
    _check_destination(parent, ARM_PACKAGE)
    parent.mkdir(parents=True, exist_ok=True)
    return _install_tree(parent, ARM_PACKAGE, build_moveit_files(report.robot, plan, standalone=True)), plan

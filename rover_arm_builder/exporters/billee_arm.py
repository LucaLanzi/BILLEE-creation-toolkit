"""Generate the arm packages for billee-software-2027.

arm_description (drop-in): the team's package with `urdf/arm_model.urdf.xacro` regenerated, using the
contract of its import_urdf.py. arm_bringup/config/arm_axes.yaml stays the source of truth for motors.

billee_arm_moveit_config: MoveIt 2 for the rover and arm together. It drives the existing /arm
controllers through arm_moveit_bridge.py and does not replace or modify any of them.
"""

from __future__ import annotations

import json
import os
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field, replace
from pathlib import Path

from ..core.assembly import FlatJoint, FlatLink, FlatRobot, frame_pose_in_root, link_pose_in_module
from ..core.model import Actuation, ControlRole, FrameKind, JointType, Mesh, Module, ModuleKind, Project
from ..core.transforms import Transform
from ..core.validate import Issue, Severity, Target, ValidationReport, validate_project
from . import billee as rover
from .package import PackageConflict
from .stl import mesh_to_stl_bytes, tessellate
from .urdf import ExportBlocked, fmt, fmt_pose, mesh_filename, robot_to_urdf
from .xacro import XACRO_NS

ARM_PACKAGE = "arm_description"
PACKAGE = "billee_arm_moveit_config"
RESOURCES = Path(__file__).resolve().parent / "resources"
REQUIRED_ARM = ("package.xml", "setup.py", "launch/arm_description.launch.py", "urdf/arm.urdf.xacro",
                "urdf/arm_model.urdf.xacro", "urdf/arm_standalone.urdf.xacro")
ACCELERATION_FACTOR = 2.0
__all__ = ["PackageConflict", "analyze_arm", "write_arm", "write_assembly"]


@dataclass(frozen=True)
class Axis:
    joint: str
    driver: str
    output: str
    min_position: float | None
    max_position: float | None
    max_velocity: float | None
    verified: bool


def parse_axes(text: str) -> list[Axis]:
    """Read arm_axes.yaml without PyYAML, which Blender does not bundle."""
    clean = "\n".join(line.split("#", 1)[0].rstrip() for line in text.splitlines())
    axes = []
    for match in re.finditer(r"- joint:\s*(\S+)(.*?)(?=\n\s*- joint:|\Z)", clean, re.S):
        body = match.group(2)

        def value(key):
            found = re.search(rf"\b{key}:\s*(\S+)", body)
            return found.group(1) if found else None

        def number(key):
            raw = value(key)
            return None if raw is None else float(raw)

        axes.append(Axis(match.group(1), value("driver") or "", value("output") or "motor", number("min_position"),
                         number("max_position"), number("max_velocity"), value("verified") == "true"))
    if not axes:
        raise rover.WiringError("arm_axes.yaml lists no axes.")
    return axes


def find_arm_wiring(explicit: str | os.PathLike | None = None) -> tuple[Path, Path, str]:
    candidates = []
    if explicit:
        candidates.append(Path(explicit).expanduser())
    if os.environ.get("BILLEE_SOFTWARE_DIR"):
        candidates.append(Path(os.environ["BILLEE_SOFTWARE_DIR"]).expanduser())
    candidates.append(Path.home() / "billee-software-2027")
    for candidate in candidates:
        options = ((candidate / "ros2_ws" / "src" / ARM_PACKAGE,
                    candidate / "ros2_ws" / "src" / "arm_bringup" / "config" / "arm_axes.yaml"),
                   (candidate / ARM_PACKAGE, candidate / "arm_axes.yaml"))
        for package, axes in options:
            if axes.is_file() and all((package / item).is_file() for item in REQUIRED_ARM):
                return package, axes, rover.wiring_label(package)
    if explicit:
        raise rover.WiringError(f"No arm_description and arm_axes.yaml found in {explicit}.")
    package = rover.BUNDLED_ROOT / ARM_PACKAGE
    return package, rover.BUNDLED_ROOT / "arm_axes.yaml", rover.wiring_label(package)


@dataclass
class ArmPlan:
    report: ValidationReport
    module: Module | None = None
    issues: list[Issue] = field(default_factory=list)
    names: dict[str, str] = field(default_factory=dict)
    joints: dict[str, str] = field(default_factory=dict)
    dropped: set[str] = field(default_factory=set)
    frozen: list[str] = field(default_factory=list)
    arm_joints: list[str] = field(default_factory=list)
    gripper: str | None = None
    gripper_parent: str = ""
    tip_owner: str = ""
    axes: dict[str, Axis] = field(default_factory=dict)
    poses: dict[str, dict[str, float]] = field(default_factory=dict)

    @property
    def errors(self) -> list[Issue]:
        return [i for i in self.issues if i.severity is Severity.ERROR]

    @property
    def ready(self) -> bool:
        return not self.errors and self.report.ok(Target.PLANNING)

    @property
    def unverified(self) -> list[str]:
        used = [*self.arm_joints, *([self.gripper] if self.gripper else [])]
        return [name for name in used if name in self.axes and not self.axes[name].verified]

    def error(self, code: str, entity: str, message: str, action: str) -> None:
        self.issues.append(Issue(Severity.ERROR, code, entity, message, action))

    def warn(self, code: str, entity: str, message: str, action: str) -> None:
        self.issues.append(Issue(Severity.WARNING, code, entity, message, action))


def analyze_arm(project: Project, axes: list[Axis] | None = None) -> ArmPlan:
    """Map the arm module onto the joints in arm_axes.yaml, by order from base to tool."""
    if axes is None:
        axes = parse_axes(find_arm_wiring()[1].read_text(encoding="utf-8"))
    modules = [m for m in project.modules if m.kind is ModuleKind.ARM]
    if len(modules) != 1:
        report = ValidationReport()
        plan = ArmPlan(report)
        plan.error("arm_module", project.name, f"Expected one arm module, found {len(modules)}.",
                   "Create one arm module in the Arm tab.")
        return plan
    module = modules[0]
    report = validate_project(Project(project.name, [module]), [module.id], "billee_arm")
    plan = ArmPlan(report, module, axes={a.joint: a for a in axes if a.output != "duty"})
    robot = report.robot
    if robot is None:
        plan.error("arm_structure", module.id, "Fix the arm's structural issues first.", "See the issues above.")
        return plan

    tools = [f for f in module.frames if f.kind is FrameKind.TOOL]
    if len(tools) != 1:
        plan.error("arm_tool", module.id, "The arm needs exactly one tool frame at its tip.",
                   "Add a frame of type Tool in the Arm tab.")
        return plan
    tool = f"{module.prefix}{tools[0].name}"
    by_child = {j.child: j for j in robot.joints}
    chain: list[FlatJoint] = []
    current = tool
    while current in by_child:
        joint = by_child[current]
        if joint.role is ControlRole.ARM and joint.type is not JointType.FIXED:
            chain.append(joint)
        current = joint.parent
    chain.reverse()
    declared = {j.name for j in robot.joints if j.role is ControlRole.ARM and j.type is not JointType.FIXED}
    for name in sorted(declared - {j.name for j in chain}):
        plan.error("arm_chain", robot.joint(name).source_id,
                   f"{name} has the Arm role but is not between the arm base and tool frame.",
                   "Correct its role or move the tool frame.")

    expected = [a.joint for a in axes if a.joint.startswith("joint_arm_") and a.output != "duty"]
    if len(chain) != len(expected):
        plan.error("arm_axis_count", module.id,
                   f"arm_axes.yaml defines {len(expected)} arm axes, but this arm has {len(chain)} between base and tool.",
                   "Give each axis one joint with the Arm / manipulation role.")
        return plan
    for joint in chain:
        if joint.actuation is not Actuation.ACTUATED or joint.type not in (JointType.REVOLUTE, JointType.CONTINUOUS):
            plan.error("arm_joint", joint.source_id, f"{joint.name} must be an actuated rotary joint.",
                       f"Select {joint.name} and set a rotary motion and Actuated.")

    plan.names[robot.root] = "base_link"
    for index, joint in enumerate(chain):
        plan.joints[joint.name] = expected[index]
        plan.names[joint.child] = f"link_{index + 1}"
    plan.arm_joints = list(expected)
    plan.names[tool] = "tool_frame"
    tool_joint = by_child.get(tool)
    plan.tip_owner = plan.names.get(tool_joint.parent, "") if tool_joint else ""

    grippers = [j for j in robot.joints if j.role is ControlRole.GRIPPER and j.type is not JointType.FIXED]
    if len(grippers) > 1:
        plan.error("arm_gripper", grippers[1].source_id, "The arm can have one End effector joint.",
                   "Keep one joint with the End effector role.")
    elif grippers and "end_effector" not in plan.axes:
        plan.error("arm_gripper", grippers[0].source_id, "arm_axes.yaml has no end_effector axis for this joint.",
                   "Remove the End effector role or add the axis to arm_axes.yaml.")
    elif grippers:
        plan.gripper = "end_effector"
        plan.joints[grippers[0].name] = "end_effector"
        plan.names[grippers[0].child] = "end_effector_link"
        plan.gripper_parent = plan.names.get(grippers[0].parent, "")
    else:
        plan.warn("arm_gripper", module.id, "No End effector joint, so MoveIt will have no gripper group.",
                  "Set the end effector's joint to the End effector role.")

    for frame in module.frames:
        if frame.kind is FrameKind.MOUNT:
            plan.dropped.add(f"{module.prefix}{frame.name}")
    for link in robot.links:
        if link.name not in plan.names and link.name not in plan.dropped:
            plan.names[link.name] = "extra_" + re.sub(r"\W", "_", link.name)
    for joint in robot.joints:
        if joint.type is not JointType.FIXED and joint.name not in plan.joints:
            plan.frozen.append(joint.name)
            plan.warn("arm_frozen_joint", joint.source_id, f"{joint.name} is exported as fixed, because arm_axes.yaml has no axis for it.",
                      "Give it the Arm or End effector role if it should move.")

    for joint in [*chain, *grippers[:1]]:
        name = plan.joints[joint.name]
        axis = plan.axes.get(name)
        limit = joint.limit
        if limit is None or not limit.velocity:
            plan.error("arm_velocity", joint.source_id, f"{joint.name} needs a velocity limit above zero.",
                       f"Select {joint.name} and open its joint limits.")
        if axis and limit and limit.lower is not None and axis.min_position is not None and (
                limit.lower < axis.min_position or limit.upper > axis.max_position):
            plan.error("arm_limits", joint.source_id,
                       f"{joint.name} limits exceed {name} in arm_axes.yaml ({axis.min_position} to {axis.max_position} rad).",
                       f"Select {joint.name} and narrow its limits.")

    source = {j.id: plan.joints.get(f"{module.prefix}{j.name}") for j in module.joints}
    for pose, values in module.poses.items():
        plan.poses[pose] = {source[k]: v for k, v in values.items() if source.get(k) in plan.arm_joints}
    plan.poses.setdefault("home", {name: 0.0 for name in plan.arm_joints})
    return plan


def arm_robot(plan: ArmPlan, prefix: str, movable_gripper: bool = False) -> FlatRobot:
    """The arm with contract names. Arm joints stay unprefixed, as import_urdf.py requires."""
    robot = plan.report.robot
    output = FlatRobot("billee_arm")
    for link in robot.links:
        if link.name in plan.dropped:
            continue

        def material(item):
            # arm_description's contract expects mesh visuals. Collision shapes stay analytic.
            if not isinstance(item.shape, Mesh):
                asset = "toolkit_" + re.sub(r"\W", "_", item.name)
                item = replace(item, shape=Mesh(asset, tessellate(item.shape)))
            if item.material is None:
                return item
            clean = re.sub(r"\W", "_", item.material.name.removeprefix(plan.module.prefix))
            return replace(item, material=replace(item.material, name=f"{prefix}{clean}"))

        output.links.append(replace(link, name=f"{prefix}{plan.names[link.name]}",
                                    visuals=[material(v) for v in link.visuals]))
    for joint in robot.joints:
        if joint.child in plan.dropped:
            continue
        name = plan.joints.get(joint.name)
        copy = replace(joint, parent=f"{prefix}{plan.names[joint.parent]}", child=f"{prefix}{plan.names[joint.child]}",
                       name=name or f"{prefix}{plan.names[joint.child]}_joint")
        if joint.name in plan.frozen or (name == "end_effector" and not movable_gripper):
            copy = replace(copy, type=JointType.FIXED, limit=None, damping=0.0, friction=0.0, actuation=Actuation.FIXED)
        output.joints.append(copy)
    return output


def _model_xacro(plan: ArmPlan) -> str:
    """arm_model.urdf.xacro. The end effector is fixed in TF unless MoveIt's model enables it."""
    robot = arm_robot(plan, "${prefix}", movable_gripper=True)
    parsed = ET.fromstring(robot_to_urdf(robot, ARM_PACKAGE))
    if plan.gripper:
        for joint in parsed.findall("joint"):
            if joint.get("name") == plan.gripper:
                joint.set("type", "${arm_model_end_effector_type}")
    root = ET.Element("robot")
    tag = lambda name: f"{{{XACRO_NS}}}{name}"
    ET.SubElement(root, tag("property"), {"name": "arm_model_root_link", "value": "base_link"})
    ET.SubElement(root, tag("property"), {"name": "arm_model_tip_link", "value": "tool_frame"})
    ET.SubElement(root, tag("property"), {"name": "arm_model_end_effector_type", "default": "fixed"})
    macro = ET.SubElement(root, tag("macro"), {"name": "arm_model", "params": "prefix"})
    macro.extend(list(parsed))
    ET.indent(root, space="  ")
    return ('<?xml version="1.0"?>\n<!-- GENERATED by the BILLEE Creation Toolkit. Export again rather than editing. -->\n'
            + ET.tostring(root, encoding="unicode") + "\n")


def _patch_mount(files: dict[str, bytes], mount: Transform) -> None:
    xyz = " ".join(fmt_pose(v) for v in mount.t)
    rpy = " ".join(fmt_pose(v) for v in mount.rpy())
    edits = (("urdf/arm_standalone.urdf.xacro", r'(<xacro:arg name="mount_{}" default=")[^"]*'),
             ("launch/arm_description.launch.py", r'("mount_{}": \(")[^"]*'))
    for path, pattern in edits:
        text = files[path].decode("utf-8")
        for key, value in (("xyz", xyz), ("rpy", rpy)):
            text, count = re.subn(pattern.format(key), lambda m: m.group(1) + value, text, count=1)
            if count != 1:
                raise rover.WiringError(f"{path} has no mount_{key} default to update.")
        files[path] = text.encode("utf-8")


def build_arm_files(plan: ArmPlan, package: Path, mount: Transform | None = None) -> dict[str, bytes]:
    files = rover._read_wiring(package)
    files["urdf/arm_model.urdf.xacro"] = _model_xacro(plan).encode("utf-8")
    if mount is not None:
        _patch_mount(files, mount)
    for link in arm_robot(plan, "").links:
        for item in [*link.visuals, *link.collisions]:
            if isinstance(item.shape, Mesh):
                files[f"meshes/{mesh_filename(item.shape.asset)}"] = mesh_to_stl_bytes(item.shape)
    return dict(sorted(files.items()))


def _srdf(plan: ArmPlan, rover_robot: FlatRobot | None) -> str:
    root = ET.Element("robot", {"name": "robot" if rover_robot else "billee_arm"})
    group = ET.SubElement(root, "group", {"name": "arm"})
    ET.SubElement(group, "chain", {"base_link": "arm_base_link", "tip_link": "arm_tool_link"})
    if plan.gripper:
        ET.SubElement(ET.SubElement(root, "group", {"name": "gripper"}), "joint", {"name": plan.gripper})
    for pose in sorted(plan.poses):
        state = ET.SubElement(root, "group_state", {"name": pose, "group": "arm"})
        for name in plan.arm_joints:
            ET.SubElement(state, "joint", {"name": name, "value": fmt(plan.poses[pose].get(name, 0.0))})
    if plan.gripper:
        joint = next(j for j in plan.report.robot.joints if plan.joints.get(j.name) == plan.gripper)
        for pose, value in (("open", joint.limit.upper), ("closed", joint.limit.lower)):
            state = ET.SubElement(root, "group_state", {"name": pose, "group": "gripper"})
            ET.SubElement(state, "joint", {"name": plan.gripper, "value": fmt(value)})
        ET.SubElement(root, "end_effector", {"name": "gripper", "parent_link": f"arm_{plan.gripper_parent}",
                                             "group": "gripper", "parent_group": "arm"})

    # Conservative: only directly connected links, and the camera rigidly fixed to the tool link.
    arm = arm_robot(plan, "arm_", movable_gripper=True)
    robots = [arm, *([rover_robot] if rover_robot else [])]
    colliding = {l.name for r in robots for l in r.links if l.collisions}
    pairs = {tuple(sorted((j.parent, j.child))) for r in robots for j in r.joints
             if j.parent in colliding and j.child in colliding}
    if rover_robot and "base_link" in colliding and "arm_base_link" in colliding:
        pairs.add(("arm_base_link", "base_link"))
    if plan.tip_owner:
        pairs.add(tuple(sorted(("arm_cam_link", f"arm_{plan.tip_owner}"))))
    for first, second in sorted(pairs):
        ET.SubElement(root, "disable_collisions", {"link1": first, "link2": second, "reason": "Adjacent"})
    ET.indent(root, space="  ")
    return '<?xml version="1.0"?>\n' + ET.tostring(root, encoding="unicode") + "\n"


def _moveit_model(mount: Transform | None) -> str:
    gripper = '  <xacro:property name="arm_model_end_effector_type" value="revolute"/>\n'
    arm = '  <xacro:include filename="$(find arm_description)/urdf/arm.urdf.xacro"/>\n'
    if mount is None:
        return ('<?xml version="1.0"?>\n<robot name="billee_arm" xmlns:xacro="http://www.ros.org/wiki/xacro">\n'
                + gripper + arm + '  <link name="base_link"/>\n  <xacro:billee_arm parent="base_link" prefix="arm_">\n'
                '    <origin xyz="0 0 0" rpy="0 0 0"/>\n  </xacro:billee_arm>\n</robot>\n')
    xyz = " ".join(fmt_pose(v) for v in mount.t)
    rpy = " ".join(fmt_pose(v) for v in mount.rpy())
    return ('<?xml version="1.0"?>\n<robot name="robot" xmlns:xacro="http://www.ros.org/wiki/xacro">\n' + gripper
            + '  <xacro:include filename="$(find robot_description)/urdf/robot_core.urdf.xacro"/>\n'
            '  <xacro:include filename="$(find robot_description)/urdf/camera.urdf.xacro"/>\n' + arm
            + f'  <xacro:billee_arm parent="base_link" prefix="arm_">\n    <origin xyz="{xyz}" rpy="{rpy}"/>\n'
            "  </xacro:billee_arm>\n</robot>\n")


def _joint_limits(plan: ArmPlan) -> str:
    lines = ["# Velocity is the lower of the Blender limit and arm_axes.yaml. Acceleration is a planning default.",
             "default_velocity_scaling_factor: 0.2", "default_acceleration_scaling_factor: 0.2", "joint_limits:"]
    by_name = {plan.joints[j.name]: j for j in plan.report.robot.joints if j.name in plan.joints}
    for name in [*plan.arm_joints, *([plan.gripper] if plan.gripper else [])]:
        velocity = by_name[name].limit.velocity
        if plan.axes[name].max_velocity is not None:
            velocity = min(velocity, plan.axes[name].max_velocity)
        lines += [f"  {name}:", "    has_velocity_limits: true", f"    max_velocity: {float(fmt(velocity))!r}",
                  "    has_acceleration_limits: true",
                  f"    max_acceleration: {float(fmt(velocity * ACCELERATION_FACTOR))!r}"]
    return "\n".join(lines) + "\n"


def _controllers(plan: ArmPlan) -> str:
    groups = {"billee_arm_controller": plan.arm_joints}
    if plan.gripper:
        groups["billee_gripper_controller"] = [plan.gripper]
    lines = ["moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager",
             "trajectory_execution:", "  allowed_execution_duration_scaling: 2.0", "  allowed_goal_duration_margin: 3.0",
             "  allowed_start_tolerance: 0.05", "moveit_simple_controller_manager:",
             f"  controller_names: {json.dumps(list(groups))}"]
    for name, joints in groups.items():
        lines += [f"  {name}:", "    type: FollowJointTrajectory", "    action_ns: follow_joint_trajectory",
                  "    default: true", f"    joints: {json.dumps(joints)}"]
    return "\n".join(lines) + "\n"


_MOVEIT_LAUNCH = '''"""MoveIt for the BILLEE arm. Start the rover and arm first (the dashboard's Teleop - Arm mode does)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

PACKAGE = "billee_arm_moveit_config"


def _setup(context):
    share = get_package_share_directory(PACKAGE)
    config = (
        MoveItConfigsBuilder("@ROBOT@", package_name=PACKAGE)
        .robot_description(file_path=os.path.join(share, "urdf", "moveit_model.urdf.xacro"))
        .robot_description_semantic(file_path="config/robot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=False, publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "arm_bridge.launch.py")),
        launch_arguments={"allow_unverified": LaunchConfiguration("allow_unverified")}.items(),
        condition=IfCondition(LaunchConfiguration("start_bridge")),
    )
    # The arm runs on the wall clock in every mode, so move_group does too.
    move_group = Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        parameters=[config.to_dict(), {"use_sim_time": False}],
        remappings=[("joint_states", "/billee_arm/joint_states")],
    )
    return [bridge, move_group]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("allow_unverified", default_value="false",
                              description="true allows axes not yet verified in arm_axes.yaml. Mock hardware only."),
        DeclareLaunchArgument("start_bridge", default_value="true",
                              description="false when arm_bridge.launch.py already runs on the rover."),
        OpaqueFunction(function=_setup),
    ])
'''

_BRIDGE_LAUNCH = '''"""The bridge between MoveIt and the /arm controllers. On real hardware, run it on the rover."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("billee_arm_moveit_config")
    return LaunchDescription([
        DeclareLaunchArgument("allow_unverified", default_value="false"),
        Node(
            package="billee_arm_moveit_config", executable="arm_moveit_bridge.py", name="billee_arm_bridge",
            output="screen", sigterm_timeout="5",
            parameters=[os.path.join(share, "config", "bridge.yaml"), {
                "allow_unverified": ParameterValue(LaunchConfiguration("allow_unverified"), value_type=bool),
            }],
        ),
    ])
'''


def build_moveit_files(plan: ArmPlan, rover_robot: FlatRobot | None, mount: Transform | None) -> dict[str, bytes]:
    depends = ["arm_bringup", "arm_description", "control_msgs", "moveit_configs_utils", "moveit_kinematics",
               "moveit_planners_ompl", "moveit_ros_move_group", "moveit_simple_controller_manager", "rclpy",
               "sensor_msgs", "std_msgs", "trajectory_msgs"]
    if rover_robot is not None:
        depends.insert(2, "robot_description")
    package_xml = (f'<?xml version="1.0"?>\n<package format="3">\n  <name>{PACKAGE}</name>\n  <version>0.1.0</version>\n'
                   "  <description>MoveIt 2 for the BILLEE arm, generated by the BILLEE Creation Toolkit.</description>\n"
                   '  <maintainer email="maintainer@example.invalid">BILLEE team</maintainer>\n'
                   "  <license>GPL-3.0-or-later</license>\n  <buildtool_depend>ament_cmake</buildtool_depend>\n"
                   + "".join(f"  <exec_depend>{d}</exec_depend>\n" for d in depends)
                   + "  <export><build_type>ament_cmake</build_type></export>\n</package>\n")
    cmake = (f"cmake_minimum_required(VERSION 3.16)\nproject({PACKAGE} NONE)\nfind_package(ament_cmake REQUIRED)\n"
             "install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})\n"
             "install(PROGRAMS scripts/arm_moveit_bridge.py DESTINATION lib/${PROJECT_NAME})\nament_package()\n")
    position_only = "true" if len(plan.arm_joints) < 6 else "false"
    kinematics = ("arm:\n  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin\n"
                  "  kinematics_solver_search_resolution: 0.005\n  kinematics_solver_timeout: 0.05\n"
                  f"  position_only_ik: {position_only}\n")
    bridge = ("billee_arm_bridge:\n  ros__parameters:\n"
              f"    arm_joints: {json.dumps(plan.arm_joints)}\n"
              f"    gripper_joints: {json.dumps([plan.gripper] if plan.gripper else [''])}\n")
    text = {
        "package.xml": package_xml,
        "CMakeLists.txt": cmake,
        "config/robot.srdf": _srdf(plan, rover_robot),
        "config/kinematics.yaml": kinematics,
        "config/joint_limits.yaml": _joint_limits(plan),
        "config/moveit_controllers.yaml": _controllers(plan),
        "config/bridge.yaml": bridge,
        "launch/moveit.launch.py": _MOVEIT_LAUNCH.replace("@ROBOT@", "robot" if rover_robot else "billee_arm"),
        "launch/arm_bridge.launch.py": _BRIDGE_LAUNCH,
        "urdf/moveit_model.urdf.xacro": _moveit_model(mount),
    }
    files = {path: content.encode("utf-8") for path, content in text.items()}
    files["scripts/arm_moveit_bridge.py"] = (RESOURCES / "arm_moveit_bridge.py").read_bytes()
    return dict(sorted(files.items()))


def mount_pose(project: Project, rover_plan: rover.DropinPlan) -> Transform:
    """T_baseLink_armRoot = T_baseLink_roverFrame * T_offset * inverse(T_armRoot_armFrame)."""
    arm = next(m for m in project.modules if m.kind is ModuleKind.ARM)
    mounts = [m for m in project.mounts if m.child_module_id == arm.id]
    if len(mounts) != 1:
        raise rover.WiringError("Mount the arm to the rover in the Assembly tab.")
    mount = mounts[0]
    parent = project.module(mount.parent_module_id)
    if parent is None or parent.kind is not ModuleKind.ROVER:
        raise rover.WiringError("The arm must be mounted on the rover module.")
    frame = next(f for f in parent.frames if f.id == mount.parent_frame_id)
    pose = Transform(t=rover_plan.origin_shift) @ link_pose_in_module(parent, frame.link_id) @ frame.pose
    return pose @ mount.offset @ frame_pose_in_root(arm, mount.child_frame_id).inverse()


def _report(plan: ArmPlan, label: str, mount: Transform | None) -> str:
    lines = ["# Arm export", "", f"- Software wiring source: {label}"]
    if mount is not None:
        lines.append(f"- Mount on base_link: xyz {' '.join(fmt_pose(v) for v in mount.t)}, "
                     f"rpy {' '.join(fmt_pose(v) for v in mount.rpy())}")
    lines += ["", "| Your joint | BILLEE joint | Driver | Bench verified |", "|---|---|---|---|"]
    for source, name in plan.joints.items():
        axis = plan.axes[name]
        lines.append(f"| `{source}` | `{name}` | {axis.driver} | {'yes' if axis.verified else 'no'} |")
    if plan.unverified:
        lines += ["", "MoveIt refuses to move these axes until `verified: true` is set in `arm_axes.yaml`:", "",
                  ", ".join(plan.unverified)]
    if plan.frozen:
        lines += ["", "Exported as fixed: " + ", ".join(plan.frozen)]
    return "\n".join(lines) + "\n"


@dataclass
class ArmExport:
    arm_path: Path
    moveit_path: Path
    report_path: Path
    plan: ArmPlan
    wiring: str
    rover_path: Path | None = None


def _load(wiring) -> tuple[Path, list[Axis], str]:
    package, axes_path, label = find_arm_wiring(wiring)
    return package, parse_axes(axes_path.read_text(encoding="utf-8")), label


def _blocked(plan: ArmPlan) -> None:
    if not plan.ready:
        raise ExportBlocked(Target.PLANNING, plan.errors + plan.report.blockers(Target.PLANNING))


def write_arm(project: Project, output, wiring=None) -> ArmExport:
    """Write arm_description and a MoveIt package for the arm alone."""
    package, axes, label = _load(wiring)
    plan = analyze_arm(project, axes)
    _blocked(plan)
    files = build_arm_files(plan, package)
    moveit = build_moveit_files(plan, None, None)
    parent = Path(output).expanduser()
    rover._check_destination(parent, ARM_PACKAGE)
    rover._check_destination(parent, PACKAGE)
    parent.mkdir(parents=True, exist_ok=True)
    arm_path = rover._install_tree(parent, ARM_PACKAGE, files)
    moveit_path = rover._install_tree(parent, PACKAGE, moveit)
    report = parent / "arm_export.md"
    report.write_text(_report(plan, label, None), encoding="utf-8", newline="\n")
    return ArmExport(arm_path, moveit_path, report, plan, label)


def write_assembly(project: Project, output, wiring=None) -> ArmExport:
    """Write robot_description, arm_description, and MoveIt for the mounted robot."""
    package, axes, label = _load(wiring)
    rover_files, rover_plan, rover_label = rover.build_files(project, wiring)
    plan = analyze_arm(project, axes)
    _blocked(plan)
    mount = mount_pose(project, rover_plan)
    rover_robot = rover.rewrite_robot(rover_plan.report.robot, rover_plan)
    arm_files = build_arm_files(plan, package, mount)
    moveit = build_moveit_files(plan, rover_robot, mount)
    parent = Path(output).expanduser()
    for name in (rover.PACKAGE, ARM_PACKAGE, PACKAGE):
        rover._check_destination(parent, name)
    parent.mkdir(parents=True, exist_ok=True)
    rover_path = rover._install_tree(parent, rover.PACKAGE, rover_files)
    arm_path = rover._install_tree(parent, ARM_PACKAGE, arm_files)
    moveit_path = rover._install_tree(parent, PACKAGE, moveit)
    (parent / f"{rover.PACKAGE}_export.md").write_text(rover._report(rover_plan, rover_label), encoding="utf-8", newline="\n")
    report = parent / "arm_export.md"
    report.write_text(_report(plan, label, mount), encoding="utf-8", newline="\n")
    return ArmExport(arm_path, moveit_path, report, plan, label, rover_path)


__all__ += ["ArmExport", "Axis", "FlatLink", "parse_axes", "find_arm_wiring", "mount_pose"]

"""Generate billee_arm_moveit_config, a self-contained MoveIt 2 package for the arm.

The arm stays modular. In an assembly the package plans with the full rover in the collision
model and adds its controllers to the running rover stack, without changing that stack.
Standalone, it brings its own robot model and mock hardware.
"""

from __future__ import annotations

import json
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field, replace

from ..core.assembly import FlatJoint, FlatLink, FlatRobot
from ..core.model import Actuation, ControlRole, DriveKind, FrameKind, JointType, ModuleKind, Project
from ..core.transforms import Transform
from ..core.validate import Issue, Severity
from .urdf import fmt, robot_to_urdf

PACKAGE = "billee_arm_moveit_config"
CONTROLLER_TYPE = "joint_trajectory_controller/JointTrajectoryController"
DEFAULT_ACCELERATION_FACTOR = 2.0


@dataclass
class ArmPlan:
    arm_joints: list[str] = field(default_factory=list)
    gripper_joints: list[str] = field(default_factory=list)
    base_link: str = ""
    tip_link: str = ""
    controllers: dict[str, list[str]] = field(default_factory=dict)
    poses: dict[str, dict[str, float]] = field(default_factory=dict)
    issues: list[Issue] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not any(i.severity is Severity.ERROR for i in self.issues)

    @property
    def all_joints(self) -> list[str]:
        return [*self.arm_joints, *self.gripper_joints]

    def error(self, code: str, entity: str, message: str, action: str) -> None:
        self.issues.append(Issue(Severity.ERROR, code, entity, message, action))

    def warn(self, code: str, entity: str, message: str, action: str) -> None:
        self.issues.append(Issue(Severity.WARNING, code, entity, message, action))


def analyze_arm(project: Project, robot: FlatRobot) -> ArmPlan:
    """Find the planning chain, end effector, controllers, and named poses."""
    plan = ArmPlan()
    modules = [m for m in project.modules if m.kind is ModuleKind.ARM]
    if len(modules) != 1:
        plan.error("arm_module", project.name, f"Expected one arm module, found {len(modules)}.",
                   "Create one arm module in the Arm tab.")
        return plan
    module = modules[0]
    tools = [f for f in module.frames if f.kind is FrameKind.TOOL]
    if len(tools) != 1:
        plan.error("arm_tool", module.id, "The arm needs exactly one tool frame at its tip.",
                   "Add a frame of type Tool in the Arm tab.")
        return plan
    plan.tip_link = f"{module.prefix}{tools[0].name}"

    by_child = {j.child: j for j in robot.joints}
    chain: list[FlatJoint] = []
    current = plan.tip_link
    while current in by_child:
        joint = by_child[current]
        if joint.role is ControlRole.ARM and joint.type is not JointType.FIXED:
            chain.append(joint)
        current = joint.parent
    chain.reverse()
    declared = {j.name for j in robot.joints if j.role is ControlRole.ARM and j.type is not JointType.FIXED}
    outside = sorted(declared - {j.name for j in chain})
    for name in outside:
        plan.error("arm_chain", robot.joint(name).source_id, f"{name} has the Arm role but is not between the arm base and tool frame.",
                   "Correct its role or move the tool frame.")
    if not chain:
        plan.error("arm_chain", module.id, "No Arm joints lead to the tool frame.",
                   "Set the arm's joints to Actuated and Arm / manipulation.")
        return plan
    for joint in chain:
        if joint.actuation is not Actuation.ACTUATED:
            plan.error("arm_actuation", joint.source_id, f"{joint.name} must be actuated to be planned.",
                       f"Select {joint.name} and set it to Actuated.")
    plan.arm_joints = [j.name for j in chain]
    plan.base_link = chain[0].parent
    plan.gripper_joints = sorted(
        j.name for j in robot.joints if j.role is ControlRole.GRIPPER and j.type is not JointType.FIXED)

    groups: dict[str, list[str]] = {}
    for joint in chain:
        kind = joint.drive.kind if joint.drive is not None else None
        name = {DriveKind.CAN: "arm_can_controller", DriveKind.ROBOCLAW: "arm_usb_controller"}.get(kind, "arm_controller")
        groups.setdefault(name, []).append(joint.name)
    if plan.gripper_joints:
        groups["gripper_controller"] = list(plan.gripper_joints)
    plan.controllers = groups

    names = {j.id: f"{module.prefix}{j.name}" for j in module.joints}
    for pose, values in module.poses.items():
        plan.poses[pose] = {names[k]: v for k, v in values.items() if names.get(k) in plan.arm_joints}
    plan.poses.setdefault("home", {name: 0.0 for name in plan.arm_joints})

    for joint in [robot.joint(n) for n in plan.all_joints]:
        if joint.limit is None or joint.limit.velocity is None or joint.limit.velocity <= 0.0:
            plan.error("arm_velocity", joint.source_id, f"{joint.name} needs a velocity limit above zero for planning.",
                       f"Select {joint.name} and open its joint limits.")
    return plan


def real_hardware_gaps(robot: FlatRobot, plan: ArmPlan) -> list[str]:
    """Reasons real motors cannot be enabled yet. These never block simulation."""
    gaps = []
    for name in plan.all_joints:
        joint = robot.joint(name)
        if joint.drive is None:
            gaps.append(f"{name}: no motor assigned")
        else:
            gaps.extend(f"{name}: missing {item}" for item in joint.drive.missing())
    if any(robot.joint(n).drive is not None and robot.joint(n).drive.kind is DriveKind.ROBOCLAW for n in plan.all_joints):
        gaps.append("RoboClaw joints: the pinned driver reports position only, but the Humble trajectory "
                    "controller needs a velocity state for velocity commands")
    return gaps


def _srdf(robot: FlatRobot, plan: ArmPlan, virtual_root: str | None) -> str:
    root = ET.Element("robot", {"name": robot.name})
    group = ET.SubElement(root, "group", {"name": "arm"})
    ET.SubElement(group, "chain", {"base_link": plan.base_link, "tip_link": plan.tip_link})
    if plan.gripper_joints:
        gripper = ET.SubElement(root, "group", {"name": "gripper"})
        for name in plan.gripper_joints:
            ET.SubElement(gripper, "joint", {"name": name})
    for pose in sorted(plan.poses):
        state = ET.SubElement(root, "group_state", {"name": pose, "group": "arm"})
        for name in plan.arm_joints:
            ET.SubElement(state, "joint", {"name": name, "value": fmt(plan.poses[pose].get(name, 0.0))})
    if plan.gripper_joints:
        for pose, pick in (("open", "upper"), ("closed", "lower")):
            state = ET.SubElement(root, "group_state", {"name": pose, "group": "gripper"})
            for name in plan.gripper_joints:
                ET.SubElement(state, "joint", {"name": name, "value": fmt(getattr(robot.joint(name).limit, pick))})
        parent = robot.joint(plan.gripper_joints[0]).parent
        ET.SubElement(root, "end_effector", {"name": "gripper", "parent_link": parent, "group": "gripper",
                                             "parent_group": "arm"})
    if virtual_root:
        ET.SubElement(root, "virtual_joint", {"name": "world_joint", "type": "fixed", "parent_frame": "world",
                                              "child_link": virtual_root})
    colliding = {l.name for l in robot.links if l.collisions}
    pairs = sorted({tuple(sorted((j.parent, j.child))) for j in robot.joints
                    if j.parent in colliding and j.child in colliding})
    for first, second in pairs:
        ET.SubElement(root, "disable_collisions", {"link1": first, "link2": second, "reason": "Adjacent"})
    ET.indent(root, space="  ")
    return '<?xml version="1.0"?>\n' + ET.tostring(root, encoding="unicode") + "\n"


def _kinematics(plan: ArmPlan) -> str:
    position_only = "true" if len(plan.arm_joints) < 6 else "false"
    note = ("# With fewer than six axes the arm cannot reach every position and orientation, so IK\n"
            "# solves for tool position only.\n" if len(plan.arm_joints) < 6 else "")
    return (f"{note}arm:\n  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin\n"
            "  kinematics_solver_search_resolution: 0.005\n  kinematics_solver_timeout: 0.05\n"
            f"  position_only_ik: {position_only}\n")


def _joint_limits(robot: FlatRobot, plan: ArmPlan) -> str:
    lines = ["# Acceleration limits are planning defaults. Replace them with measured values.",
             "default_velocity_scaling_factor: 0.3", "default_acceleration_scaling_factor: 0.3", "joint_limits:"]
    for name in plan.all_joints:
        velocity = robot.joint(name).limit.velocity
        # MoveIt rejects YAML integers here, so these must always be written as floats.
        lines += [f"  {name}:", "    has_velocity_limits: true", f"    max_velocity: {float(fmt(velocity))!r}",
                  "    has_acceleration_limits: true",
                  f"    max_acceleration: {float(fmt(velocity * DEFAULT_ACCELERATION_FACTOR))!r}"]
    return "\n".join(lines) + "\n"


def _moveit_controllers(plan: ArmPlan) -> str:
    lines = ["moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager",
             "moveit_simple_controller_manager:", f"  controller_names: {json.dumps(list(plan.controllers))}"]
    for name, joints in plan.controllers.items():
        lines += [f"  {name}:", "    type: FollowJointTrajectory", "    action_ns: follow_joint_trajectory",
                  "    default: true", f"    joints: {json.dumps(joints)}"]
    return "\n".join(lines) + "\n"


def _arm_controllers(plan: ArmPlan, use_sim_time: bool = True) -> str:
    lines = []
    for name, joints in plan.controllers.items():
        lines += [f"{name}:", "  ros__parameters:", f"    use_sim_time: {str(use_sim_time).lower()}",
                  f"    joints: {json.dumps(joints)}",
                  '    command_interfaces: ["position"]', '    state_interfaces: ["position", "velocity"]',
                  "    allow_partial_joints_goal: false", "    state_publish_rate: 50.0", "    action_monitor_rate: 20.0",
                  "    constraints:", "      stopped_velocity_tolerance: 0.05", "      goal_time: 2.0"]
        for joint in joints:
            lines += [f"      {joint}:", "        trajectory: 0.0", "        goal: 0.06"]
    return "\n".join(lines) + "\n"


def control_block(name: str, plugin: str, joints: list[str]) -> str:
    entries = "".join(
        f'    <joint name="{joint}">\n      <command_interface name="position"/>\n'
        '      <state_interface name="position"/>\n      <state_interface name="velocity"/>\n    </joint>\n'
        for joint in joints
    )
    return (f'  <ros2_control name="{name}" type="system">\n    <hardware>\n      <plugin>{plugin}</plugin>\n'
            f"    </hardware>\n{entries}  </ros2_control>\n")


def simulation_block(plan: ArmPlan) -> str:
    """Added to the rover model, so Gazebo drives the arm. It is inactive on real hardware."""
    return ('  <xacro:if value="${use_sim}">\n'
            + control_block("BilleeArmSim", "ign_ros2_control/IgnitionSystem", plan.all_joints)
            + "  </xacro:if>\n")


_LAUNCH = '''"""Start MoveIt for the BILLEE arm.

Assembly: start the rover first, then run this launch. It adds the arm controllers to the rover's
controller manager and starts move_group with the full rover in the collision model.
Standalone: this launch also starts the arm's own model and mock hardware.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

PACKAGE = "@PACKAGE@"
CONTROLLERS = @CONTROLLERS@
STANDALONE = @STANDALONE@


def _setup(context):
    share = get_package_share_directory(PACKAGE)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() in ("true", "1")
    if STANDALONE:
        model = os.path.join(share, "urdf", "arm.urdf")
        mappings = None
    else:
        model = os.path.join(get_package_share_directory("robot_description"), "urdf", "robot.urdf.xacro")
        mappings = {"use_sim": "true" if use_sim_time else "false"}
    config = (
        MoveItConfigsBuilder("@ROBOT@", package_name=PACKAGE)
        .robot_description(file_path=model, mappings=mappings)
        .robot_description_semantic(file_path="config/robot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=False, publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    controllers_file = os.path.join(share, "config", "arm_controllers.yaml")
    actions = []
    if STANDALONE:
        actions += [
            Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
                 parameters=[config.robot_description]),
            Node(package="controller_manager", executable="ros2_control_node", output="screen",
                 parameters=[config.robot_description, os.path.join(share, "config", "standalone_manager.yaml")]),
            Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"]),
        ]
    for name in CONTROLLERS:
        actions.append(Node(
            package="controller_manager", executable="spawner", output="screen",
            arguments=[name, "--controller-type", "@CONTROLLER_TYPE@", "--param-file", controllers_file],
        ))
    actions.append(Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        parameters=[config.to_dict(), {"use_sim_time": use_sim_time}],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="@SIM_DEFAULT@"),
        OpaqueFunction(function=_setup),
    ])
'''


def _package_xml(standalone: bool) -> str:
    depends = ["moveit_ros_move_group", "moveit_kinematics", "moveit_planners_ompl", "moveit_simple_controller_manager",
               "moveit_configs_utils", "controller_manager", "joint_state_broadcaster", "joint_trajectory_controller"]
    if standalone:
        depends += ["robot_state_publisher"]
    else:
        depends += ["robot_description"]
    lines = "\n".join(f"  <exec_depend>{d}</exec_depend>" for d in depends)
    return f"""<?xml version="1.0"?>
<package format="3">
  <name>{PACKAGE}</name>
  <version>0.1.0</version>
  <description>MoveIt 2 configuration for the BILLEE arm, generated by the BILLEE Creation Toolkit.</description>
  <maintainer email="maintainer@example.invalid">BILLEE team</maintainer>
  <license>GPL-3.0-or-later</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
{lines}
  <export><build_type>ament_cmake</build_type></export>
</package>
"""


def _cmake(standalone: bool) -> str:
    folders = "config launch urdf" if standalone else "config launch"
    return f"""cmake_minimum_required(VERSION 3.16)
project({PACKAGE} NONE)
find_package(ament_cmake REQUIRED)
install(DIRECTORY {folders} DESTINATION share/${{PROJECT_NAME}})
ament_package()
"""


def _standalone_robot(robot: FlatRobot) -> FlatRobot:
    """Give the arm a massless world link, since KDL ignores inertia on a root link."""
    root = robot.root
    output = FlatRobot(robot.name, [FlatLink("world", robot.link(root).source_id), *robot.links], list(robot.joints))
    output.joints.append(FlatJoint("world_to_arm", robot.link(root).source_id, JointType.FIXED, "world", root, Transform()))
    return output


def build_moveit_files(robot: FlatRobot, plan: ArmPlan, standalone: bool = False) -> dict[str, bytes]:
    """Return the package as {path: bytes}. `robot` is the exact model MoveIt will load."""
    model = _standalone_robot(robot) if standalone else robot
    controllers = _arm_controllers(plan, use_sim_time=not standalone)
    text = {
        "package.xml": _package_xml(standalone),
        "CMakeLists.txt": _cmake(standalone),
        "config/robot.srdf": _srdf(model, plan, None),
        "config/kinematics.yaml": _kinematics(plan),
        "config/joint_limits.yaml": _joint_limits(model, plan),
        "config/moveit_controllers.yaml": _moveit_controllers(plan),
        "config/arm_controllers.yaml": controllers,
    }
    if standalone:
        urdf = robot_to_urdf(model, PACKAGE)
        block = control_block("BilleeArmMock", "mock_components/GenericSystem", plan.all_joints)
        text["urdf/arm.urdf"] = urdf.replace("</robot>", block + "</robot>")
        text["config/standalone_manager.yaml"] = (
            "controller_manager:\n  ros__parameters:\n    update_rate: 100\n    joint_state_broadcaster:\n"
            "      type: joint_state_broadcaster/JointStateBroadcaster\n")
    launch = (_LAUNCH.replace("@PACKAGE@", PACKAGE).replace("@CONTROLLERS@", repr(list(plan.controllers)))
              .replace("@STANDALONE@", repr(standalone)).replace("@ROBOT@", model.name)
              .replace("@CONTROLLER_TYPE@", CONTROLLER_TYPE).replace("@SIM_DEFAULT@", "false" if standalone else "true"))
    text["launch/moveit.launch.py"] = launch
    return {path: content.encode("utf-8") for path, content in sorted(text.items())}


__all__ = ["ArmPlan", "PACKAGE", "analyze_arm", "build_moveit_files", "real_hardware_gaps", "simulation_block", "replace"]

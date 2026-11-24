"""Acceptance test: MoveIt plans and executes through the team's real /arm controllers.

Usage: arm_moveit_check.py refused | assembly | standalone
"""

import math
import re
import subprocess
import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose, PoseStamped, Twist
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import BoundingVolume, Constraints, JointConstraint, MoveItErrorCodes, PositionConstraint, RobotState
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetStateValidity
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformListener

ARM = [f"joint_arm_{n}" for n in range(1, 6)]
REACHABLE = dict(zip(ARM, (0.25, -0.2, 0.45, 0.3, 0.4)))
SECOND = dict(zip(ARM, (-0.25, 0.15, 0.35, -0.3, -0.3)))
FAR = dict(zip(ARM, (0.3, -0.2, 0.4, 2.2, 1.2)))
HOME = dict(zip(ARM, [0.0] * 5))
INTO_CHASSIS = dict(zip(ARM, (0.0, 1.57, 1.57, 0.0, 0.0)))
ARM_CONTROLLERS = {"joint_state_broadcaster", "arm_can_controller", "arm_roboclaw_controller"}
RESULTS = []


def report(name, passed, detail=""):
    RESULTS.append(bool(passed))
    print(f"CHECK_{'PASS' if passed else 'FAIL'} {name}{': ' + detail if detail else ''}", flush=True)
    return bool(passed)


class Client:
    def __init__(self):
        self.node = rclpy.create_node("arm_moveit_check")
        self.position, self.odom, self.velocity = {}, None, None
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(JointState, "/arm/joint_states", self.on_state, qos)
        self.node.create_subscription(Odometry, "/gz/odom", self.on_odom, qos)
        self.node.create_subscription(Float64MultiArray, "/arm/arm_roboclaw_controller/commands", self.on_velocity, 10)
        self.move = ActionClient(self.node, MoveGroup, "/move_action")
        self.bridge = ActionClient(self.node, FollowJointTrajectory, "/billee_arm_controller/follow_joint_trajectory")
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.validity = self.node.create_client(GetStateValidity, "/check_state_validity")
        self.drive = self.node.create_publisher(Twist, "/diff_drive_controller/cmd_vel_unstamped", 10)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self.node)

    def on_state(self, message):
        self.position.update(zip(message.name, message.position))

    def on_odom(self, message):
        self.odom = (message.pose.pose.position.x, message.pose.pose.position.y)

    def on_velocity(self, message):
        self.velocity = list(message.data)

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def wait(self, future, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.02)
        return future.result() if future.done() else None

    def goal(self, group, constraints, plan_only):
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = group
        request.num_planning_attempts = 5
        request.allowed_planning_time = 6.0
        request.max_velocity_scaling_factor = 0.8
        request.max_acceleration_scaling_factor = 0.8
        request.start_state.is_diff = True
        request.goal_constraints = [constraints]
        goal.planning_options.plan_only = plan_only
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        return goal

    def run(self, group, constraints, plan_only=False, seconds=60):
        handle = self.wait(self.move.send_goal_async(self.goal(group, constraints, plan_only)), 15)
        if handle is None or not handle.accepted:
            return None
        wrapped = self.wait(handle.get_result_async(), seconds)
        return None if wrapped is None else wrapped.result

    def joints(self, values):
        constraints = Constraints()
        for name, value in values.items():
            constraints.joint_constraints.append(JointConstraint(
                joint_name=name, position=value, tolerance_above=0.01, tolerance_below=0.01, weight=1.0))
        return constraints

    def state(self, values):
        state = RobotState(is_diff=True)
        state.joint_state.name = list(values)
        state.joint_state.position = [float(v) for v in values.values()]
        return state

    def reached(self, values, tolerance=0.06):
        return all(abs(self.position.get(n, 99.0) - v) < tolerance for n, v in values.items())


def code(result):
    return "no result" if result is None else str(result.error_code.val)


def ok(result):
    return result is not None and result.error_code.val == MoveItErrorCodes.SUCCESS


def controllers(manager):
    try:
        raw = subprocess.run(["ros2", "control", "list_controllers", "-c", manager],
                             capture_output=True, text=True, timeout=20).stdout
    except subprocess.TimeoutExpired:
        return set()
    clean = re.sub(r"\x1b\[[0-9;]*m", "", raw)
    return {l.split()[0] for l in clean.splitlines() if re.search(r"\bactive\b", l)}


def wait_ready(client, require_rover):
    end = time.monotonic() + 70
    arm, base = set(), set()
    wanted = {"diff_drive_controller", "joint_state_broadcaster"} if require_rover else set()
    while time.monotonic() < end and not (ARM_CONTROLLERS <= arm and wanted <= base):
        client.spin(1.5)
        arm = controllers("/arm/controller_manager")
        base = controllers("/controller_manager") if require_rover else set()
    if not report("team arm controllers active under /arm", ARM_CONTROLLERS <= arm, ",".join(sorted(arm))):
        return False
    if require_rover and not report("rover controllers active", wanted <= base, ",".join(sorted(base))):
        return False
    return report("move_group is running", client.move.wait_for_server(timeout_sec=45.0)) and report(
        "arm joints publish on /arm/joint_states", wait_joints(client))


def wait_joints(client):
    end = time.monotonic() + 15
    while time.monotonic() < end and not set(ARM + ["end_effector"]) <= set(client.position):
        client.spin(0.2)
    return set(ARM + ["end_effector"]) <= set(client.position)


def refused(client):
    if not wait_ready(client, True):
        return
    client.spin(2)
    before = dict(client.position)
    planned = client.run("arm", client.joints(REACHABLE), plan_only=True)
    report("plans while the axes are unverified", ok(planned), f"code {code(planned)}")
    result = client.run("arm", client.joints(REACHABLE))
    client.spin(1.5)
    report("refuses to move axes not verified in arm_axes.yaml", result is not None and not ok(result), f"code {code(result)}")
    report("arm did not move", all(abs(client.position[n] - before[n]) < 0.01 for n in ARM))


def assembly(client):
    if not wait_ready(client, True):
        return
    client.spin(4)
    joined = client.tf.can_transform("base_link", "arm_tool_link", Time())
    report("one TF tree joins the rover and arm tool", joined)
    frames = client.tf.all_frames_as_yaml()
    report("the arm appears once in TF", "arm_base_link" in frames and "\narm_base:" not in frames)

    for label, target in (("plans and executes across CAN and RoboClaw axes", REACHABLE),
                          ("executes a second joint goal", SECOND)):
        result = client.run("arm", client.joints(target))
        client.spin(1.0)
        report(label, ok(result) and client.reached(target), f"code {code(result)}, "
               + str({k[-1]: round(client.position[k], 3) for k in ARM}))
    report("RoboClaw velocity is zero after motion", client.velocity is not None and all(v == 0.0 for v in client.velocity))

    result = client.run("gripper", client.joints({"end_effector": 0.5}))
    client.spin(0.8)
    report("end effector moves through its own group", ok(result) and client.reached({"end_effector": 0.5}),
           f"code {code(result)}, position {client.position['end_effector']:.3f}")

    request = GetPositionFK.Request()
    request.header.frame_id = "base_footprint"
    request.fk_link_names = ["arm_tool_link"]
    request.robot_state = client.state(REACHABLE)
    fk = client.wait(client.fk.call_async(request), 10)
    if report("forward kinematics", fk is not None and fk.error_code.val == MoveItErrorCodes.SUCCESS):
        pose = fk.pose_stamped[0].pose
        ik_request = GetPositionIK.Request()
        ik_request.ik_request.group_name = "arm"
        ik_request.ik_request.ik_link_name = "arm_tool_link"
        ik_request.ik_request.avoid_collisions = True
        ik_request.ik_request.timeout.sec = 2
        ik_request.ik_request.pose_stamped = PoseStamped(pose=pose)
        ik_request.ik_request.pose_stamped.header.frame_id = "base_footprint"
        ik = client.wait(client.ik.call_async(ik_request), 12)
        report("position IK solves a reachable tool position", ik is not None and ik.error_code.val == MoveItErrorCodes.SUCCESS)
        position = PositionConstraint(link_name="arm_tool_link", weight=1.0)
        position.header.frame_id = "base_footprint"
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.015])
        position.constraint_region = BoundingVolume(primitives=[sphere], primitive_poses=[Pose(position=Point(
            x=pose.position.x, y=pose.position.y, z=pose.position.z))])
        result = client.run("arm", Constraints(position_constraints=[position]), plan_only=True)
        report("plans to a Cartesian tool position", ok(result), f"code {code(result)}")

    request = GetStateValidity.Request(group_name="arm")
    request.robot_state = client.state(INTO_CHASSIS)
    validity = client.wait(client.validity.call_async(request), 10)
    pairs = [] if validity is None else sorted({f"{c.contact_body_1}/{c.contact_body_2}" for c in validity.contacts})
    report("detects the arm colliding with the rover chassis",
           validity is not None and not validity.valid and any("base_link" in p and "arm_link" in p for p in pairs),
           ", ".join(pairs[:3]))
    before = dict(client.position)
    result = client.run("arm", client.joints(INTO_CHASSIS))
    client.spin(0.8)
    report("refuses to move into the chassis", result is not None and not ok(result), f"code {code(result)}")
    report("arm did not move for the refused goal", all(abs(client.position[n] - before[n]) < 0.03 for n in ARM))

    handle = client.wait(client.move.send_goal_async(client.goal("arm", client.joints(FAR), False)), 15)
    moving = handle is not None and handle.accepted
    if moving:
        end = time.monotonic() + 12
        while time.monotonic() < end and not (client.velocity and any(abs(v) > 0.02 for v in client.velocity)):
            client.spin(0.05)
        moving = bool(client.velocity and any(abs(v) > 0.02 for v in client.velocity))
    if report("RoboClaw axes receive velocity during motion", moving) and handle is not None:
        client.wait(handle.cancel_goal_async(), 5)
        client.spin(1.0)
        first = dict(client.position)
        client.spin(1.0)
        report("cancel stops the RoboClaw axes", all(v == 0.0 for v in client.velocity)
               and all(abs(client.position[n] - first[n]) < 0.01 for n in ARM), str(client.velocity))
        client.wait(handle.get_result_async(), 10)

    result = client.run("arm", client.joints(HOME))
    client.spin(0.8)
    report("returns to home", ok(result) and client.reached(HOME), f"code {code(result)}")

    start = client.odom
    twist = Twist()
    twist.linear.x = 0.3
    end = time.monotonic() + 5
    while time.monotonic() < end:
        client.drive.publish(twist)
        client.spin(0.1)
    client.drive.publish(Twist())
    client.spin(0.5)
    moved = 0.0 if start is None or client.odom is None else math.dist(start, client.odom)
    report("rover still drives with MoveIt running", moved > 0.4, f"{moved:.3f} m")


def standalone(client):
    if not wait_ready(client, False):
        return
    client.spin(2)
    result = client.run("arm", client.joints(REACHABLE))
    client.spin(1.0)
    report("arm alone plans and executes", ok(result) and client.reached(REACHABLE), f"code {code(result)}")
    result = client.run("gripper", client.joints({"end_effector": -0.4}))
    client.spin(0.8)
    report("arm alone moves its end effector", ok(result) and client.reached({"end_effector": -0.4}), f"code {code(result)}")


def main():
    rclpy.init()
    client = Client()
    try:
        {"refused": refused, "assembly": assembly, "standalone": standalone}[sys.argv[1]](client)
    finally:
        client.node.destroy_node()
        rclpy.shutdown()
    return 0 if RESULTS and all(RESULTS) else 1


if __name__ == "__main__":
    sys.exit(main())

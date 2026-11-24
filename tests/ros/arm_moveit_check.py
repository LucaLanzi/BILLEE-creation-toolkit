"""Acceptance test: MoveIt plans and executes the arm on the running BILLEE rover stack."""

import math
import re
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, Twist
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetStateValidity
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

ARM = [f"joint_arm_{n}" for n in range(1, 6)]
GRIPPER = ["joint_gripper"]
REACHABLE = dict(zip(ARM, (0.5, -0.4, 0.9, 0.3, 0.5)))
SECOND = dict(zip(ARM, (-0.6, 0.3, 0.7, -0.4, -0.3)))
INTO_CHASSIS = dict(zip(ARM, (0.0, 1.57, 1.57, 0.0, 0.0)))
REQUIRED_CONTROLLERS = {"diff_drive_controller", "joint_state_broadcaster", "arm_can_controller",
                        "arm_usb_controller", "gripper_controller"}
results = []


def report(name, passed, detail=""):
    results.append(bool(passed))
    print(f"MOVEIT_{'PASS' if passed else 'FAIL'} {name}{': ' + detail if detail else ''}", flush=True)
    return passed


class Client:
    def __init__(self, sim_time=True):
        self.node = rclpy.create_node("arm_moveit_check", parameter_overrides=[Parameter("use_sim_time", value=sim_time)])
        self.position, self.odom = {}, None
        self.node.create_subscription(JointState, "/joint_states", self.on_state, qos_profile_sensor_data)
        self.node.create_subscription(Odometry, "/gz/odom", self.on_odom,
                                      QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.move = ActionClient(self.node, MoveGroup, "/move_action")
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.validity = self.node.create_client(GetStateValidity, "/check_state_validity")
        self.drive = self.node.create_publisher(Twist, "/diff_drive_controller/cmd_vel_unstamped", 10)

    def on_state(self, message):
        self.position.update(zip(message.name, message.position))

    def on_odom(self, message):
        self.odom = (message.pose.pose.position.x, message.pose.pose.position.y)

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait(self, future, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return future.result() if future.done() else None

    def request(self, group, constraints, plan_only):
        goal = MoveGroup.Goal()
        goal.request.group_name = group
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 6.0
        goal.request.max_velocity_scaling_factor = 0.6
        goal.request.max_acceleration_scaling_factor = 0.6
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = plan_only
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        handle = self.wait(self.move.send_goal_async(goal), 15)
        if handle is None or not handle.accepted:
            return None
        wrapped = self.wait(handle.get_result_async(), 45)
        return None if wrapped is None else wrapped.result

    def joint_goal(self, group, values, plan_only=False):
        constraints = Constraints()
        for name, value in values.items():
            constraints.joint_constraints.append(JointConstraint(
                joint_name=name, position=value, tolerance_above=0.01, tolerance_below=0.01, weight=1.0))
        return self.request(group, constraints, plan_only)

    def state(self, values):
        state = RobotState()
        state.is_diff = True
        state.joint_state.name = list(values)
        state.joint_state.position = [float(v) for v in values.values()]
        return state

    def reached(self, values, tolerance=0.08):
        return all(abs(self.position.get(n, 99.0) - v) < tolerance for n, v in values.items())


def code(result):
    return "no result" if result is None else str(result.error_code.val)


def succeeded(result):
    return result is not None and result.error_code.val == MoveItErrorCodes.SUCCESS


def active_controllers():
    try:
        raw = subprocess.run(["ros2", "control", "list_controllers"], capture_output=True, text=True, timeout=20).stdout
    except subprocess.TimeoutExpired:
        return set()
    clean = re.sub(r"\x1b\[[0-9;]*m", "", raw)
    return {l.split()[0] for l in clean.splitlines() if re.search(r"\bactive\b", l)}


def run(client):
    end = time.monotonic() + 75
    active = set()
    while time.monotonic() < end and not REQUIRED_CONTROLLERS <= active:
        client.spin(2)
        active = active_controllers()
    if not report("rover and arm controllers active together", REQUIRED_CONTROLLERS <= active, ",".join(sorted(active))):
        return
    ready = client.move.wait_for_server(timeout_sec=40.0)
    services = all(s.wait_for_service(timeout_sec=20.0) for s in (client.ik, client.fk, client.validity))
    if not report("move_group is running", ready and services):
        return
    if not report("all arm joints publish state", client.position and set(ARM + GRIPPER) <= set(client.position)):
        return
    client.spin(2)

    result = client.joint_goal("arm", REACHABLE)
    client.spin(1.5)
    report("plans and executes a joint goal in Gazebo", succeeded(result) and client.reached(REACHABLE),
           f"code {code(result)}, " + str({k[-1]: round(client.position[k], 3) for k in ARM}))
    result = client.joint_goal("arm", SECOND)
    client.spin(1.5)
    report("executes across CAN and USB controllers", succeeded(result) and client.reached(SECOND), f"code {code(result)}")

    result = client.joint_goal("gripper", {"joint_gripper": 0.5})
    client.spin(1.0)
    report("end effector moves through its own group", succeeded(result) and client.reached({"joint_gripper": 0.5}),
           f"code {code(result)}, position {client.position.get('joint_gripper', float('nan')):.3f}")

    request = GetPositionFK.Request()
    request.header.frame_id = "base_footprint"
    request.fk_link_names = ["arm_tool"]
    request.robot_state = client.state(REACHABLE)
    fk = client.wait(client.fk.call_async(request), 10)
    if report("forward kinematics", fk is not None and fk.error_code.val == MoveItErrorCodes.SUCCESS):
        pose: Pose = fk.pose_stamped[0].pose
        ik_request = GetPositionIK.Request()
        ik_request.ik_request.group_name = "arm"
        ik_request.ik_request.ik_link_name = "arm_tool"
        ik_request.ik_request.avoid_collisions = True
        ik_request.ik_request.timeout.sec = 2
        ik_request.ik_request.pose_stamped = PoseStamped(pose=pose)
        ik_request.ik_request.pose_stamped.header.frame_id = "base_footprint"
        ik = client.wait(client.ik.call_async(ik_request), 12)
        report("position IK solves a reachable tool position", ik is not None and ik.error_code.val == MoveItErrorCodes.SUCCESS,
               "" if ik is None else f"code {ik.error_code.val}")

        constraints = Constraints()
        position = PositionConstraint()
        position.header.frame_id = "base_footprint"
        position.link_name = "arm_tool"
        position.weight = 1.0
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.015])
        position.constraint_region = BoundingVolume(primitives=[sphere], primitive_poses=[Pose(position=Point(
            x=pose.position.x, y=pose.position.y, z=pose.position.z))])
        constraints.position_constraints = [position]
        result = client.request("arm", constraints, plan_only=True)
        report("plans to a Cartesian tool position", succeeded(result), f"code {code(result)}")

    request = GetStateValidity.Request()
    request.group_name = "arm"
    request.robot_state = client.state(INTO_CHASSIS)
    validity = client.wait(client.validity.call_async(request), 10)
    touching = [] if validity is None else sorted({f"{c.contact_body_1}/{c.contact_body_2}" for c in validity.contacts})
    hit_rover = any("base_link" in pair for pair in touching)
    report("detects the arm colliding with the chassis", validity is not None and not validity.valid and hit_rover,
           ", ".join(touching[:3]))
    before = dict(client.position)
    result = client.joint_goal("arm", INTO_CHASSIS, plan_only=True)
    report("refuses to plan into the chassis", result is not None and not succeeded(result), f"code {code(result)}")
    client.spin(1.0)
    report("arm did not move for the refused goal",
           all(abs(client.position[n] - before[n]) < 0.03 for n in ARM))

    result = client.joint_goal("arm", dict(zip(ARM, [0.0] * 5)))
    client.spin(1.0)
    report("returns to home", succeeded(result) and client.reached(dict(zip(ARM, [0.0] * 5))), f"code {code(result)}")

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


def run_standalone(client):
    """The arm alone: its own model and mock hardware, with no rover or Gazebo."""
    required = {"joint_state_broadcaster", "arm_can_controller", "arm_usb_controller", "gripper_controller"}
    end = time.monotonic() + 50
    active = set()
    while time.monotonic() < end and not required <= active:
        client.spin(2)
        active = active_controllers()
    if not report("standalone arm controllers active", required <= active, ",".join(sorted(active))):
        return
    if not report("standalone move_group is running", client.move.wait_for_server(timeout_sec=30.0)):
        return
    client.spin(2)
    result = client.joint_goal("arm", REACHABLE)
    client.spin(1.0)
    report("standalone arm plans and executes", succeeded(result) and client.reached(REACHABLE, 0.03), f"code {code(result)}")
    result = client.joint_goal("gripper", {"joint_gripper": -0.4})
    client.spin(0.5)
    report("standalone end effector moves", succeeded(result) and client.reached({"joint_gripper": -0.4}, 0.03),
           f"code {code(result)}")


def main():
    rclpy.init()
    standalone = "--standalone" in sys.argv
    client = Client(sim_time=not standalone)
    try:
        if standalone:
            run_standalone(client)
        else:
            run(client)
    finally:
        client.node.destroy_node()
        rclpy.shutdown()
    return 0 if results and all(results) else 1


if __name__ == "__main__":
    sys.exit(main())

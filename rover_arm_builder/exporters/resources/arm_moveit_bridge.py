#!/usr/bin/env python3
"""Connect MoveIt to the BILLEE arm's existing controllers.

MoveIt sends one FollowJointTrajectory goal per planning group. This node:
  - forwards the P17/P20 axes to /arm/arm_can_controller (a trajectory controller);
  - tracks the RoboClaw axes itself and streams /arm/arm_roboclaw_controller/commands,
    because that controller only takes velocities and holds its last command;
  - merges /joint_states (rover) and /arm/joint_states (arm) for move_group.

Safety:
  - Goals are refused while any axis they use is `verified: false` in arm_axes.yaml, unless
    allow_unverified is true. Use that only with mock hardware or during a supervised bench test.
  - Velocity commands are clamped to each axis's max_velocity.
  - Motion stops on cancel, lost arm feedback, excess tracking error, or shutdown.
  - Velocity commands are only published during a goal, so the jog teleop is not overridden when idle.
  - A hard kill cannot send a stop, and the RoboClaw controller would hold its last command.
    Keep the E-stop within reach on real hardware.
"""

import bisect
import signal
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint

Result = FollowJointTrajectory.Result


class Track:
    """Position and velocity targets over time for a subset of a trajectory's joints."""

    def __init__(self, trajectory, joints):
        index = {name: trajectory.joint_names.index(name) for name in joints}
        self.joints = list(joints)
        self.times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in trajectory.points]
        self.positions = {n: [p.positions[i] for p in trajectory.points] for n, i in index.items()}
        has_velocity = all(len(p.velocities) == len(trajectory.joint_names) for p in trajectory.points)
        self.velocities = None
        if has_velocity:
            self.velocities = {n: [p.velocities[i] for p in trajectory.points] for n, i in index.items()}

    @property
    def duration(self):
        return self.times[-1]

    def sample(self, name, t):
        positions = self.positions[name]
        if t <= self.times[0]:
            return positions[0], 0.0
        if t >= self.times[-1]:
            return positions[-1], 0.0
        upper = bisect.bisect_right(self.times, t)
        lower = upper - 1
        span = self.times[upper] - self.times[lower]
        ratio = (t - self.times[lower]) / span if span > 0.0 else 1.0
        position = positions[lower] + ratio * (positions[upper] - positions[lower])
        if self.velocities is None:
            velocity = (positions[upper] - positions[lower]) / span if span > 0.0 else 0.0
        else:
            values = self.velocities[name]
            velocity = values[lower] + ratio * (values[upper] - values[lower])
        return position, velocity


class ArmMoveitBridge(Node):
    def __init__(self):
        super().__init__("billee_arm_bridge")
        self.declare_parameter("axes_file", "")
        self.declare_parameter("arm_joints", [""])
        self.declare_parameter("gripper_joints", [""])
        self.declare_parameter("allow_unverified", False)
        self.declare_parameter("arm_namespace", "/arm")
        self.declare_parameter("tracking_gain", 4.0)
        self.declare_parameter("path_tolerance", 0.35)
        self.declare_parameter("goal_tolerance", 0.04)
        self.declare_parameter("settle_time", 2.0)
        self.declare_parameter("feedback_timeout", 0.3)

        value = self.get_parameter
        self.allow_unverified = value("allow_unverified").value
        self.gain = value("tracking_gain").value
        self.path_tolerance = value("path_tolerance").value
        self.goal_tolerance = value("goal_tolerance").value
        self.settle_time = value("settle_time").value
        self.feedback_timeout = value("feedback_timeout").value
        namespace = value("arm_namespace").value.rstrip("/")
        path = value("axes_file").value or (get_package_share_directory("arm_bringup") + "/config/arm_axes.yaml")
        with open(path, encoding="utf-8") as handle:
            axes = yaml.safe_load(handle)["axes"]
        motors = [a for a in axes if a.get("output", "motor") != "duty"]
        self.axes = {a["joint"]: a for a in motors}
        self.can_joints = [a["joint"] for a in motors if a["driver"] == "pxx"]
        self.usb_joints = [a["joint"] for a in motors if a["driver"] == "roboclaw"]
        self.groups = {
            "billee_arm_controller": [j for j in value("arm_joints").value if j],
            "billee_gripper_controller": [j for j in value("gripper_joints").value if j],
        }

        self.lock = threading.Lock()
        self.arm_state = {}
        self.rover_state = {}
        self.arm_stamp = 0.0
        self.tracks = {}
        self.busy = set()
        self.zero_ticks = 0
        group = ReentrantCallbackGroup()
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JointState, f"{namespace}/joint_states", self.on_arm_state, qos, callback_group=group)
        self.create_subscription(JointState, "/joint_states", self.on_rover_state, qos, callback_group=group)
        self.merged = self.create_publisher(JointState, "/billee_arm/joint_states", 10)
        self.velocity = self.create_publisher(Float64MultiArray, f"{namespace}/arm_roboclaw_controller/commands", 10)
        self.can_client = ActionClient(self, FollowJointTrajectory,
                                       f"{namespace}/arm_can_controller/follow_joint_trajectory", callback_group=group)
        self.servers = [
            ActionServer(self, FollowJointTrajectory, f"{name}/follow_joint_trajectory",
                         execute_callback=lambda handle, n=name: self.execute(handle, n),
                         goal_callback=lambda request, n=name: self.on_goal(n),
                         cancel_callback=lambda _handle: CancelResponse.ACCEPT, callback_group=group)
            for name, joints in self.groups.items() if joints
        ]
        self.create_timer(0.02, self.publish_merged, callback_group=group)
        self.create_timer(0.01, self.control, callback_group=group)
        unverified = sorted(j for j in self.groups["billee_arm_controller"] + self.groups["billee_gripper_controller"]
                            if not self.axes.get(j, {}).get("verified", False))
        if unverified and not self.allow_unverified:
            self.get_logger().warn(f"Motion is refused until these axes are verified: {', '.join(unverified)}")

    def on_arm_state(self, message):
        with self.lock:
            self.arm_state.update(zip(message.name, message.position))
            self.arm_stamp = time.monotonic()

    def on_rover_state(self, message):
        with self.lock:
            self.rover_state.update(zip(message.name, message.position))

    def fresh(self):
        return time.monotonic() - self.arm_stamp < self.feedback_timeout

    def publish_merged(self):
        with self.lock:
            if not self.fresh():
                return
            state = {**self.rover_state, **self.arm_state}
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(state)
        message.position = [float(v) for v in state.values()]
        self.merged.publish(message)

    def control(self):
        with self.lock:
            active = bool(self.tracks)
            if not active and self.zero_ticks <= 0:
                return
            commands = {name: 0.0 for name in self.usb_joints}
            if active and self.fresh():
                now = time.monotonic()
                for track, start in self.tracks.values():
                    for name in track.joints:
                        target, feedforward = track.sample(name, now - start)
                        command = feedforward + self.gain * (target - self.arm_state.get(name, target))
                        limit = float(self.axes[name]["limits"]["max_velocity"])
                        commands[name] = max(-limit, min(limit, command))
            if not active:
                self.zero_ticks -= 1
        if self.usb_joints:
            self.velocity.publish(Float64MultiArray(data=[commands[n] for n in self.usb_joints]))

    def stop_motors(self):
        with self.lock:
            self.tracks.clear()
            self.zero_ticks = 0
        if self.usb_joints:
            stop = Float64MultiArray(data=[0.0] * len(self.usb_joints))
            for _ in range(5):
                self.velocity.publish(stop)
                time.sleep(0.01)

    def on_goal(self, name):
        with self.lock:
            return GoalResponse.REJECT if name in self.busy else GoalResponse.ACCEPT

    def problem(self, trajectory, group):
        names = list(trajectory.joint_names)
        if not names or not trajectory.points:
            return "The trajectory is empty."
        if set(names) != set(self.groups[group]):
            return f"Expected joints {self.groups[group]}, got {names}."
        unverified = [n for n in names if not self.axes[n].get("verified", False)]
        if unverified and not self.allow_unverified:
            return f"Refused: not bench-verified in arm_axes.yaml: {', '.join(unverified)}."
        for index, name in enumerate(names):
            limits = self.axes[name]["limits"]
            for point in trajectory.points:
                if not limits["min_position"] <= point.positions[index] <= limits["max_position"]:
                    return f"{name} target is outside arm_axes.yaml limits."
                if point.velocities and abs(point.velocities[index]) > float(limits["max_velocity"]) * 1.05:
                    return f"{name} velocity exceeds arm_axes.yaml max_velocity."
        return None

    def finish(self, handle, code, text, cancelled=False):
        result = Result(error_code=code, error_string=text)
        if cancelled:
            handle.canceled()
        elif code == Result.SUCCESSFUL:
            handle.succeed()
        else:
            self.get_logger().error(text)
            handle.abort()
        return result

    def wait(self, future, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not future.done():
            time.sleep(0.005)
        return future.result() if future.done() else None

    def send_can(self, trajectory, joints):
        if not self.can_client.wait_for_server(timeout_sec=3.0):
            return None
        index = [trajectory.joint_names.index(n) for n in joints]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joints)
        for point in trajectory.points:
            item = JointTrajectoryPoint(time_from_start=point.time_from_start)
            item.positions = [point.positions[i] for i in index]
            if point.velocities:
                item.velocities = [point.velocities[i] for i in index]
            if point.accelerations:
                item.accelerations = [point.accelerations[i] for i in index]
            goal.trajectory.points.append(item)
        handle = self.wait(self.can_client.send_goal_async(goal), 3.0)
        return handle if handle is not None and handle.accepted else None

    def execute(self, handle, group):
        trajectory = handle.request.trajectory
        with self.lock:
            if group in self.busy:
                return self.finish(handle, Result.INVALID_GOAL, "This group is already moving.")
            self.busy.add(group)
        can_handle = None
        try:
            text = self.problem(trajectory, group)
            if text:
                return self.finish(handle, Result.INVALID_GOAL, text)
            if not self.fresh():
                return self.finish(handle, Result.INVALID_GOAL, "No current arm feedback on /arm/joint_states.")
            full = Track(trajectory, trajectory.joint_names)
            usb = [n for n in trajectory.joint_names if n in self.usb_joints]
            can = [n for n in trajectory.joint_names if n in self.can_joints]
            can_result = None
            if can:
                can_handle = self.send_can(trajectory, can)
                if can_handle is None:
                    return self.finish(handle, Result.INVALID_GOAL, "arm_can_controller did not accept the goal.")
                can_result = can_handle.get_result_async()
            start = time.monotonic()
            if usb:
                with self.lock:
                    self.tracks[group] = (Track(trajectory, usb), start)

            while rclpy.ok():
                elapsed = time.monotonic() - start
                if handle.is_cancel_requested:
                    return self.finish(handle, Result.SUCCESSFUL, "Cancelled.", cancelled=True)
                with self.lock:
                    fresh = self.fresh()
                    state = dict(self.arm_state)
                if not fresh:
                    return self.finish(handle, Result.PATH_TOLERANCE_VIOLATED, "Lost arm feedback. Motion stopped.")
                errors = {n: abs(full.sample(n, elapsed)[0] - state.get(n, 1e9)) for n in trajectory.joint_names}
                worst = max(errors, key=errors.get)
                if errors[worst] > self.path_tolerance:
                    return self.finish(handle, Result.PATH_TOLERANCE_VIOLATED,
                                       f"{worst} is {errors[worst]:.2f} rad off its path. Motion stopped.")
                if can_result is not None and can_result.done():
                    wrapped = can_result.result()
                    if wrapped.result.error_code != Result.SUCCESSFUL:
                        return self.finish(handle, wrapped.result.error_code,
                                           f"arm_can_controller failed: {wrapped.result.error_string}")
                if elapsed >= full.duration:
                    can_done = can_result is None or can_result.done()
                    if can_done and max(errors.values()) <= self.goal_tolerance:
                        return self.finish(handle, Result.SUCCESSFUL, "")
                    if elapsed >= full.duration + self.settle_time:
                        return self.finish(handle, Result.GOAL_TOLERANCE_VIOLATED,
                                           f"{worst} stopped {errors[worst]:.3f} rad from its goal.")
                time.sleep(0.02)
            return self.finish(handle, Result.PATH_TOLERANCE_VIOLATED, "Shutting down.")
        finally:
            with self.lock:
                self.tracks.pop(group, None)
                self.busy.discard(group)
                self.zero_ticks = 5
            if can_handle is not None and handle.status != 4:  # 4 = GoalStatus.STATUS_SUCCEEDED
                can_handle.cancel_goal_async()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ArmMoveitBridge()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    try:
        while not stop.wait(0.1):
            pass
    finally:
        node.stop_motors()
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

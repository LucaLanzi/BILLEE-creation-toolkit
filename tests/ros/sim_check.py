"""Check the demo in Gazebo: settled pose, arm control, and wheel drive.

Usage: sim_check.py <gazebo model name>
"""

import math
import re
import subprocess
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration as RosDuration
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM = ["arm_elbow_pitch_joint", "arm_shoulder_yaw_joint"]
ARM_GOAL = {"arm_elbow_pitch_joint": 0.6, "arm_shoulder_yaw_joint": 0.5}
WHEELS = [
    "rover_left_front_wheel_joint", "rover_left_rear_wheel_joint",
    "rover_right_front_wheel_joint", "rover_right_rear_wheel_joint",
]
PASSIVE = ["rover_front_axle_joint"]


class Check:
    def __init__(self, model):
        self.model = model
        self.node = rclpy.create_node("rab_sim_check", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.state = {}
        self.velocity = {}
        self.node.create_subscription(JointState, "/joint_states", self.on_state, qos_profile_sensor_data)
        self.arm = self.node.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.wheels = self.node.create_publisher(Float64MultiArray, "/wheel_controller/commands", 10)

    def on_state(self, msg):
        self.state.update(zip(msg.name, msg.position))
        self.velocity.update(zip(msg.name, msg.velocity))

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait_for(self, predicate, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def pose(self):
        output = subprocess.run(["ign", "model", "-m", self.model, "-p"], capture_output=True, text=True, timeout=20).stdout
        groups = re.findall(r"\[([^\]]+)\]", output.split("Pose", 1)[-1])
        numbers = [[float(v) for v in re.findall(r"-?\d+\.?\d*(?:e-?\d+)?", g)] for g in groups]
        numbers = [n for n in numbers if len(n) == 3]
        if len(numbers) < 2:
            raise RuntimeError(f"Could not read the model pose: {output[:300]}")
        return numbers[0], numbers[1]


def report(name, passed, detail):
    print(f"SIM_{'PASS' if passed else 'FAIL'} {name}: {detail}", flush=True)
    return passed


def run(check):
    results = []
    needed = ARM + WHEELS + PASSIVE
    ok = check.wait_for(lambda: all(n in check.state for n in needed), 90)
    results.append(report("all joints publish state", ok, f"{len(check.state)} joints"))
    if not ok:
        return results

    check.spin(8)
    first_xyz, first_rpy = check.pose()
    check.spin(3)
    second_xyz, second_rpy = check.pose()
    finite = all(math.isfinite(v) for v in [*second_xyz, *second_rpy, *check.state.values()])
    results.append(report("model state is finite", finite, f"xyz={second_xyz}"))
    drift = math.dist(first_xyz, second_xyz)
    results.append(report("rover settles on the ground", drift < 0.01 and -0.05 < second_xyz[2] < 0.4,
                          f"z={second_xyz[2]:.4f} m, drift {drift:.5f} m over 3 s"))
    tilt = max(abs(second_rpy[0]), abs(second_rpy[1]))
    results.append(report("rover stays upright", tilt < 0.3, f"max tilt {tilt:.3f} rad"))

    trajectory = JointTrajectory()
    trajectory.joint_names = ARM
    point = JointTrajectoryPoint()
    point.positions = [ARM_GOAL[n] for n in ARM]
    point.time_from_start = RosDuration(sec=3)
    trajectory.points = [point]
    check.spin(1)
    check.arm.publish(trajectory)
    reached = check.wait_for(lambda: all(abs(check.state[n] - ARM_GOAL[n]) < 0.05 for n in ARM), 40)
    results.append(report("arm reaches joint goal", reached, str({n: round(check.state[n], 3) for n in ARM})))

    start, _ = check.pose()
    command = Float64MultiArray()
    command.data = [4.0] * len(WHEELS)
    end = time.monotonic() + 8
    while time.monotonic() < end:
        check.wheels.publish(command)
        check.spin(0.1)
    spinning = all(check.velocity.get(n, 0.0) > 1.0 for n in WHEELS)
    results.append(report("wheels follow velocity command", spinning,
                          str({n: round(check.velocity.get(n, 0.0), 2) for n in WHEELS})))
    moved_xyz, _ = check.pose()
    travel = math.dist(start[:2], moved_xyz[:2])
    results.append(report("wheel drive moves the rover", travel > 0.05, f"{travel:.3f} m"))
    command.data = [0.0] * len(WHEELS)
    check.wheels.publish(command)
    return results


def main():
    rclpy.init()
    check = Check(sys.argv[1])
    try:
        results = run(check)
    finally:
        check.node.destroy_node()
        rclpy.shutdown()
    return 0 if results and all(results) else 1


if __name__ == "__main__":
    sys.exit(main())

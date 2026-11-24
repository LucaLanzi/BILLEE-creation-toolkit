"""Observe the BILLEE stack with the loaded robot_description: controllers, TF, and driving."""

import math
import re
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

PAIRS = [("odom", "base_footprint"), ("base_footprint", "base_link"), ("base_link", "camera_link"),
         ("camera_link", "camera_link_optical")]
WHEELS = {f"joint_wheel_{side}{n}" for side in "lr" for n in "123"}
GROUND = "/gz/odom"
WHEEL_ODOM = "/diff_drive_controller/odom"


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    rclpy.init()
    node = rclpy.create_node("dropin_check")
    buffer = Buffer()
    TransformListener(buffer, node)
    latest = {}
    joints = set()
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

    def on_odom(message, topic):
        p = message.pose.pose
        latest[topic] = (p.position.x, p.position.y, yaw_of(p.orientation))

    for topic in (WHEEL_ODOM, GROUND):
        node.create_subscription(Odometry, topic, lambda m, t=topic: on_odom(m, t), qos)
    node.create_subscription(JointState, "/joint_states", lambda m: joints.update(m.name), qos_profile_sensor_data)
    command = node.create_publisher(Twist, "/diff_drive_controller/cmd_vel_unstamped", 10)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    def drive(linear, angular, seconds):
        message = Twist()
        message.linear.x, message.angular.z = linear, angular
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            command.publish(message)
            spin(0.1)
        command.publish(Twist())
        spin(1.0)

    end = time.monotonic() + 60
    while time.monotonic() < end and not (WHEEL_ODOM in latest and GROUND in latest
                                           and buffer.can_transform("odom", "base_footprint", Time())):
        rclpy.spin_once(node, timeout_sec=0.1)
    spin(3)

    try:
        raw = subprocess.run(["ros2", "control", "list_controllers"], capture_output=True, text=True, timeout=15).stdout
    except subprocess.TimeoutExpired:
        raw = ""
    clean = re.sub(r"\x1b\[[0-9;]*m", "", raw)
    active = sorted(line.split()[0] for line in clean.splitlines() if re.search(r"\bactive\b", line))
    print(f"RESULT active_controllers={','.join(active) or 'none'}")
    print(f"RESULT six_wheels_publish_state={WHEELS <= joints}")
    for parent, child in PAIRS:
        print(f"RESULT tf_{parent}_to_{child}={bool(buffer.can_transform(parent, child, Time()))}")
    print(f"RESULT wheel_odometry={WHEEL_ODOM in latest}")
    print(f"RESULT ground_truth={GROUND in latest}")
    if GROUND not in latest or WHEEL_ODOM not in latest:
        print("RESULT drives_forward=False")
        print("RESULT turns_left=False")
        node.destroy_node()
        rclpy.shutdown()
        return

    start_ground, start_wheel = latest[GROUND], latest[WHEEL_ODOM]
    drive(0.3, 0.0, 6.0)
    ground, wheel = latest[GROUND], latest[WHEEL_ODOM]
    dx, dy = ground[0] - start_ground[0], ground[1] - start_ground[1]
    travel = math.hypot(dx, dy)
    forward = dx * math.cos(start_ground[2]) + dy * math.sin(start_ground[2])
    odom_travel = math.hypot(wheel[0] - start_wheel[0], wheel[1] - start_wheel[1])
    print(f"RESULT drives_forward={forward > 0.5}")
    print(f"METRIC ground_travel_m={travel:.3f}")
    print(f"METRIC wheel_odometry_travel_m={odom_travel:.3f}")

    before = latest[GROUND][2]
    drive(0.0, 0.6, 4.0)
    turn = math.atan2(math.sin(latest[GROUND][2] - before), math.cos(latest[GROUND][2] - before))
    print(f"RESULT turns_left={turn > 0.1}")
    print(f"METRIC yaw_change_rad={turn:.3f}")
    node.destroy_node()
    rclpy.shutdown()


main()

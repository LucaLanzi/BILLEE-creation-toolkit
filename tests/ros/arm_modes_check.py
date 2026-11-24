"""Checks for how the arm behaves outside the main MoveIt run.

hold: with the rover stack running but no arm controllers, the arm must not collapse.
pxx:  the team's real pxx driver, in mock mode, must accept the generated CAN controller.
"""

import json
import os
import re
import subprocess
import sys
import tempfile
import time

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM = [f"joint_arm_{n}" for n in range(1, 6)]
CAN = ARM[:3]


def report(name, passed, detail=""):
    print(f"MOVEIT_{'PASS' if passed else 'FAIL'} {name}{': ' + detail if detail else ''}", flush=True)
    return bool(passed)


class States:
    def __init__(self, sim_time):
        self.node = rclpy.create_node("arm_modes_check", parameter_overrides=[Parameter("use_sim_time", value=sim_time)])
        self.position = {}
        self.node.create_subscription(JointState, "/joint_states", self.on_state, qos_profile_sensor_data)

    def on_state(self, message):
        self.position.update(zip(message.name, message.position))

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait_for(self, names, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not set(names) <= set(self.position):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return set(names) <= set(self.position)


def hold():
    states = States(sim_time=True)
    if not report("arm publishes state without MoveIt", states.wait_for(ARM, 45)):
        return False
    states.spin(3)
    started = states.node.get_clock().now()
    states.spin(6)
    elapsed = (states.node.get_clock().now() - started).nanoseconds / 1e9
    if not report("simulation is running during the hold test", elapsed > 2.0, f"{elapsed:.1f} s of sim time"):
        return False

    # A straight-up arm is balanced even with no holding force, so disturb it by driving hard.
    drive = states.node.create_publisher(Twist, "/diff_drive_controller/cmd_vel_unstamped", 10)
    fast, stop, worst = Twist(), Twist(), 0.0
    fast.linear.x = 0.8
    for command, seconds in ((fast, 3.0), (stop, 3.0)):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            drive.publish(command)
            states.spin(0.1)
            worst = max(worst, max(abs(states.position[n]) for n in ARM))
    return report("arm holds under hard acceleration with no arm controller", worst < 0.05,
                  f"largest joint movement {worst:.4f} rad")


def pxx():
    joints = "".join(
        f'<joint name="{name}"><command_interface name="position"/><state_interface name="position"/>'
        f'<state_interface name="velocity"/><param name="node_id">{index}</param><param name="gear_ratio">1.0</param></joint>'
        for index, name in enumerate(CAN, 1))
    links = '<link name="w"/>' + "".join(f'<link name="l{i}"/>' for i in range(3))
    chain = "".join(
        f'<joint name="{name}" type="revolute"><parent link="{"w" if i == 0 else f"l{i - 1}"}"/><child link="l{i}"/>'
        '<axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="10" velocity="1"/></joint>'
        for i, name in enumerate(CAN))
    urdf = (f'<robot name="t">{links}{chain}<ros2_control name="ArmCan" type="system"><hardware>'
            f'<plugin>pxx/PxxSystemHardware</plugin><param name="can_interface">mock</param></hardware>{joints}'
            "</ros2_control></robot>")
    folder = tempfile.mkdtemp()
    path = os.path.join(folder, "pxx.yaml")
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("controller_manager:\n  ros__parameters:\n    update_rate: 100\n    robot_description: "
                     + json.dumps(urdf) + "\n    joint_state_broadcaster:\n      type: joint_state_broadcaster/JointStateBroadcaster\n"
                     "    arm_can_controller:\n      type: joint_trajectory_controller/JointTrajectoryController\n"
                     "arm_can_controller:\n  ros__parameters:\n    joints: " + json.dumps(CAN) +
                     '\n    command_interfaces: ["position"]\n    state_interfaces: ["position", "velocity"]\n')
    log = open("/evidence/pxx.log", "w", encoding="utf-8")
    manager = subprocess.Popen(["ros2", "run", "controller_manager", "ros2_control_node", "--ros-args", "--params-file", path],
                               stdout=log, stderr=subprocess.STDOUT)
    states = States(sim_time=False)
    try:
        time.sleep(3)
        for name in ("joint_state_broadcaster", "arm_can_controller"):
            subprocess.run(["ros2", "run", "controller_manager", "spawner", name, "--controller-manager-timeout", "10"],
                           capture_output=True, timeout=25, check=False)
        raw = subprocess.run(["ros2", "control", "list_controllers"], capture_output=True, text=True, timeout=15).stdout
        clean = re.sub(r"\x1b\[[0-9;]*m", "", raw)
        active = any(line.startswith("arm_can_controller") and re.search(r"\bactive\b", line) for line in clean.splitlines())
        if not report("real pxx driver accepts the CAN controller", active):
            return False
        if not report("pxx mock publishes joint state", states.wait_for(CAN, 12)):
            return False
        publisher = states.node.create_publisher(JointTrajectory, "/arm_can_controller/joint_trajectory", 10)
        goal = dict(zip(CAN, (0.4, -0.3, 0.6)))
        message = JointTrajectory(joint_names=CAN, points=[JointTrajectoryPoint(
            positions=[goal[n] for n in CAN], time_from_start=Duration(sec=2))])
        states.spin(1)
        publisher.publish(message)
        states.spin(4)
        reached = all(abs(states.position[n] - goal[n]) < 0.03 for n in CAN)
        return report("pxx mock follows a trajectory", reached, str({n[-1]: round(states.position[n], 3) for n in CAN}))
    finally:
        states.node.destroy_node()
        manager.terminate()
        try:
            manager.wait(timeout=4)
        except subprocess.TimeoutExpired:
            manager.kill()
        log.close()


def main():
    rclpy.init()
    try:
        passed = hold() if sys.argv[1] == "hold" else pxx()
    finally:
        rclpy.shutdown()
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())

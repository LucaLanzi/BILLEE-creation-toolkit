"""Check published TF against forward kinematics computed from the URDF.

Usage: tf_check.py robot.urdf root_frame tip_frame
"""

import math
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def matmul(a, b):
    return [[sum(a[r][k] * b[k][c] for k in range(4)) for c in range(4)] for r in range(4)]


def pose(xyz, rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0]],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1]],
        [-sp, cp * sr, cp * cr, xyz[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def expected(path, tip):
    joints = {}
    for joint in ET.parse(path).getroot().findall("joint"):
        o = joint.find("origin")
        joints[joint.find("child").get("link")] = (
            joint.find("parent").get("link"),
            pose([float(v) for v in o.get("xyz").split()], [float(v) for v in o.get("rpy").split()]),
        )
    result = [[float(r == c) for c in range(4)] for r in range(4)]
    current = tip
    while current in joints:
        parent, step = joints[current]
        result = matmul(step, result)
        current = parent
    return [result[i][3] for i in range(3)]


def main():
    urdf, root, tip = sys.argv[1:4]
    want = expected(urdf, tip)
    rclpy.init()
    node = rclpy.create_node("rab_tf_check")
    buffer = Buffer()
    TransformListener(buffer, node)
    deadline = time.monotonic() + 30.0
    got = None
    while time.monotonic() < deadline and got is None:
        rclpy.spin_once(node, timeout_sec=0.2)
        if buffer.can_transform(root, tip, Time(), Duration(seconds=0)):
            t = buffer.lookup_transform(root, tip, Time()).transform.translation
            got = [t.x, t.y, t.z]
    node.destroy_node()
    rclpy.shutdown()
    if got is None:
        print(f"no transform from {root} to {tip}")
        return 1
    error = math.dist(got, want)
    print(f"{root} -> {tip}: published {got}, URDF {want}, error {error:.2e} m")
    return 0 if error < 1e-5 else 1


if __name__ == "__main__":
    sys.exit(main())

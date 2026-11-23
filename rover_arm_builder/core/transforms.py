"""Rigid-transform math.

Notation: T_A_B maps coordinates from frame B into frame A, so p_A = T_A_B * p_B
and T_A_C = T_A_B * T_B_C. URDF roll, pitch, yaw are fixed-axis angles with
R = Rz(yaw) * Ry(pitch) * Rx(roll).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

Vec3 = tuple[float, float, float]
Mat3 = tuple[Vec3, Vec3, Vec3]

IDENTITY3: Mat3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
GIMBAL_EPS = 1e-9


def dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(a: Vec3, s: float) -> Vec3:
    return (a[0] * s, a[1] * s, a[2] * s)


def norm(a: Vec3) -> float:
    return math.sqrt(dot(a, a))


def normalized(a: Vec3) -> Vec3:
    length = norm(a)
    if length < 1e-12 or not math.isfinite(length):
        raise ValueError("Cannot normalize a zero-length or non-finite vector.")
    return scale(a, 1.0 / length)


def mat_mul(a: Mat3, b: Mat3) -> Mat3:
    return tuple(
        tuple(sum(a[r][k] * b[k][c] for k in range(3)) for c in range(3)) for r in range(3)
    )  # type: ignore[return-value]


def mat_vec(m: Mat3, v: Vec3) -> Vec3:
    return (dot(m[0], v), dot(m[1], v), dot(m[2], v))


def transpose(m: Mat3) -> Mat3:
    return (
        (m[0][0], m[1][0], m[2][0]),
        (m[0][1], m[1][1], m[2][1]),
        (m[0][2], m[1][2], m[2][2]),
    )


def det(m: Mat3) -> float:
    return dot(m[0], cross(m[1], m[2]))


def orthonormalized(m: Mat3) -> Mat3:
    """Remove single-precision drift from a near-rotation, keeping its X axis."""
    cols = transpose(m)
    x = normalized(cols[0])
    z = normalized(cross(x, cols[1]))
    return transpose((x, cross(z, x), z))


def rot_x(a: float) -> Mat3:
    c, s = math.cos(a), math.sin(a)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def rot_y(a: float) -> Mat3:
    c, s = math.cos(a), math.sin(a)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def rot_z(a: float) -> Mat3:
    c, s = math.cos(a), math.sin(a)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def rpy_to_matrix(rpy: Vec3) -> Mat3:
    roll, pitch, yaw = rpy
    return mat_mul(rot_z(yaw), mat_mul(rot_y(pitch), rot_x(roll)))


def matrix_to_rpy(m: Mat3) -> Vec3:
    """Extract URDF roll, pitch, yaw. At gimbal lock, yaw is set to zero."""
    cos_pitch = math.hypot(m[0][0], m[1][0])
    pitch = math.atan2(-m[2][0], cos_pitch)
    if cos_pitch < GIMBAL_EPS:
        return (math.atan2(-m[1][2], m[1][1]), pitch, 0.0)
    return (math.atan2(m[2][1], m[2][2]), pitch, math.atan2(m[1][0], m[0][0]))


def axis_angle_to_matrix(axis: Vec3, angle: float) -> Mat3:
    x, y, z = normalized(axis)
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    return (
        (c + x * x * C, x * y * C - z * s, x * z * C + y * s),
        (y * x * C + z * s, c + y * y * C, y * z * C - x * s),
        (z * x * C - y * s, z * y * C + x * s, c + z * z * C),
    )


@dataclass(frozen=True)
class Transform:
    """A rigid transform T_A_B with rotation R and translation t."""

    R: Mat3 = IDENTITY3
    t: Vec3 = (0.0, 0.0, 0.0)

    def __matmul__(self, other: "Transform") -> "Transform":
        return Transform(mat_mul(self.R, other.R), add(mat_vec(self.R, other.t), self.t))

    def inverse(self) -> "Transform":
        Rt = transpose(self.R)
        return Transform(Rt, scale(mat_vec(Rt, self.t), -1.0))

    def apply(self, p: Vec3) -> Vec3:
        return add(mat_vec(self.R, p), self.t)

    def rpy(self) -> Vec3:
        return matrix_to_rpy(self.R)

    @staticmethod
    def from_xyz_rpy(xyz: Vec3, rpy: Vec3 = (0.0, 0.0, 0.0)) -> "Transform":
        return Transform(rpy_to_matrix(rpy), xyz)


def relative(T_W_A: Transform, T_W_B: Transform) -> Transform:
    """Return T_A_B = inverse(T_W_A) * T_W_B."""
    return T_W_A.inverse() @ T_W_B


def mount_arm_root(
    T_W_rover_mount: Transform, T_offset: Transform, T_arm_root_arm_mount: Transform
) -> Transform:
    """T_W_armRoot = T_W_roverMount * T_offset * inverse(T_armRoot_armMount)."""
    return T_W_rover_mount @ T_offset @ T_arm_root_arm_mount.inverse()


class AffineKind(Enum):
    RIGID = "rigid"
    UNIFORM_SCALE = "uniform_scale"
    NONUNIFORM_SCALE = "nonuniform_scale"
    SHEAR = "shear"
    REFLECTION = "reflection"
    SINGULAR = "singular"


@dataclass(frozen=True)
class AffineReport:
    kind: AffineKind
    scales: Vec3
    rotation: Mat3 | None


def classify_linear(m: Mat3, tol: float = 1e-6) -> AffineReport:
    """Classify a 3x3 linear part, as found in a Blender world matrix.

    Columns are the images of the local axes. Rotation is returned only when the
    matrix is a rotation times an axis-aligned positive scale.
    """
    cols = transpose(m)
    scales: Vec3 = (norm(cols[0]), norm(cols[1]), norm(cols[2]))
    if not all(math.isfinite(v) for row in m for v in row):
        return AffineReport(AffineKind.SINGULAR, scales, None)
    largest = max(scales)
    if largest < 1e-12 or min(scales) < largest * 1e-9:
        return AffineReport(AffineKind.SINGULAR, scales, None)
    d = det(m)
    if abs(d) < (largest**3) * 1e-12:
        return AffineReport(AffineKind.SINGULAR, scales, None)
    if d < 0.0:
        return AffineReport(AffineKind.REFLECTION, scales, None)

    units = (scale(cols[0], 1 / scales[0]), scale(cols[1], 1 / scales[1]), scale(cols[2], 1 / scales[2]))
    if max(abs(dot(units[0], units[1])), abs(dot(units[0], units[2])), abs(dot(units[1], units[2]))) > tol:
        return AffineReport(AffineKind.SHEAR, scales, None)

    rotation = transpose(units)
    if max(scales) - min(scales) > tol * largest:
        return AffineReport(AffineKind.NONUNIFORM_SCALE, scales, rotation)
    if abs(scales[0] - 1.0) > tol:
        return AffineReport(AffineKind.UNIFORM_SCALE, scales, rotation)
    return AffineReport(AffineKind.RIGID, scales, rotation)

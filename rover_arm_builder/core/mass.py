"""Mass properties for uniform-density primitives and rigid composites.

Tensor convention matches URDF: the matrix is [[ixx, ixy, ixz], [ixy, iyy, iyz],
[ixz, iyz, izz]], with products of inertia ixy = -integral(x * y dm).
All values are SI and taken about the center of mass.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .transforms import IDENTITY3, Mat3, Vec3, add, dot, mat_mul, scale, sub, transpose


@dataclass(frozen=True)
class Inertia:
    ixx: float
    ixy: float
    ixz: float
    iyy: float
    iyz: float
    izz: float

    def matrix(self) -> Mat3:
        return (
            (self.ixx, self.ixy, self.ixz),
            (self.ixy, self.iyy, self.iyz),
            (self.ixz, self.iyz, self.izz),
        )

    @staticmethod
    def from_matrix(m: Mat3) -> "Inertia":
        """Build from a matrix, averaging mirrored terms to remove rounding asymmetry."""
        return Inertia(
            m[0][0],
            (m[0][1] + m[1][0]) / 2.0,
            (m[0][2] + m[2][0]) / 2.0,
            m[1][1],
            (m[1][2] + m[2][1]) / 2.0,
            m[2][2],
        )

    def rotated(self, R: Mat3) -> "Inertia":
        """Return R * I * transpose(R)."""
        return Inertia.from_matrix(mat_mul(R, mat_mul(self.matrix(), transpose(R))))


def _require_positive(**values: float) -> None:
    for name, value in values.items():
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be positive and finite, got {value!r}.")


def box_inertia(mass: float, x: float, y: float, z: float) -> Inertia:
    _require_positive(mass=mass, x=x, y=y, z=z)
    return Inertia(
        mass * (y * y + z * z) / 12.0, 0.0, 0.0,
        mass * (x * x + z * z) / 12.0, 0.0,
        mass * (x * x + y * y) / 12.0,
    )


def cylinder_inertia(mass: float, radius: float, length: float) -> Inertia:
    """Solid cylinder with its axis along local Z."""
    _require_positive(mass=mass, radius=radius, length=length)
    radial = mass * (3.0 * radius * radius + length * length) / 12.0
    return Inertia(radial, 0.0, 0.0, radial, 0.0, mass * radius * radius / 2.0)


def sphere_inertia(mass: float, radius: float) -> Inertia:
    _require_positive(mass=mass, radius=radius)
    value = 0.4 * mass * radius * radius
    return Inertia(value, 0.0, 0.0, value, 0.0, value)


def box_volume(x: float, y: float, z: float) -> float:
    _require_positive(x=x, y=y, z=z)
    return x * y * z


def cylinder_volume(radius: float, length: float) -> float:
    _require_positive(radius=radius, length=length)
    return math.pi * radius * radius * length


def sphere_volume(radius: float) -> float:
    _require_positive(radius=radius)
    return 4.0 / 3.0 * math.pi * radius**3


@dataclass(frozen=True)
class MassContributor:
    """One physical body. `com` and `R` are expressed in the link frame.

    `inertia` is about this body's own COM, in the axes given by `R`.
    """

    mass: float
    com: Vec3
    inertia: Inertia
    R: Mat3 = IDENTITY3


@dataclass(frozen=True)
class MassProperties:
    mass: float
    com: Vec3
    inertia: Inertia


def combine(contributors: list[MassContributor]) -> MassProperties:
    """Aggregate bodies. The result is about the composite COM, in link-frame axes."""
    if not contributors:
        raise ValueError("Cannot combine an empty list of mass contributors.")
    for item in contributors:
        _require_positive(mass=item.mass)

    total = sum(item.mass for item in contributors)
    weighted: Vec3 = (0.0, 0.0, 0.0)
    for item in contributors:
        weighted = add(weighted, scale(item.com, item.mass))
    com = scale(weighted, 1.0 / total)

    summed = [[0.0] * 3 for _ in range(3)]
    for item in contributors:
        rotated = item.inertia.rotated(item.R).matrix()
        d = sub(item.com, com)
        d2 = dot(d, d)
        for r in range(3):
            for c in range(3):
                shift = item.mass * ((d2 if r == c else 0.0) - d[r] * d[c])
                summed[r][c] += rotated[r][c] + shift
    matrix: Mat3 = (tuple(summed[0]), tuple(summed[1]), tuple(summed[2]))  # type: ignore[assignment]
    return MassProperties(total, com, Inertia.from_matrix(matrix))


def principal_moments(inertia: Inertia) -> Vec3:
    """Eigenvalues of the symmetric tensor in ascending order."""
    a = inertia
    off = a.ixy**2 + a.ixz**2 + a.iyz**2
    if off == 0.0:
        low, mid, high = sorted((a.ixx, a.iyy, a.izz))
        return (low, mid, high)
    q = (a.ixx + a.iyy + a.izz) / 3.0
    p2 = (a.ixx - q) ** 2 + (a.iyy - q) ** 2 + (a.izz - q) ** 2 + 2.0 * off
    p = math.sqrt(p2 / 6.0)
    b = Inertia(
        (a.ixx - q) / p, a.ixy / p, a.ixz / p, (a.iyy - q) / p, a.iyz / p, (a.izz - q) / p
    ).matrix()
    det_b = (
        b[0][0] * (b[1][1] * b[2][2] - b[1][2] * b[2][1])
        - b[0][1] * (b[1][0] * b[2][2] - b[1][2] * b[2][0])
        + b[0][2] * (b[1][0] * b[2][1] - b[1][1] * b[2][0])
    )
    phi = math.acos(max(-1.0, min(1.0, det_b / 2.0))) / 3.0
    high = q + 2.0 * p * math.cos(phi)
    low = q + 2.0 * p * math.cos(phi + 2.0 * math.pi / 3.0)
    mid = 3.0 * q - high - low
    return (low, mid, high)


def inertia_problems(mass: float, inertia: Inertia, rel_tol: float = 1e-9) -> list[str]:
    """Return readable reasons this mass and tensor are not physical. Empty means valid."""
    values = (inertia.ixx, inertia.ixy, inertia.ixz, inertia.iyy, inertia.iyz, inertia.izz)
    if not math.isfinite(mass) or not all(math.isfinite(v) for v in values):
        return ["Mass and inertia values must all be finite numbers."]

    problems: list[str] = []
    if mass <= 0.0:
        problems.append(f"Mass must be greater than zero, got {mass:g} kg.")
    low, mid, high = principal_moments(inertia)
    tol = rel_tol * max(abs(high), 1e-30)
    if low <= tol:
        problems.append(
            f"Inertia is not positive definite: smallest principal moment is {low:.6g} kg*m^2."
        )
    elif low + mid < high - tol:
        problems.append(
            "Principal moments break the triangle inequality "
            f"({low:.6g} + {mid:.6g} < {high:.6g}), so no real body has this tensor."
        )
    return problems

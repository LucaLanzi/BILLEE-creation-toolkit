"""Deterministic little-endian binary STL writer."""

from __future__ import annotations

import struct

from ..core.model import Mesh
from ..core.transforms import cross, norm, scale, sub

_HEADER = b"rover_arm_builder binary STL (meters)".ljust(80, b"\0")


def mesh_to_stl_bytes(mesh: Mesh) -> bytes:
    parts = [_HEADER, struct.pack("<I", len(mesh.triangles))]
    for a, b, c in mesh.triangles:
        normal = cross(sub(b, a), sub(c, a))
        length = norm(normal)
        unit = scale(normal, 1.0 / length) if length > 0.0 else (0.0, 0.0, 0.0)
        parts.append(struct.pack("<12fH", *unit, *a, *b, *c, 0))
    return b"".join(parts)

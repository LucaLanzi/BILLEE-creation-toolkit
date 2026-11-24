"""Deterministic little-endian binary STL writer."""

from __future__ import annotations

import math
import struct

from ..core.model import Box, Cylinder, Mesh, Sphere
from ..core.transforms import cross, norm, scale, sub

_HEADER = b"rover_arm_builder binary STL (meters)".ljust(80, b"\0")


def tessellate(shape, segments: int = 24) -> tuple:
    """Triangles with outward normals for an analytic box, Z-axis cylinder, or sphere."""
    triangles = []
    if isinstance(shape, Box):
        half = (shape.x / 2.0, shape.y / 2.0, shape.z / 2.0)
        for axis in range(3):
            u, v = (axis + 1) % 3, (axis + 2) % 3
            for sign in (1.0, -1.0):
                corners = []
                for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                    point = [0.0, 0.0, 0.0]
                    point[axis], point[u], point[v] = sign * half[axis], a * half[u], b * half[v]
                    corners.append(tuple(point))
                if sign < 0.0:
                    corners.reverse()
                triangles += [(corners[0], corners[1], corners[2]), (corners[0], corners[2], corners[3])]
    elif isinstance(shape, Cylinder):
        top, bottom = shape.length / 2.0, -shape.length / 2.0
        ring = [(shape.radius * math.cos(2 * math.pi * i / segments), shape.radius * math.sin(2 * math.pi * i / segments))
                for i in range(segments)]
        for i, (x1, y1) in enumerate(ring):
            x2, y2 = ring[(i + 1) % segments]
            triangles += [((x1, y1, bottom), (x2, y2, bottom), (x2, y2, top)),
                          ((x1, y1, bottom), (x2, y2, top), (x1, y1, top)),
                          ((0.0, 0.0, top), (x1, y1, top), (x2, y2, top)),
                          ((0.0, 0.0, bottom), (x2, y2, bottom), (x1, y1, bottom))]
    elif isinstance(shape, Sphere):
        rings = segments // 2

        def point(ring_index, step):
            theta, phi = math.pi * ring_index / rings, 2 * math.pi * step / segments
            return (shape.radius * math.sin(theta) * math.cos(phi), shape.radius * math.sin(theta) * math.sin(phi),
                    shape.radius * math.cos(theta))

        for r in range(rings):
            for s in range(segments):
                a, b, c, d = point(r, s), point(r + 1, s), point(r + 1, s + 1), point(r, s + 1)
                if r < rings - 1:
                    triangles.append((a, b, c))
                if r > 0:
                    triangles.append((a, c, d))
    else:
        raise ValueError("Only boxes, cylinders, and spheres can be tessellated.")
    return tuple(triangles)


def mesh_to_stl_bytes(mesh: Mesh) -> bytes:
    parts = [_HEADER, struct.pack("<I", len(mesh.triangles))]
    for a, b, c in mesh.triangles:
        normal = cross(sub(b, a), sub(c, a))
        length = norm(normal)
        unit = scale(normal, 1.0 / length) if length > 0.0 else (0.0, 0.0, 0.0)
        parts.append(struct.pack("<12fH", *unit, *a, *b, *c, 0))
    return b"".join(parts)

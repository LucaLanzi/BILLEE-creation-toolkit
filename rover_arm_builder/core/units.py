"""Conversion of user-facing units to SI. Stored model values are always SI."""

from __future__ import annotations

import math

LENGTH_TO_M = {"mm": 0.001, "cm": 0.01, "m": 1.0}
MASS_TO_KG = {"g": 0.001, "kg": 1.0}
ANGLE_TO_RAD = {"deg": math.pi / 180.0, "rad": 1.0}


def _convert(value: float, unit: str, table: dict[str, float], kind: str) -> float:
    try:
        factor = table[unit]
    except KeyError:
        options = ", ".join(sorted(table))
        raise ValueError(f"Unknown {kind} unit {unit!r}. Use one of: {options}.") from None
    if not math.isfinite(value):
        raise ValueError(f"{kind.capitalize()} value must be finite, got {value!r}.")
    return value * factor


def length_to_m(value: float, unit: str) -> float:
    return _convert(value, unit, LENGTH_TO_M, "length")


def mass_to_kg(value: float, unit: str) -> float:
    return _convert(value, unit, MASS_TO_KG, "mass")


def angle_to_rad(value: float, unit: str) -> float:
    return _convert(value, unit, ANGLE_TO_RAD, "angle")


def blender_length_to_m(value: float, scale_length: float) -> float:
    """Convert a raw Blender length to meters using the scene's unit scale."""
    if not math.isfinite(scale_length) or scale_length <= 0.0:
        raise ValueError(f"Scene unit scale must be positive and finite, got {scale_length!r}.")
    return value * scale_length

"""Per-target readiness with staleness tracking.

Each recorded stage stores a fingerprint of the model sections that target depends on.
When those sections change, the record no longer applies and the target falls back to
"Authored". A stage is only recorded when the matching check actually ran.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from enum import Enum

from .assembly import FlatRobot
from .model import Box, Cylinder, Mesh, Sphere
from .transforms import Transform
from .validate import Target, ValidationReport


class Stage(Enum):
    MISSING_INPUTS = "Missing inputs"
    AUTHORED = "Authored"
    EXPORTED = "Exported"
    INTERNALLY_VALIDATED = "Internally validated"
    EXTERNALLY_VALIDATED = "Externally validated"
    TESTED_IN_RUNTIME = "Tested in target runtime"
    NOT_CHECKED = "Not checked"


RECORDABLE = (
    Stage.EXPORTED,
    Stage.INTERNALLY_VALIDATED,
    Stage.EXTERNALLY_VALIDATED,
    Stage.TESTED_IN_RUNTIME,
)


@dataclass(frozen=True)
class TargetStatus:
    target: Target
    stage: Stage
    detail: str


def _round(values) -> list[float]:
    return [round(float(v), 9) + 0.0 for v in values]


def _pose(pose: Transform) -> list[float]:
    return _round(pose.t) + _round(v for row in pose.R for v in row)


def _shape(shape) -> list:
    if isinstance(shape, Box):
        return ["box", *_round((shape.x, shape.y, shape.z))]
    if isinstance(shape, Cylinder):
        return ["cylinder", *_round((shape.radius, shape.length))]
    if isinstance(shape, Sphere):
        return ["sphere", *_round((shape.radius,))]
    if isinstance(shape, Mesh):
        digest = hashlib.sha256(repr(shape.triangles).encode()).hexdigest()
        return ["mesh", shape.asset, digest]
    return ["unknown"]


def _elements(items) -> list:
    return sorted(([item.name, _shape(item.shape), _pose(item.origin)] for item in items), key=lambda e: e[0])


def fingerprint(robot: FlatRobot, target: Target) -> str:
    """Hash the sections this target depends on."""
    joints = []
    for j in sorted(robot.joints, key=lambda item: item.name):
        entry = [j.name, j.type.value, j.parent, j.child, _pose(j.origin), _round(j.axis)]
        if target is not Target.VISUALIZATION:
            limit = j.limit
            entry.append(None if limit is None else [limit.lower, limit.upper, limit.velocity, limit.effort])
            entry.extend([j.actuation.value, j.role.value])
        if target is Target.PHYSICS:
            entry.extend([round(j.damping, 9), round(j.friction, 9)])
        joints.append(entry)

    links = []
    for link in sorted(robot.links, key=lambda item: item.name):
        entry = [link.name, _elements(link.visuals)]
        if target is not Target.VISUALIZATION:
            entry.append(_elements(link.collisions))
        if target is Target.PHYSICS and link.mass is not None:
            i = link.mass.inertia
            entry.append(_round((link.mass.mass, *link.mass.com, i.ixx, i.ixy, i.ixz, i.iyy, i.iyz, i.izz)))
        links.append(entry)

    payload = json.dumps({"links": links, "joints": joints}, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def record_stage(records: dict, robot: FlatRobot, target: Target, stage: Stage) -> dict:
    """Return updated records, storing that `stage` ran against the current model."""
    if stage not in RECORDABLE:
        raise ValueError(f"{stage.value} is computed and cannot be recorded.")
    updated = {key: dict(value) for key, value in records.items()}
    current = fingerprint(robot, target)
    entry = updated.get(target.value, {})
    if entry.get("hash") != current:
        entry = {"hash": current, "stages": []}
    stages = set(entry.get("stages", []))
    stages.add(stage.name)
    entry["stages"] = sorted(stages)
    updated[target.value] = entry
    return updated


def evaluate(report: ValidationReport, records: dict) -> list[TargetStatus]:
    """Compute the honest status of every target."""
    statuses = []
    for target in Target:
        blockers = report.blockers(target)
        if blockers or report.robot is None:
            count = len(blockers)
            statuses.append(TargetStatus(target, Stage.MISSING_INPUTS, f"{count} blocking issue(s)"))
            continue
        entry = records.get(target.value)
        if not entry:
            statuses.append(TargetStatus(target, Stage.AUTHORED, "Not exported or checked yet"))
            continue
        if entry.get("hash") != fingerprint(report.robot, target):
            statuses.append(TargetStatus(target, Stage.AUTHORED, "Model changed since the last check"))
            continue
        recorded = set(entry.get("stages", []))
        best = next((s for s in reversed(RECORDABLE) if s.name in recorded), None)
        if best is None:
            statuses.append(TargetStatus(target, Stage.NOT_CHECKED, "No recorded checks"))
        else:
            missing = [s.value for s in RECORDABLE if s.name not in recorded]
            detail = "Not yet: " + ", ".join(missing) if missing else "All recorded checks are current"
            statuses.append(TargetStatus(target, best, detail))
    return statuses

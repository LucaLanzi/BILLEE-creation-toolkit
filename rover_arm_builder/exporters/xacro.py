"""Reusable Xacro macros generated from the validated model.

Parametric: module prefix, mount pose, primitive dimensions of mass-bearing components,
mass or density, and joint limits. Inertia and center of mass are expressions of those
parameters. Joint placements and component placements are fixed values: they are not
modelled as functions of dimensions.
"""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET

from ..core.assembly import frame_pose_in_root
from ..core.model import Box, Component, Cylinder, GeometryElement, JointType, MassMode, Mesh, Module, Project, Sphere
from ..core.transforms import Transform, normalized
from .urdf import fmt, fmt_pose, mesh_filename

XACRO_NS = "http://www.ros.org/wiki/xacro"
ET.register_namespace("xacro", XACRO_NS)


def _x(tag: str) -> str:
    return f"{{{XACRO_NS}}}{tag}"


class _Params:
    """Macro parameters with unique, valid identifiers, in a stable order."""

    def __init__(self) -> None:
        self.items: list[tuple[str, str]] = []
        self._used = {"prefix"}

    def unique(self, raw: str) -> str:
        base = re.sub(r"\W", "_", raw)
        if not re.match(r"[A-Za-z]", base):
            base = f"p_{base}"
        name, index = base, 2
        while name in self._used:
            name, index = f"{base}_{index}", index + 1
        self._used.add(name)
        return name

    def add(self, raw: str, default: float) -> str:
        name = self.unique(raw)
        self.items.append((name, fmt(default)))
        return name


def _origin(parent: ET.Element, pose: Transform) -> None:
    ET.SubElement(parent, "origin", {
        "xyz": " ".join(fmt_pose(v) for v in pose.t),
        "rpy": " ".join(fmt_pose(v) for v in pose.rpy()),
    })


def _coef_terms(pairs: list[tuple[float, str]]) -> str:
    kept = [f"({fmt(c)})*({e})" for c, e in pairs if abs(c) > 1e-15]
    return " + ".join(kept) if kept else "0"


class _ComponentExpr:
    """Symbolic mass, dimensions, and local inertia of one component."""

    def __init__(self, link_name: str, component: Component, params: _Params, macro: ET.Element) -> None:
        spec = component.mass
        self.spec = spec
        self.dims: dict[str, str] = {}
        stem = f"{link_name}_{component.name}"
        self.manual = spec.mode is MassMode.MANUAL
        shape = spec.shape
        if isinstance(shape, Box):
            self.dims = {k: params.add(f"{stem}_{k}", getattr(shape, k)) for k in ("x", "y", "z")}
        elif isinstance(shape, Cylinder):
            self.dims = {k: params.add(f"{stem}_{k}", getattr(shape, k)) for k in ("radius", "length")}
        elif isinstance(shape, Sphere):
            self.dims = {"radius": params.add(f"{stem}_radius", shape.radius)}

        d = self.dims
        if spec.mode is MassMode.DENSITY:
            density = params.add(f"{stem}_density", spec.density)
            if isinstance(shape, Box):
                volume = f"{d['x']}*{d['y']}*{d['z']}"
            elif isinstance(shape, Cylinder):
                volume = f"pi*{d['radius']}**2*{d['length']}"
            else:
                volume = f"4.0/3.0*pi*{d['radius']}**3"
            self.mass = params.unique(f"{stem}_mass_value")
            ET.SubElement(macro, _x("property"), {"name": self.mass, "value": f"${{{density}*{volume}}}"})
        else:
            self.mass = params.add(f"{stem}_mass", spec.mass)

        m = self.mass
        if self.manual:
            self.local = None
        elif isinstance(shape, Box):
            self.local = (
                f"{m}*({d['y']}**2+{d['z']}**2)/12.0",
                f"{m}*({d['x']}**2+{d['z']}**2)/12.0",
                f"{m}*({d['x']}**2+{d['y']}**2)/12.0",
            )
        elif isinstance(shape, Cylinder):
            radial = f"{m}*(3.0*{d['radius']}**2+{d['length']}**2)/12.0"
            self.local = (radial, radial, f"{m}*{d['radius']}**2/2.0")
        else:
            value = f"0.4*{m}*{d['radius']}**2"
            self.local = (value, value, value)

    def rotated_term(self, a: int, b: int) -> str:
        R = self.spec.origin.R
        if self.local is None:
            value = self.spec.manual_inertia.rotated(R).matrix()[a][b]
            return fmt(value)
        return _coef_terms([(R[a][k] * R[b][k], self.local[k]) for k in range(3)])

    def matches(self, shape) -> bool:
        return bool(self.dims) and shape == self.spec.shape


def _geometry(parent: ET.Element, item: GeometryElement, expr: _ComponentExpr | None, package: str) -> None:
    node = ET.SubElement(parent, "geometry")
    shape = item.shape
    d = expr.dims if expr is not None and expr.matches(shape) else None
    if isinstance(shape, Box):
        size = f"${{{d['x']}}} ${{{d['y']}}} ${{{d['z']}}}" if d else f"{fmt(shape.x)} {fmt(shape.y)} {fmt(shape.z)}"
        ET.SubElement(node, "box", {"size": size})
    elif isinstance(shape, Cylinder):
        ET.SubElement(node, "cylinder", {
            "radius": f"${{{d['radius']}}}" if d else fmt(shape.radius),
            "length": f"${{{d['length']}}}" if d else fmt(shape.length),
        })
    elif isinstance(shape, Sphere):
        ET.SubElement(node, "sphere", {"radius": f"${{{d['radius']}}}" if d else fmt(shape.radius)})
    elif isinstance(shape, Mesh):
        ET.SubElement(node, "mesh", {"filename": f"package://{package}/meshes/{mesh_filename(shape.asset)}"})


def _inertial(node: ET.Element, link_name: str, exprs: list[_ComponentExpr], macro: ET.Element) -> None:
    stem = re.sub(r"\W", "_", link_name)
    total = f"{stem}_total_mass"
    ET.SubElement(macro, _x("property"), {"name": total, "value": "${" + " + ".join(e.mass for e in exprs) + "}"})
    com = []
    for axis, label in enumerate("xyz"):
        name = f"{stem}_com_{label}"
        sum_expr = " + ".join(f"{e.mass}*({fmt(e.spec.origin.t[axis])})" for e in exprs)
        ET.SubElement(macro, _x("property"), {"name": name, "value": f"${{({sum_expr})/{total}}}"})
        com.append(name)

    def delta(e: _ComponentExpr, i: int) -> str:
        return f"({fmt(e.spec.origin.t[i])}-{com[i]})"

    values = {}
    for key, a, b in (("ixx", 0, 0), ("ixy", 0, 1), ("ixz", 0, 2), ("iyy", 1, 1), ("iyz", 1, 2), ("izz", 2, 2)):
        parts = []
        for e in exprs:
            shift = f"-{delta(e, a)}*{delta(e, b)}"
            if a == b:
                shift = f"{delta(e, 0)}**2+{delta(e, 1)}**2+{delta(e, 2)}**2{shift}"
            parts.append(f"({e.rotated_term(a, b)}) + {e.mass}*({shift})")
        values[key] = "${" + " + ".join(parts) + "}"

    inertial = ET.SubElement(node, "inertial")
    ET.SubElement(inertial, "origin", {"xyz": " ".join(f"${{{c}}}" for c in com), "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": f"${{{total}}}"})
    ET.SubElement(inertial, "inertia", values)


def macro_name(module: Module) -> str:
    return re.sub(r"\W", "_", module.name.lower()) + "_macro"


def module_macro_xml(module: Module, package: str) -> str:
    root = ET.Element("robot")
    macro = ET.SubElement(root, _x("macro"), {"name": macro_name(module), "params": ""})
    params = _Params()
    names = {l.id: l.name for l in module.links}
    body: list[ET.Element] = []

    materials = {
        v.material.name: v.material
        for link in module.links for c in link.components for v in c.visuals if v.material is not None
    }
    for name in sorted(materials):
        node = ET.Element("material", {"name": f"${{prefix}}{name}"})
        ET.SubElement(node, "color", {"rgba": " ".join(fmt(v) for v in materials[name].rgba)})
        body.append(node)

    for link in sorted(module.links, key=lambda l: l.name):
        node = ET.Element("link", {"name": f"${{prefix}}{link.name}"})
        exprs: dict[str, _ComponentExpr] = {}
        for component in link.components:
            if component.mass is not None:
                exprs[component.id] = _ComponentExpr(link.name, component, params, macro)
        if exprs:
            _inertial(node, link.name, list(exprs.values()), macro)
        owned = [(v, exprs.get(c.id)) for c in link.components for v in c.visuals]
        for visual, expr in sorted(owned, key=lambda p: p[0].name):
            element = ET.SubElement(node, "visual", {"name": visual.name})
            _origin(element, visual.origin)
            _geometry(element, visual, expr, package)
            if visual.material is not None:
                ET.SubElement(element, "material", {"name": f"${{prefix}}{visual.material.name}"})
        owned = [(v, exprs.get(c.id)) for c in link.components for v in c.collisions]
        for collision, expr in sorted(owned, key=lambda p: p[0].name):
            element = ET.SubElement(node, "collision", {"name": collision.name})
            _origin(element, collision.origin)
            _geometry(element, collision, expr, package)
        body.append(node)

    for frame in sorted(module.frames, key=lambda f: f.name):
        body.append(ET.Element("link", {"name": f"${{prefix}}{frame.name}"}))
        joint = ET.Element("joint", {"name": f"${{prefix}}{frame.name}_joint", "type": "fixed"})
        _origin(joint, frame.pose)
        ET.SubElement(joint, "parent", {"link": f"${{prefix}}{names[frame.link_id]}"})
        ET.SubElement(joint, "child", {"link": f"${{prefix}}{frame.name}"})
        body.append(joint)

    for joint in sorted(module.joints, key=lambda j: j.name):
        node = ET.Element("joint", {"name": f"${{prefix}}{joint.name}", "type": joint.type.value})
        _origin(node, joint.origin)
        ET.SubElement(node, "parent", {"link": f"${{prefix}}{names[joint.parent_id]}"})
        ET.SubElement(node, "child", {"link": f"${{prefix}}{names[joint.child_id]}"})
        if joint.type is not JointType.FIXED:
            ET.SubElement(node, "axis", {"xyz": " ".join(fmt_pose(v) for v in normalized(joint.axis))})
            limit = joint.limit
            if limit is not None and limit.effort is not None and limit.velocity is not None:
                attrs = {}
                if joint.type is not JointType.CONTINUOUS:
                    attrs["lower"] = f"${{{params.add(f'{joint.name}_lower', limit.lower)}}}"
                    attrs["upper"] = f"${{{params.add(f'{joint.name}_upper', limit.upper)}}}"
                attrs["effort"] = f"${{{params.add(f'{joint.name}_effort', limit.effort)}}}"
                attrs["velocity"] = f"${{{params.add(f'{joint.name}_velocity', limit.velocity)}}}"
                ET.SubElement(node, "limit", attrs)
            if joint.damping or joint.friction:
                ET.SubElement(node, "dynamics", {"damping": fmt(joint.damping), "friction": fmt(joint.friction)})
        body.append(node)

    macro.extend(body)
    # An empty default must be quoted, or xacro reads the next parameter as its value.
    declared = [f"prefix:={module.prefix or chr(39) * 2}"] + [f"{n}:={v}" for n, v in params.items]
    macro.set("params", " ".join(declared))
    return _serialize(root)


def _serialize(root: ET.Element) -> str:
    ET.indent(root, space="  ")
    return '<?xml version="1.0"?>\n' + ET.tostring(root, encoding="unicode") + "\n"


def module_filename(module: Module) -> str:
    return f"{macro_name(module)}.xacro"


def top_level_xacro(project: Project, robot_name: str, module_ids: list[str]) -> str:
    """A wrapper that expands to one connected robot from the selected modules."""
    root = ET.Element("robot", {"name": robot_name})
    selected = [m for m in project.modules if m.id in module_ids]
    for module in selected:
        ET.SubElement(root, _x("include"), {"filename": module_filename(module)})
    for module in selected:
        ET.SubElement(root, _x(macro_name(module)), {"prefix": module.prefix})

    chosen = {m.id for m in selected}
    for mount in project.mounts:
        if mount.parent_module_id not in chosen or mount.child_module_id not in chosen:
            continue
        parent = project.module(mount.parent_module_id)
        child = project.module(mount.child_module_id)
        parent_frame = next(f for f in parent.frames if f.id == mount.parent_frame_id)
        parent_link = next(l for l in parent.links if l.id == parent_frame.link_id)
        child_root = next(l for l in child.links if l.id == child.root_link_id)
        pose = parent_frame.pose @ mount.offset @ frame_pose_in_root(child, mount.child_frame_id).inverse()
        stem = re.sub(r"\W", "_", f"{child.prefix}mount")
        values = {}
        for label, value in zip(("x", "y", "z", "roll", "pitch", "yaw"), (*pose.t, *pose.rpy())):
            values[label] = f"{stem}_{label}"
            ET.SubElement(root, _x("property"), {"name": values[label], "value": fmt_pose(value)})
        joint = ET.SubElement(root, "joint", {"name": f"{child.prefix}mount_joint", "type": "fixed"})
        ET.SubElement(joint, "origin", {
            "xyz": f"${{{values['x']}}} ${{{values['y']}}} ${{{values['z']}}}",
            "rpy": f"${{{values['roll']}}} ${{{values['pitch']}}} ${{{values['yaw']}}}",
        })
        ET.SubElement(joint, "parent", {"link": f"{parent.prefix}{parent_link.name}"})
        ET.SubElement(joint, "child", {"link": f"{child.prefix}{child_root.name}"})
    return _serialize(root)

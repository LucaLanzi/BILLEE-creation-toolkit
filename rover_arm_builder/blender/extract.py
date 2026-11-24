"""Read scene metadata and objects into the core model without changing the scene.

Meters are computed here, exactly once. Values are rounded to 7 significant digits
because Blender stores single-precision floats, so equal geometry authored at
different unit scales exports identically.
"""

import re

import bpy
from mathutils import Matrix

from ..core import model as X
from ..core.mass import Inertia
from ..core.transforms import AffineKind, Transform, classify_linear, det, orthonormalized, rpy_to_matrix
from ..core.validate import ALL_TARGETS, ValidationReport, validate_project
from .authoring import find, rigid

CLASSIFY_TOL = 1e-5


def si(value, scale):
    return float(f"{value * scale:.7g}")


def _mat3(matrix):
    return tuple(tuple(float(matrix[r][c]) for c in range(3)) for r in range(3))


def _pose(pose, scale):
    rpy = tuple(float(v) for v in pose.rotation)
    return Transform(rpy_to_matrix(rpy), tuple(si(v, scale) for v in pose.location))


def _asset_name(prefix, name, uid):
    return re.sub(r"[^A-Za-z0-9_]", "_", f"{prefix}{name}") + f"_{uid[:6]}"


def _mesh_shape(obj, relative, depsgraph, scale, asset):
    evaluated = obj.evaluated_get(depsgraph)
    mesh = evaluated.to_mesh()
    try:
        mesh.calc_loop_triangles()
        flip = det(_mat3(relative)) < 0.0
        points = [relative @ v.co for v in mesh.vertices]
        triangles = []
        for tri in mesh.loop_triangles:
            a, b, c = (tuple(si(v, scale) for v in points[i]) for i in tri.vertices)
            triangles.append((a, c, b) if flip else (a, b, c))
        return X.Mesh(asset, tuple(triangles))
    finally:
        evaluated.to_mesh_clear()


def _analytic(item, report, scale):
    """Return an analytic shape in meters, or None with a reason it cannot stay analytic."""
    if report.kind is AffineKind.RIGID:
        sx, sy, sz = 1.0, 1.0, 1.0
    else:
        sx, sy, sz = (float(f"{v:.6g}") for v in report.scales)
    if report.kind in (AffineKind.RIGID, AffineKind.UNIFORM_SCALE, AffineKind.NONUNIFORM_SCALE):
        close = lambda a, b: abs(a - b) <= CLASSIFY_TOL * max(a, b)
        if item.shape == "BOX":
            return X.Box(si(item.size[0] * sx, scale), si(item.size[1] * sy, scale), si(item.size[2] * sz, scale)), ""
        if item.shape == "CYLINDER":
            if close(sx, sy):
                return X.Cylinder(si(item.radius * sx, scale), si(item.length * sz, scale)), ""
            return None, "its radial axes are scaled differently, so it is no longer a circular cylinder"
        if item.shape == "SPHERE":
            if close(sx, sy) and close(sy, sz):
                return X.Sphere(si(item.radius * sx, scale)), ""
            return None, "it is stretched, so it is no longer a sphere"
    reasons = {
        AffineKind.REFLECTION: "its transform is mirrored",
        AffineKind.SHEAR: "its transform is sheared",
        AffineKind.SINGULAR: "its transform is flattened to zero thickness",
    }
    return None, reasons.get(report.kind, "its transform cannot be represented")


def _relative_to_link(obj, link_object, W_L):
    """Return T_L_O from the parent chain, so world placement and preview add no float noise.

    Objects with constraints or non-object parenting fall back to world matrices,
    because only the evaluated world matrix describes them.
    """
    relative = Matrix.Identity(4)
    current = obj
    while current != link_object:
        if current is None or current.parent is None or current.constraints or current.parent_type != "OBJECT":
            return W_L.inverted() @ obj.matrix_world
        relative = current.matrix_parent_inverse @ current.matrix_basis @ relative
        current = current.parent
    return relative


def _geometry(item, link_object, W_L, prefix, depsgraph, scale, issues):
    """Return (shape, T_L_G), or None after recording a blocking issue."""
    obj = item.object
    if obj is None:
        issues.append(("missing_object", item.uid, f"{item.name} has no scene object.",
                       "Assign an object or remove this entry."))
        return None
    relative = _relative_to_link(obj, link_object, W_L)
    linear = _mat3(relative)
    report = classify_linear(linear, CLASSIFY_TOL)
    translation = tuple(si(relative[i][3], scale) for i in range(3))

    if report.kind is AffineKind.SINGULAR:
        issues.append(("invalid_transform", item.uid, f"{item.name} is flattened to zero thickness.",
                       "Select the object and fix its scale."))
        return None

    deformed = any(m.show_viewport for m in obj.modifiers)
    if item.shape != "MESH" and not deformed:
        shape, reason = _analytic(item, report, scale)
        if shape is not None:
            return shape, Transform(orthonormalized(report.rotation), translation)
    elif item.shape != "MESH":
        reason = "it has active modifiers that change its shape"
    else:
        reason = ""

    if item.shape != "MESH" and not item.allow_mesh_fallback:
        issues.append(("non_analytic_shape", item.uid,
                       f"{item.name} cannot export as a {item.shape.lower()} because {reason}.",
                       "Remove the deformation or enable Export as Mesh if Deformed."))
        return None
    if obj.type != "MESH":
        issues.append(("missing_object", item.uid, f"{item.name} needs a mesh object.", "Assign a mesh object."))
        return None
    asset = _asset_name(prefix, item.name, item.uid)
    no_translation = relative.copy()
    for i in range(3):
        no_translation[i][3] = 0.0
    return _mesh_shape(obj, no_translation, depsgraph, scale, asset), Transform(t=translation)


def _component(comp, link_object, W_L, module, depsgraph, scale, issues):
    geometry = _geometry(comp, link_object, W_L, module.prefix, depsgraph, scale, issues)
    visuals, collisions = [], []
    if geometry is not None:
        shape, origin = geometry
        if comp.export_visual:
            color = tuple(round(float(v), 4) for v in comp.color)
            material = X.Material(re.sub(r"[^A-Za-z0-9_]", "_", f"{comp.name}_color"), color)
            visuals.append(X.GeometryElement(comp.uid, f"{comp.name}_visual", shape, origin, material))
        if comp.use_as_collision:
            collisions.append(X.GeometryElement(comp.uid, f"{comp.name}_collision", shape, origin))
    for proxy in comp.collisions:
        proxy_geometry = _geometry(proxy, link_object, W_L, module.prefix, depsgraph, scale, issues)
        if proxy_geometry is not None:
            collisions.append(X.GeometryElement(proxy.uid, proxy.name, proxy_geometry[0], proxy_geometry[1]))

    mass = None
    if comp.mass_mode != "none":
        origin = geometry[1] if geometry is not None else Transform()
        distribution = geometry[0] if geometry is not None and not isinstance(geometry[0], X.Mesh) else None
        mode = X.MassMode(comp.mass_mode)
        if mode is X.MassMode.MANUAL:
            com = tuple(si(v, scale) for v in comp.manual_com)
            mass = X.MassSpec(
                mode, Transform(origin.R, origin.apply(com)), mass=float(comp.mass),
                manual_inertia=Inertia(comp.ixx, comp.ixy, comp.ixz, comp.iyy, comp.iyz, comp.izz),
                assumption="Manual values",
            )
        else:
            mass = X.MassSpec(
                mode, origin, shape=distribution,
                mass=float(comp.mass) if mode is not X.MassMode.DENSITY else None,
                density=float(comp.density) if mode is X.MassMode.DENSITY else None,
                assumption=comp.assumption,
            )
    return X.Component(comp.uid, comp.name, mass, visuals, collisions, comp.overlap_confirmed)


def _joint(joint, scale):
    linear = joint.type == "prismatic"
    factor = (lambda v: si(v, scale)) if linear else float
    limit = X.JointLimit(
        lower=factor(joint.lower) if joint.has_position_limits else None,
        upper=factor(joint.upper) if joint.has_position_limits else None,
        velocity=float(joint.velocity) if joint.has_velocity else None,
        effort=float(joint.effort) if joint.has_effort else None,
    )
    return X.Joint(
        id=joint.uid, name=joint.name, type=X.JointType(joint.type),
        parent_id=joint.parent_uid, child_id=joint.child_uid,
        origin=_pose(joint.origin, scale), axis=tuple(float(v) for v in joint.axis),
        limit=limit, damping=float(joint.damping), friction=float(joint.friction),
        actuation=X.Actuation(joint.actuation), role=X.ControlRole(joint.role), drive=_drive(joint),
    )


def _drive(joint):
    if joint.drive_kind == "can":
        return X.MotorDrive(X.DriveKind.CAN, joint.motor or "P20 or P17", node_id=joint.node_id or None,
                            gear_ratio=float(joint.gear_ratio))
    if joint.drive_kind == "roboclaw":
        return X.MotorDrive(
            X.DriveKind.ROBOCLAW, joint.motor or "goBILDA 5203 19.2:1", gear_ratio=float(joint.gear_ratio),
            address=joint.roboclaw_address if joint.roboclaw_address >= 128 else None,
            channel=None if joint.roboclaw_channel == "none" else joint.roboclaw_channel,
            qppr=joint.qppr or None,
        )
    return None


def extract_project(scene, depsgraph=None):
    """Return (core project, extraction issues). The scene is not modified."""
    data = scene.rab
    scale = float(scene.unit_settings.scale_length)
    depsgraph = depsgraph or bpy.context.evaluated_depsgraph_get()
    issues = []
    project = X.Project(name=data.robot_name)

    for module in data.modules:
        links = []
        for link in module.links:
            if link.frame_object is None:
                issues.append(("missing_object", link.uid, f"Link {link.name} lost its frame object.",
                               "Recreate the link frame."))
                links.append(X.Link(link.uid, link.name))
                continue
            world = link.frame_object.matrix_world
            kind = classify_linear(_mat3(world), CLASSIFY_TOL).kind
            if kind is not AffineKind.RIGID:
                issues.append(("link_frame_scaled", link.uid,
                               f"Link frame {link.name} has {kind.value.replace('_', ' ')}. URDF frames must be rigid.",
                               "Select the link frame and reset its scale to 1."))
            W_L = rigid(world)
            components = [
                _component(c, link.frame_object, W_L, module, depsgraph, scale, issues) for c in link.components
            ]
            links.append(X.Link(link.uid, link.name, components))

        frames = [
            X.AttachmentFrame(f.uid, f.name, f.link_uid, _pose(f.pose, scale), X.FrameKind(f.kind))
            for f in module.frames
        ]
        linear = {j.uid for j in module.joints if j.type == "prismatic"}
        poses = {
            named.name: {
                v.joint_uid: si(v.value, scale) if v.joint_uid in linear else float(v.value)
                for v in named.values
            }
            for named in module.poses
        }
        project.modules.append(X.Module(
            id=module.uid, name=module.name, kind=X.ModuleKind(module.kind), prefix=module.prefix,
            root_link_id=module.root_link_uid, links=links,
            joints=[_joint(j, scale) for j in module.joints], frames=frames, poses=poses,
        ))

    for mount in data.mounts:
        project.mounts.append(X.Mount(
            mount.uid, mount.parent_module_uid, mount.parent_frame_uid,
            mount.child_module_uid, mount.child_frame_uid, _pose(mount.offset, scale),
        ))
    return project, issues


def validate_scene(scene, module_ids=None, name=None):
    """Extract and validate. Extraction problems are merged as blocking issues."""
    project, issues = extract_project(scene)
    report = validate_project(project, module_ids, name)
    selected = None if module_ids is None else set(module_ids)
    owners = _issue_owners(scene)
    for code, entity, message, action in issues:
        if selected is None or owners.get(entity) in selected:
            report.add(code, entity, message, action, blocks=ALL_TARGETS)
    return project, report


def _issue_owners(scene):
    owners = {}
    for module in scene.rab.modules:
        for link in module.links:
            owners[link.uid] = module.uid
            for component in link.components:
                owners[component.uid] = module.uid
                for proxy in component.collisions:
                    owners[proxy.uid] = module.uid
    return owners


__all__ = ["extract_project", "validate_scene", "si", "find", "Matrix", "ValidationReport"]

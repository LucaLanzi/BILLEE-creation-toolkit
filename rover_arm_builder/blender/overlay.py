"""Viewport overlays: frames, joint axes, limits, COM markers, and collision shapes.

The collision overlay is a display aid. It performs no collision testing.
"""

import math

import blf
import bpy
import gpu
from bpy_extras import view3d_utils
from gpu_extras.batch import batch_for_shader
from mathutils import Matrix, Vector

from . import authoring as A

COLORS = {"x": (0.95, 0.25, 0.25, 1), "y": (0.35, 0.85, 0.3, 1), "z": (0.3, 0.5, 1.0, 1)}
JOINT = (1.0, 0.85, 0.15, 1)
LIMIT = (1.0, 0.55, 0.1, 1)
COM = (0.95, 0.3, 0.9, 1)
COLLISION = (0.1, 0.8, 0.9, 0.18)

_handles = []
_mesh_cache = {}


def _axes(lines, matrix, size):
    origin = matrix.to_translation()
    rotation = matrix.to_3x3().normalized()
    for index, key in enumerate("xyz"):
        lines.setdefault(COLORS[key], []).extend([origin, origin + rotation.col[index] * size])


def _arrow(lines, start, direction, length, color):
    end = start + direction * length
    side = direction.orthogonal().normalized() * length * 0.12
    other = direction.cross(side).normalized() * length * 0.12
    base = end - direction * length * 0.2
    lines.setdefault(color, []).extend([start, end, end, base + side, end, base - side, end, base + other, end, base - other])


def _joint_guides(lines, project, module, size):
    for joint in module.joints:
        if joint.type == "fixed":
            continue
        parent = A.find(module.links, joint.parent_uid)
        if parent is None or parent.frame_object is None:
            continue
        axis = Vector(joint.axis)
        if axis.length < 1e-9:
            continue
        axis.normalize()
        world = A.rigid(parent.frame_object.matrix_world) @ A.pose_matrix(joint.origin)
        origin = world.to_translation()
        rotation = world.to_3x3()
        world_axis = (rotation @ axis).normalized()
        if project.show_axes:
            _arrow(lines, origin, world_axis, size * 1.6, JOINT)
        if not project.show_limits or not joint.has_position_limits:
            continue
        if joint.type == "revolute":
            start = (rotation @ axis.orthogonal()).normalized() * size
            steps = max(6, int(abs(joint.upper - joint.lower) / 0.1))
            points = [
                origin + Matrix.Rotation(joint.lower + (joint.upper - joint.lower) * i / steps, 3, world_axis) @ start
                for i in range(steps + 1)
            ]
            arc = lines.setdefault(LIMIT, [])
            arc.extend([origin, points[0], origin, points[-1]])
            for a, b in zip(points, points[1:]):
                arc.extend([a, b])
        elif joint.type == "prismatic":
            lines.setdefault(LIMIT, []).extend([origin + world_axis * joint.lower, origin + world_axis * joint.upper])


def _com_markers(lines, module, scale, size):
    for link in module.links:
        total, weighted = 0.0, Vector((0, 0, 0))
        for component in link.components:
            mass = A.component_mass_kg(component, scale)
            com = A.component_com_world(component)
            if mass <= 0.0 or com is None:
                continue
            total += mass
            weighted += com * mass
            _cross(lines, com, size * 0.35)
        if total > 0.0:
            _cross(lines, weighted / total, size * 0.8)


def _cross(lines, point, size):
    target = lines.setdefault(COM, [])
    for axis in (Vector((1, 0, 0)), Vector((0, 1, 0)), Vector((0, 0, 1))):
        target.extend([point - axis * size, point + axis * size])


def _collision_batch(shader, obj):
    mesh = obj.data
    key = (mesh.as_pointer(), len(mesh.vertices), len(mesh.polygons), A.mesh_version())
    cached = _mesh_cache.get(mesh.as_pointer())
    if cached is not None and cached[0] == key:
        return cached[1]
    mesh.calc_loop_triangles()
    positions = [tuple(v.co) for v in mesh.vertices]
    indices = [tuple(t.vertices) for t in mesh.loop_triangles]
    batch = batch_for_shader(shader, "TRIS", {"pos": positions}, indices=indices)
    _mesh_cache[mesh.as_pointer()] = (key, batch)
    return batch


def _draw_collision(project, shader):
    gpu.state.blend_set("ALPHA")
    gpu.state.depth_mask_set(False)
    gpu.state.depth_test_set("LESS_EQUAL")
    shader.bind()
    shader.uniform_float("color", COLLISION)
    for module in project.modules:
        for link in module.links:
            for component in link.components:
                items = [p.object for p in component.collisions]
                if component.use_as_collision:
                    items.append(component.object)
                for obj in items:
                    if obj is None or obj.type != "MESH" or not obj.visible_get():
                        continue
                    with gpu.matrix.push_pop():
                        gpu.matrix.multiply_matrix(obj.matrix_world)
                        _collision_batch(shader, obj).draw(shader)
    gpu.state.depth_mask_set(True)
    gpu.state.depth_test_set("NONE")
    gpu.state.blend_set("NONE")


def draw_3d():
    context = bpy.context
    scene = context.scene
    if scene is None or not hasattr(scene, "rab") or not len(scene.rab.modules):
        return
    project = scene.rab
    scale = scene.unit_settings.scale_length or 1.0
    size = 0.06 / scale
    shader = gpu.shader.from_builtin("UNIFORM_COLOR")
    if project.show_collision and project.display_mode != "visual":
        _draw_collision(project, shader)

    lines = {}
    for module in project.modules:
        if project.show_frames:
            for link in module.links:
                if link.frame_object is not None:
                    _axes(lines, link.frame_object.matrix_world, size)
            for frame in module.frames:
                if frame.object is not None:
                    _axes(lines, frame.object.matrix_world, size * 0.8)
        if project.show_axes or project.show_limits:
            _joint_guides(lines, project, module, size)
        if project.show_com:
            _com_markers(lines, module, scale, size)

    gpu.state.line_width_set(2.0)
    shader.bind()
    for color, points in lines.items():
        if points:
            shader.uniform_float("color", color)
            batch_for_shader(shader, "LINES", {"pos": [tuple(p) for p in points]}).draw(shader)
    gpu.state.line_width_set(1.0)


def draw_labels():
    context = bpy.context
    scene = context.scene
    region = context.region
    view = context.region_data
    if scene is None or view is None or not hasattr(scene, "rab") or not scene.rab.show_frames:
        return
    font = 0
    blf.size(font, 11)
    blf.color(font, 1, 1, 1, 0.9)
    for module in scene.rab.modules:
        entries = [(l.name, l.frame_object) for l in module.links] + [(f.name, f.object) for f in module.frames]
        for name, obj in entries:
            if obj is None:
                continue
            point = view3d_utils.location_3d_to_region_2d(region, view, obj.matrix_world.to_translation())
            if point is not None:
                blf.position(font, point.x + 6, point.y + 4, 0)
                blf.draw(font, f"{module.prefix}{name}")


def register():
    if _handles:
        return
    space = bpy.types.SpaceView3D
    _handles.append(space.draw_handler_add(draw_3d, (), "WINDOW", "POST_VIEW"))
    _handles.append(space.draw_handler_add(draw_labels, (), "WINDOW", "POST_PIXEL"))


def unregister():
    space = bpy.types.SpaceView3D
    while _handles:
        space.draw_handler_remove(_handles.pop(), "WINDOW")
    _mesh_cache.clear()


__all__ = ["register", "unregister", "math"]

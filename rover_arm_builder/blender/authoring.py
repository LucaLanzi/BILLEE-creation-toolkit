"""Scene authoring services. These use direct data access, not context-sensitive operators."""

import math

import bmesh
import bpy
from mathutils import Euler, Matrix, Vector

from .props import new_uid

_syncing = False


class _Guard:
    """Stops property updates and scene writes from re-triggering each other."""

    def __enter__(self):
        global _syncing
        self.previous = _syncing
        _syncing = True

    def __exit__(self, *exc):
        global _syncing
        _syncing = self.previous


def find(items, uid):
    return next((item for item in items if item.uid == uid), None)


def pose_matrix(pose):
    return Matrix.Translation(Vector(pose.location)) @ Euler(pose.rotation, "XYZ").to_matrix().to_4x4()


def set_pose(pose, matrix):
    location, rotation, _scale = matrix.decompose()
    with _Guard():
        pose.location = location
        pose.rotation = rotation.to_euler("XYZ")


def rigid(matrix):
    location, rotation, _scale = matrix.decompose()
    return Matrix.LocRotScale(location, rotation, None)


def joint_motion(joint):
    q = joint.preview
    if joint.type in {"revolute", "continuous"}:
        axis = Vector(joint.axis)
        if axis.length < 1e-9:
            return Matrix.Identity(4)
        return Matrix.Rotation(q, 4, axis.normalized())
    if joint.type == "prismatic":
        axis = Vector(joint.axis)
        if axis.length < 1e-9:
            return Matrix.Identity(4)
        return Matrix.Translation(axis.normalized() * q)
    return Matrix.Identity(4)


def _new_empty(name, collection, display="ARROWS", size=0.05):
    obj = bpy.data.objects.new(name, None)
    obj.empty_display_type = display
    obj.empty_display_size = size
    obj.rotation_mode = "XYZ"
    collection.objects.link(obj)
    return obj


def _attach(child, parent, basis=None):
    child.parent = parent
    child.matrix_parent_inverse = Matrix.Identity(4)
    child.matrix_basis = basis if basis is not None else Matrix.Identity(4)


_mesh_version = 0


def mesh_version():
    """Changes whenever a primitive mesh is rebuilt, so display caches can refresh."""
    return _mesh_version


def build_primitive_mesh(mesh, item):
    global _mesh_version
    _mesh_version += 1
    bm = bmesh.new()
    try:
        if item.shape == "BOX":
            bmesh.ops.create_cube(bm, size=1.0, matrix=Matrix.Diagonal((*item.size, 1.0)))
        elif item.shape == "CYLINDER":
            bmesh.ops.create_cone(
                bm, cap_ends=True, cap_tris=False, segments=32,
                radius1=item.radius, radius2=item.radius, depth=item.length,
            )
        elif item.shape == "SPHERE":
            bmesh.ops.create_uvsphere(bm, u_segments=32, v_segments=16, radius=item.radius)
        else:
            return
        bm.to_mesh(mesh)
        mesh.update()
    finally:
        bm.free()


_model_version = 0


def model_version():
    """Changes whenever exportable data may have changed, so old check results are not shown."""
    return _model_version


def touch_model():
    global _model_version
    _model_version += 1


def on_preview_changed(scene):
    """Preview moves objects only. It does not change the model, so checks stay current."""
    if not _syncing and isinstance(scene, bpy.types.Scene):
        apply_all(scene)


def on_shape_changed(item):
    touch_model()
    if _syncing:
        return
    obj = item.object
    if obj is not None and obj.type == "MESH" and item.shape != "MESH":
        build_primitive_mesh(obj.data, item)
    if getattr(item, "mass_mode", "") == "manual":
        with _Guard():
            item.needs_review = True


def on_metadata_changed(scene):
    touch_model()
    if _syncing or not isinstance(scene, bpy.types.Scene):
        return
    apply_all(scene)


def apply_module(module):
    """Write metadata poses to objects. Preview affects objects only, never metadata."""
    with _Guard():
        for joint in module.joints:
            child = find(module.links, joint.child_uid)
            if child is None or child.frame_object is None:
                continue
            target = pose_matrix(joint.origin) @ joint_motion(joint)
            if not _same(child.frame_object.matrix_basis, target):
                child.frame_object.matrix_basis = target
        for frame in module.frames:
            if frame.object is not None:
                target = pose_matrix(frame.pose)
                if not _same(frame.object.matrix_basis, target):
                    frame.object.matrix_basis = target


def _same(a, b, tol=1e-6):
    return all(abs(a[r][c] - b[r][c]) <= tol for r in range(4) for c in range(4))


def module_frame_in_root(module, frame):
    """T_root_frame at the zero pose, from metadata, ignoring any active preview."""
    by_child = {j.child_uid: j for j in module.joints}
    pose = pose_matrix(frame.pose)
    current = frame.link_uid
    seen = set()
    while current != module.root_link_uid and current not in seen:
        seen.add(current)
        joint = by_child.get(current)
        if joint is None:
            break
        pose = pose_matrix(joint.origin) @ pose
        current = joint.parent_uid
    return pose


def apply_mounts(scene):
    project = scene.rab
    with _Guard():
        for mount in project.mounts:
            parent = find(project.modules, mount.parent_module_uid)
            child = find(project.modules, mount.child_module_uid)
            if parent is None or child is None:
                continue
            parent_frame = find(parent.frames, mount.parent_frame_uid)
            child_frame = find(child.frames, mount.child_frame_uid)
            if parent_frame is None or child_frame is None:
                continue
            owner = find(parent.links, parent_frame.link_uid)
            root = find(child.links, child.root_link_uid)
            if owner is None or root is None or owner.frame_object is None or root.frame_object is None:
                continue
            basis = pose_matrix(parent_frame.pose) @ pose_matrix(mount.offset) @ module_frame_in_root(child, child_frame).inverted()
            if root.frame_object.parent != owner.frame_object:
                root.frame_object.parent = owner.frame_object
                root.frame_object.matrix_parent_inverse = Matrix.Identity(4)
            if not _same(root.frame_object.matrix_basis, basis):
                root.frame_object.matrix_basis = basis


def apply_all(scene):
    for module in scene.rab.modules:
        apply_module(module)
    apply_mounts(scene)


def is_previewing(module):
    return any(abs(j.preview) > 1e-9 for j in module.joints)


def pull_from_objects(scene):
    """Copy viewport edits into metadata. Skipped for a module while it is being previewed."""
    if _syncing:
        return False
    changed = False
    for module in scene.rab.modules:
        if is_previewing(module):
            continue
        for joint in module.joints:
            child = find(module.links, joint.child_uid)
            if child is None or child.frame_object is None:
                continue
            if not _same(child.frame_object.matrix_basis, pose_matrix(joint.origin), 1e-5):
                set_pose(joint.origin, child.frame_object.matrix_basis)
                changed = True
        for frame in module.frames:
            if frame.object is not None and not _same(frame.object.matrix_basis, pose_matrix(frame.pose), 1e-5):
                set_pose(frame.pose, frame.object.matrix_basis)
                changed = True
    return changed


def create_module(scene, name, kind, prefix):
    project = scene.rab
    collection = bpy.data.collections.new(f"RAB {name}")
    scene.collection.children.link(collection)
    module = project.modules.add()
    module.uid = new_uid()
    module.name = name
    module.kind = kind
    module.prefix = prefix
    module.collection = collection
    root = _add_link_record(module, "base")
    module.root_link_uid = root.uid
    project.active_module = len(project.modules) - 1
    return module


def _add_link_record(module, name):
    link = module.links.add()
    link.uid = new_uid()
    link.name = name
    empty = _new_empty(f"{module.prefix}{name}", module.collection)
    empty.rab.uid = link.uid
    empty.rab.kind = "LINK"
    link.frame_object = empty
    module.active_link = len(module.links) - 1
    return link


def add_link(module, name, parent, joint_name, joint_type="fixed", origin=None, axis=(0, 0, 1)):
    parent_uid = parent.uid
    link = _add_link_record(module, name)
    parent = find(module.links, parent_uid)
    joint = module.joints.add()
    joint.uid = new_uid()
    joint.name = joint_name
    with _Guard():
        joint.type = joint_type
        joint.parent_uid = parent.uid
        joint.child_uid = link.uid
        joint.axis = axis
        joint.actuation = "fixed" if joint_type == "fixed" else "actuated"
    _attach(link.frame_object, parent.frame_object)
    if origin is not None:
        set_pose(joint.origin, origin)
    apply_module(module)
    module.active_joint = len(module.joints) - 1
    return link, joint


def _configure_shape(item, shape, size=None, radius=None, length=None):
    with _Guard():
        item.shape = shape
        if size is not None:
            item.size = size
        if radius is not None:
            item.radius = radius
        if length is not None:
            item.length = length


def add_component(module, link, name, shape="BOX", size=None, radius=None, length=None, local=None, obj=None):
    """Add a component. Pass `obj` to adopt existing geometry, which keeps its world placement."""
    component = link.components.add()
    component.uid = new_uid()
    component.name = name
    _configure_shape(component, shape, size, radius, length)
    if obj is None:
        mesh = bpy.data.meshes.new(f"{module.prefix}{name}")
        obj = bpy.data.objects.new(f"{module.prefix}{name}", mesh)
        module.collection.objects.link(obj)
        build_primitive_mesh(mesh, component)
        _attach(obj, link.frame_object, local)
    else:
        world = obj.matrix_world.copy()
        obj.parent = link.frame_object
        obj.matrix_parent_inverse = Matrix.Identity(4)
        bpy.context.view_layer.update()
        obj.matrix_world = world
    obj.rab.uid = component.uid
    obj.rab.kind = "COMPONENT"
    component.object = obj
    link.active_component = len(link.components) - 1
    return component


def add_collision_proxy(module, link, component, name, shape="BOX", size=None, radius=None, length=None, local=None):
    proxy = component.collisions.add()
    proxy.uid = new_uid()
    proxy.name = name
    _configure_shape(proxy, shape, size, radius, length)
    mesh = bpy.data.meshes.new(f"{module.prefix}{name}")
    obj = bpy.data.objects.new(f"{module.prefix}{name}", mesh)
    module.collection.objects.link(obj)
    build_primitive_mesh(mesh, proxy)
    obj.display_type = "WIRE"
    obj.hide_render = True
    _attach(obj, link.frame_object, local)
    obj.rab.uid = proxy.uid
    obj.rab.kind = "COLLISION"
    proxy.object = obj
    component.use_as_collision = False
    return proxy


def add_frame(module, link, name, kind="mount", pose=None):
    frame = module.frames.add()
    frame.uid = new_uid()
    frame.name = name
    frame.link_uid = link.uid
    with _Guard():
        frame.kind = kind
    empty = _new_empty(f"{module.prefix}{name}", module.collection, "ARROWS", 0.04)
    empty.rab.uid = frame.uid
    empty.rab.kind = "FRAME"
    frame.object = empty
    _attach(empty, link.frame_object)
    if pose is not None:
        set_pose(frame.pose, pose)
    apply_module(module)
    module.active_frame = len(module.frames) - 1
    return frame


def add_mount(scene, parent_module, parent_frame, child_module, child_frame, offset=None):
    project = scene.rab
    if parent_module.uid == child_module.uid:
        raise ValueError("A module cannot be mounted to itself.")
    if any(m.child_module_uid == child_module.uid for m in project.mounts):
        raise ValueError(f"{child_module.name} is already mounted. Detach it first.")
    ancestor = parent_module.uid
    parents = {m.child_module_uid: m.parent_module_uid for m in project.mounts}
    while ancestor in parents:
        ancestor = parents[ancestor]
        if ancestor == child_module.uid:
            raise ValueError("This mount would create a cycle between modules.")
    if ancestor == child_module.uid:
        raise ValueError("This mount would create a cycle between modules.")
    mount = project.mounts.add()
    mount.uid = new_uid()
    mount.parent_module_uid = parent_module.uid
    mount.parent_frame_uid = parent_frame.uid
    mount.child_module_uid = child_module.uid
    mount.child_frame_uid = child_frame.uid
    if offset is not None:
        set_pose(mount.offset, offset)
    apply_mounts(scene)
    return mount


def detach_mount(scene, mount_uid):
    """Remove a mount. The child module keeps its world placement and internal joints."""
    project = scene.rab
    index = next((i for i, m in enumerate(project.mounts) if m.uid == mount_uid), None)
    if index is None:
        return False
    mount = project.mounts[index]
    child = find(project.modules, mount.child_module_uid)
    root = find(child.links, child.root_link_uid) if child else None
    if root is not None and root.frame_object is not None:
        bpy.context.view_layer.update()
        world = root.frame_object.matrix_world.copy()
        with _Guard():
            root.frame_object.parent = None
            root.frame_object.matrix_parent_inverse = Matrix.Identity(4)
            root.frame_object.matrix_basis = world
    project.mounts.remove(index)
    return True


def reset_preview(module):
    with _Guard():
        for joint in module.joints:
            joint.preview = 0.0
    apply_module(module)


def apply_named_pose(module, pose_name):
    named = module.poses.get(pose_name)
    if named is None:
        return False
    values = {v.joint_uid: v.value for v in named.values}
    with _Guard():
        for joint in module.joints:
            joint.preview = values.get(joint.uid, 0.0)
    apply_module(module)
    return True


def store_named_pose(module, pose_name):
    named = module.poses.get(pose_name)
    if named is None:
        named = module.poses.add()
        named.name = pose_name
    named.values.clear()
    for joint in module.joints:
        if joint.type != "fixed":
            item = named.values.add()
            item.joint_uid = joint.uid
            item.value = joint.preview
    return named


def reframe_link(scene, module, link, new_world):
    """Move a link frame while keeping all geometry, child joints, frames, and COM in place."""
    bpy.context.view_layer.update()
    old_world = rigid(link.frame_object.matrix_world)
    target = rigid(new_world)
    delta = old_world.inverted() @ target
    inverse = delta.inverted()

    captured = []
    for component in link.components:
        for item in (component, *component.collisions):
            if item.object is not None:
                captured.append((item.object, item.object.matrix_world.copy()))
    mounted = any(m.child_module_uid == module.uid for m in scene.rab.mounts)

    with _Guard():
        for joint in module.joints:
            if joint.child_uid == link.uid:
                set_pose(joint.origin, pose_matrix(joint.origin) @ delta)
            elif joint.parent_uid == link.uid:
                set_pose(joint.origin, inverse @ pose_matrix(joint.origin))
        for frame in module.frames:
            if frame.link_uid == link.uid:
                set_pose(frame.pose, inverse @ pose_matrix(frame.pose))
        if link.uid == module.root_link_uid and not mounted:
            link.frame_object.matrix_world = target
    apply_all(scene)
    bpy.context.view_layer.update()
    for obj, world in captured:
        obj.matrix_world = world
    bpy.context.view_layer.update()


def select_link_objects(view_layer, link):
    """Highlight a link's scene objects and make its first body active."""
    with _Guard():
        for obj in view_layer.objects:
            if obj.select_get(view_layer=view_layer):
                obj.select_set(False, view_layer=view_layer)
        objects = [c.object for c in link.components if c.object is not None]
        if link.frame_object is not None:
            objects.append(link.frame_object)
        visible = [o for o in objects if o.name in view_layer.objects]
        for obj in visible:
            obj.select_set(True, view_layer=view_layer)
        if visible:
            view_layer.objects.active = visible[0]


def on_active_link_changed(module, context):
    if _syncing or bpy.app.background or not 0 <= module.active_link < len(module.links):
        return
    link = module.links[module.active_link]
    owner = owner_of_object(context.scene, context.view_layer.objects.active)
    if owner is not None and owner[1] is not None and owner[1].uid == link.uid:
        return
    select_link_objects(context.view_layer, link)


def _preview_records(scene):
    project = scene.rab
    parent = find(project.modules, project.mount_parent_module)
    child = find(project.modules, project.mount_child_module)
    if parent is None or child is None or parent.uid == child.uid:
        return None
    parent_frame = find(parent.frames, project.mount_parent_frame)
    child_frame = find(child.frames, project.mount_child_frame)
    root = find(child.links, child.root_link_uid)
    if parent_frame is None or child_frame is None or root is None or root.frame_object is None:
        return None
    return parent, parent_frame, child, child_frame, root


def mount_offset_matrix(project):
    return (Matrix.Translation(Vector(project.mount_offset_location))
            @ Euler(project.mount_offset_rotation, "XYZ").to_matrix().to_4x4())


def apply_mount_preview(scene):
    """Show the pending alignment. Nothing is mounted until it is confirmed."""
    project = scene.rab
    if _syncing or not project.mount_preview_active:
        return False
    records = _preview_records(scene)
    if records is None:
        return False
    _parent, parent_frame, child, child_frame, root = records
    bpy.context.view_layer.update()
    world = rigid(parent_frame.object.matrix_world) @ mount_offset_matrix(project) @ module_frame_in_root(child, child_frame).inverted()
    with _Guard():
        root.frame_object.matrix_world = world
    return True


def begin_mount_preview(scene):
    project = scene.rab
    records = _preview_records(scene)
    if records is None:
        raise ValueError("Choose a parent module, a child module, and a frame on each.")
    if any(m.child_module_uid == records[2].uid for m in project.mounts):
        raise ValueError(f"{records[2].name} is already mounted. Detach it first.")
    bpy.context.view_layer.update()
    saved = records[4].frame_object.matrix_world.copy()
    with _Guard():
        project.mount_saved_matrix = saved
        project.mount_preview_active = True
    apply_mount_preview(scene)


def cancel_mount_preview(scene):
    project = scene.rab
    if not project.mount_preview_active:
        return False
    records = _preview_records(scene)
    with _Guard():
        if records is not None:
            records[4].frame_object.matrix_world = Matrix(project.mount_saved_matrix)
        project.mount_preview_active = False
    return True


def confirm_mount_preview(scene):
    project = scene.rab
    records = _preview_records(scene)
    if not project.mount_preview_active or records is None:
        raise ValueError("Start an alignment preview first.")
    parent, parent_frame, child, child_frame, _root = records
    offset = mount_offset_matrix(project)
    with _Guard():
        project.mount_preview_active = False
    try:
        return add_mount(scene, parent, parent_frame, child, child_frame, offset)
    except ValueError:
        with _Guard():
            project.mount_preview_active = True
        raise


def component_mass_kg(component, scale):
    """Mass used for on-screen summaries. Export recomputes it through the core model."""
    if component.mass_mode == "none":
        return 0.0
    if component.mass_mode != "density":
        return component.mass
    obj = component.object
    sx, sy, sz = obj.matrix_world.to_scale() if obj is not None else (1.0, 1.0, 1.0)
    if component.shape == "BOX":
        volume = component.size[0] * sx * component.size[1] * sy * component.size[2] * sz
    elif component.shape == "CYLINDER":
        volume = math.pi * (component.radius * sx) ** 2 * component.length * sz
    elif component.shape == "SPHERE":
        volume = 4.0 / 3.0 * math.pi * (component.radius * sx) ** 3
    else:
        return 0.0
    return component.density * volume * scale**3


def component_com_world(component):
    obj = component.object
    if obj is None:
        return None
    if component.mass_mode == "manual":
        return rigid(obj.matrix_world) @ Vector(component.manual_com)
    return obj.matrix_world.to_translation()


def mass_summary(modules, scale):
    """Return (total kg, world COM or None) for the current displayed pose."""
    total, weighted = 0.0, Vector((0.0, 0.0, 0.0))
    for module in modules:
        for link in module.links:
            for component in link.components:
                mass = component_mass_kg(component, scale)
                com = component_com_world(component)
                if mass > 0.0 and com is not None:
                    total += mass
                    weighted += com * mass
    return total, (weighted / total if total > 0.0 else None)


def apply_display(scene):
    """Switch between visual and collision display without touching geometry."""
    mode = scene.rab.display_mode
    layer = bpy.context.view_layer
    for module in scene.rab.modules:
        for link in module.links:
            for component in link.components:
                obj = component.object
                if obj is not None and obj.name in layer.objects:
                    obj.hide_set(mode == "collision" and not component.use_as_collision)
                    obj.display_type = "WIRE" if mode == "collision" else "TEXTURED"
                for proxy in component.collisions:
                    if proxy.object is not None and proxy.object.name in layer.objects:
                        proxy.object.hide_set(mode == "visual")


def apply_new_zero(module, joint):
    """Make the joint's current preview the new zero, keeping physical limits in place."""
    offset = joint.preview
    if joint.type == "fixed" or abs(offset) < 1e-12:
        return False
    new_origin = pose_matrix(joint.origin) @ joint_motion(joint)
    set_pose(joint.origin, new_origin)
    with _Guard():
        if joint.has_position_limits:
            joint.lower -= offset
            joint.upper -= offset
        for named in module.poses:
            for value in named.values:
                if value.joint_uid == joint.uid:
                    value.value -= offset
        joint.preview = 0.0
    apply_module(module)
    return True


def remove_component(link, index):
    component = link.components[index]
    objects = [item.object for item in (component, *component.collisions) if item.object is not None]
    link.components.remove(index)
    link.active_component = max(0, min(link.active_component, len(link.components) - 1))
    for obj in objects:
        bpy.data.objects.remove(obj)


def remove_leaf_link(scene, module, link_uid):
    """Remove a link that has no children, with its incoming joint, frames, and objects."""
    if link_uid == module.root_link_uid:
        raise ValueError("The root link cannot be removed.")
    if any(j.parent_uid == link_uid for j in module.joints):
        raise ValueError("Remove this link's child links first.")
    frame_uids = {f.uid for f in module.frames if f.link_uid == link_uid}
    if any(m.parent_frame_uid in frame_uids or m.child_frame_uid in frame_uids for m in scene.rab.mounts):
        raise ValueError("A mount uses a frame on this link. Detach it first.")
    for index in reversed(range(len(module.frames))):
        if module.frames[index].link_uid == link_uid:
            if module.frames[index].object is not None:
                bpy.data.objects.remove(module.frames[index].object)
            module.frames.remove(index)
    for index in reversed(range(len(module.joints))):
        if module.joints[index].child_uid == link_uid:
            joint_uid = module.joints[index].uid
            module.joints.remove(index)
            for named in module.poses:
                for value_index in reversed(range(len(named.values))):
                    if named.values[value_index].joint_uid == joint_uid:
                        named.values.remove(value_index)
    link_index = next(i for i, l in enumerate(module.links) if l.uid == link_uid)
    link = module.links[link_index]
    while len(link.components):
        remove_component(link, len(link.components) - 1)
    if link.frame_object is not None:
        bpy.data.objects.remove(link.frame_object)
    module.links.remove(link_index)
    module.active_link = max(0, min(module.active_link, len(module.links) - 1))
    module.active_joint = max(0, min(module.active_joint, len(module.joints) - 1))
    module.active_frame = max(0, min(module.active_frame, len(module.frames) - 1))


def unique_prefix(project, base):
    used = {m.prefix for m in project.modules}
    stem = base.rstrip("_") or "module"
    index = 2
    while f"{stem}{index}_" in used:
        index += 1
    return f"{stem}{index}_"


def duplicate_module(scene, source_index):
    """Create an editable local copy with fresh IDs, a unique prefix, and new objects."""
    project = scene.rab
    source = project.modules[source_index]
    prefix = unique_prefix(project, source.prefix)
    name = f"{source.name} {prefix.rstrip('_')}"
    collection = bpy.data.collections.new(f"RAB {name}")
    scene.collection.children.link(collection)

    ids = {}

    def fresh(old):
        if not old:
            return old
        return ids.setdefault(old, new_uid())

    copies = {}

    def clone(obj):
        if obj is None:
            return None
        duplicate = obj.copy()
        if obj.data is not None:
            duplicate.data = obj.data.copy()
        duplicate.name = f"{prefix}{obj.name.removeprefix(source.prefix)}"
        duplicate.rab.uid = fresh(obj.rab.uid)
        collection.objects.link(duplicate)
        copies[obj] = duplicate
        return duplicate

    module = project.modules.add()
    source = project.modules[source_index]
    module.uid, module.name, module.kind, module.prefix = new_uid(), name, source.kind, prefix
    module.collection = collection
    module.root_link_uid = fresh(source.root_link_uid)

    def copy_shape(target, origin):
        with _Guard():
            target.name, target.shape, target.size = origin.name, origin.shape, origin.size
            target.radius, target.length = origin.radius, origin.length
            target.allow_mesh_fallback = origin.allow_mesh_fallback
        target.uid = fresh(origin.uid)
        target.object = clone(origin.object)

    for old_link in source.links:
        link = module.links.add()
        link.uid, link.name = fresh(old_link.uid), old_link.name
        link.frame_object = clone(old_link.frame_object)
        for old in old_link.components:
            component = link.components.add()
            copy_shape(component, old)
            with _Guard():
                for key in ("export_visual", "use_as_collision", "color", "mass_mode", "mass", "density", "assumption",
                            "overlap_confirmed", "needs_review", "manual_com", "ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
                    setattr(component, key, getattr(old, key))
            for old_proxy in old.collisions:
                copy_shape(component.collisions.add(), old_proxy)

    for old in source.joints:
        joint = module.joints.add()
        joint.uid, joint.name = fresh(old.uid), old.name
        joint.parent_uid, joint.child_uid = fresh(old.parent_uid), fresh(old.child_uid)
        with _Guard():
            for key in ("type", "axis", "has_position_limits", "lower", "upper", "has_velocity", "velocity",
                        "has_effort", "effort", "damping", "friction", "actuation", "role"):
                setattr(joint, key, getattr(old, key))
            joint.origin.location, joint.origin.rotation = old.origin.location, old.origin.rotation

    for old in source.frames:
        frame = module.frames.add()
        frame.uid, frame.name, frame.link_uid = fresh(old.uid), old.name, fresh(old.link_uid)
        with _Guard():
            frame.kind = old.kind
            frame.pose.location, frame.pose.rotation = old.pose.location, old.pose.rotation
        frame.object = clone(old.object)

    for old in source.poses:
        named = module.poses.add()
        named.name = old.name
        for value in old.values:
            item = named.values.add()
            item.joint_uid, item.value = fresh(value.joint_uid), value.value

    for old, duplicate in copies.items():
        if old.parent in copies:
            duplicate.parent = copies[old.parent]
        elif old.parent is not None:
            world = old.matrix_world.copy()
            duplicate.parent = None
            duplicate.matrix_parent_inverse = Matrix.Identity(4)
            duplicate.matrix_basis = world
    reset_preview(module)
    project.active_module = len(project.modules) - 1
    return module


def owner_of_object(scene, obj):
    """Return (module, link, component) for an object, verifying its reference is genuine."""
    if obj is None or obj.rab.kind == "NONE" or not obj.rab.uid:
        return None
    for module in scene.rab.modules:
        for link in module.links:
            if link.frame_object == obj:
                return module, link, None
            for component in link.components:
                if component.object == obj:
                    return module, link, component
                for proxy in component.collisions:
                    if proxy.object == obj:
                        return module, link, component
        for frame in module.frames:
            if frame.object == obj:
                return module, find(module.links, frame.link_uid), None
    return None


def degrees(value):
    return math.degrees(value)

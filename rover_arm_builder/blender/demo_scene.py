"""Build the illustrative demo robot as editable Blender data."""

from mathutils import Matrix

from ..core import demo
from ..core import model as X
from . import authoring as A


def to_matrix(pose):
    R, t = pose.R, pose.t
    return Matrix((
        (R[0][0], R[0][1], R[0][2], t[0]),
        (R[1][0], R[1][1], R[1][2], t[1]),
        (R[2][0], R[2][1], R[2][2], t[2]),
        (0.0, 0.0, 0.0, 1.0),
    ))


def _shape_args(shape):
    if isinstance(shape, X.Box):
        return "BOX", {"size": (shape.x, shape.y, shape.z)}
    if isinstance(shape, X.Cylinder):
        return "CYLINDER", {"radius": shape.radius, "length": shape.length}
    return "SPHERE", {"radius": shape.radius}


def _module_by_uid(scene, uid):
    return A.find(scene.rab.modules, uid)


def _build_module(scene, source):
    module = A.create_module(scene, source.name, source.kind.value, source.prefix)
    module_uid = module.uid
    root_source = next(l for l in source.links if l.id == source.root_link_id)
    root = module.links[0]
    root.name = root_source.name
    root.frame_object.name = f"{source.prefix}{root_source.name}"
    link_uids = {root_source.id: root.uid}
    joint_uids = {}

    pending = list(source.joints)
    while pending:
        ready = [j for j in pending if j.parent_id in link_uids]
        if not ready:
            raise ValueError(f"{source.name} is not a connected tree.")
        for item in ready:
            pending.remove(item)
            module = _module_by_uid(scene, module_uid)
            child_source = next(l for l in source.links if l.id == item.child_id)
            parent = A.find(module.links, link_uids[item.parent_id])
            link, joint = A.add_link(module, child_source.name, parent, item.name, item.type.value,
                                     to_matrix(item.origin), item.axis)
            link_uids[item.child_id] = link.uid
            joint_uids[item.id] = joint.uid
            joint.actuation = item.actuation.value
            joint.role = item.role.value
            joint.damping = item.damping
            joint.friction = item.friction
            limit = item.limit
            if limit is not None:
                if limit.lower is not None and limit.upper is not None:
                    joint.has_position_limits = True
                    joint.lower, joint.upper = limit.lower, limit.upper
                if limit.velocity is not None:
                    joint.has_velocity, joint.velocity = True, limit.velocity
                if limit.effort is not None:
                    joint.has_effort, joint.effort = True, limit.effort

    module = _module_by_uid(scene, module_uid)
    for link_source in source.links:
        for part in link_source.components:
            link = A.find(module.links, link_uids[link_source.id])
            visual = part.visuals[0]
            shape, dims = _shape_args(visual.shape)
            component = A.add_component(module, link, part.name, shape, local=to_matrix(visual.origin), **dims)
            component.mass_mode = part.mass.mode.value
            component.mass = part.mass.mass
            component.assumption = part.mass.assumption
            component.color = visual.material.rgba
            component.object.color = visual.material.rgba

    for frame in source.frames:
        link = A.find(module.links, link_uids[frame.link_id])
        A.add_frame(module, link, frame.name, frame.kind.value, to_matrix(frame.pose))
        module = _module_by_uid(scene, module_uid)

    for pose_name, values in source.poses.items():
        named = module.poses.add()
        named.name = pose_name
        for joint_id, value in values.items():
            entry = named.values.add()
            entry.joint_uid = joint_uids[joint_id]
            entry.value = value
    return module_uid


def add_module(scene, source):
    """Add one standalone module, such as an arm, without mounting it."""
    _build_module(scene, source)
    A.apply_all(scene)


def build_demo(scene, project=None):
    """Create a starter robot in `scene`. All values are illustrative, not real measurements."""
    project = project or demo.build_demo_project()
    data = scene.rab
    data.robot_name = project.name
    data.package_name = f"{project.name}_description"
    uids = {source.id: _build_module(scene, source) for source in project.modules}
    frame_names = {f.id: f.name for m in project.modules for f in m.frames}

    for mount in project.mounts:
        parent = _module_by_uid(scene, uids[mount.parent_module_id])
        child = _module_by_uid(scene, uids[mount.child_module_id])
        parent_frame = next(f for f in parent.frames if f.name == frame_names[mount.parent_frame_id])
        child_frame = next(f for f in child.frames if f.name == frame_names[mount.child_frame_id])
        A.add_mount(scene, parent, parent_frame, child, child_frame, to_matrix(mount.offset))
    A.apply_all(scene)
    return project

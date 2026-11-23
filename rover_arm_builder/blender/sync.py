"""Keeps viewport edits, metadata, and the robotics selection in step."""

import bpy
from bpy.app.handlers import persistent

from . import authoring

_MSGBUS_OWNER = object()


def sync_selection_from_active(scene, obj):
    """Point the robotics lists at the entity that owns `obj`."""
    owner = authoring.owner_of_object(scene, obj)
    if owner is None:
        return False
    module, link, component = owner
    project = scene.rab
    for index, candidate in enumerate(project.modules):
        if candidate.uid == module.uid:
            project.active_module = index
            break
    if link is not None:
        for index, candidate in enumerate(module.links):
            if candidate.uid == link.uid:
                module.active_link = index
                break
        if component is not None:
            for index, candidate in enumerate(link.components):
                if candidate.uid == component.uid:
                    link.active_component = index
                    break
        for index, joint in enumerate(module.joints):
            if joint.child_uid == link.uid:
                module.active_joint = index
                break
    return True


def _on_active_object_changed():
    context = bpy.context
    scene = getattr(context, "scene", None)
    if scene is not None and hasattr(scene, "rab") and len(scene.rab.modules):
        sync_selection_from_active(scene, getattr(context, "active_object", None))


def _subscribe():
    bpy.msgbus.clear_by_owner(_MSGBUS_OWNER)
    bpy.msgbus.subscribe_rna(
        key=(bpy.types.LayerObjects, "active"),
        owner=_MSGBUS_OWNER,
        args=(),
        notify=_on_active_object_changed,
        options={"PERSISTENT"},
    )


@persistent
def _on_depsgraph_update(scene, depsgraph):
    if not len(scene.rab.modules):
        return
    tagged = False
    for update in depsgraph.updates:
        if not isinstance(update.id, bpy.types.Object) or update.id.original.rab.kind == "NONE":
            continue
        if update.is_updated_geometry:
            authoring.touch_model()
        if update.is_updated_transform:
            tagged = True
    if not tagged:
        return
    if not any(authoring.is_previewing(module) for module in scene.rab.modules):
        authoring.touch_model()
    if authoring.pull_from_objects(scene):
        authoring.apply_mounts(scene)


@persistent
def _on_load_post(*_args):
    _subscribe()


HANDLERS = (
    (bpy.app.handlers.depsgraph_update_post, _on_depsgraph_update),
    (bpy.app.handlers.load_post, _on_load_post),
)


def register():
    for handlers, function in HANDLERS:
        if function not in handlers:
            handlers.append(function)
    _subscribe()


def unregister():
    bpy.msgbus.clear_by_owner(_MSGBUS_OWNER)
    for handlers, function in HANDLERS:
        while function in handlers:
            handlers.remove(function)

"""Blender integration: scene data, sync, operators, and interface."""

from . import operators, overlay, panels, props, sync

_MODULES = (props, operators, panels, overlay, sync)


def register():
    for module in _MODULES:
        module.register()


def unregister():
    for module in reversed(_MODULES):
        module.unregister()

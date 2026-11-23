"""Rover & Arm Builder extension entry point.

bpy is imported only inside register and unregister, so `core` and `exporters`
remain importable and testable outside Blender.
"""


def register():
    from . import blender

    blender.register()


def unregister():
    from . import blender

    blender.unregister()

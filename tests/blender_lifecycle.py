"""Enable, disable, and re-enable the installed extension in an isolated Blender profile."""

import os
import sys
import traceback

import addon_utils
import bpy

MODULE = "bl_ext.user_default.rover_arm_builder"


def state():
    return {
        "scene_prop": hasattr(bpy.types.Scene, "rab"),
        "operator": hasattr(bpy.types, "RAB_OT_validate"),
        "panel": hasattr(bpy.types, "RAB_PT_main"),
        "handlers": sum(1 for h in bpy.app.handlers.depsgraph_update_post if h.__name__ == "_on_depsgraph_update"),
    }


ENABLED = {"scene_prop": True, "operator": True, "panel": True, "handlers": 1}
DISABLED = {"scene_prop": False, "operator": False, "panel": False, "handlers": 0}


def check(expected, label):
    got = state()
    if got != expected:
        raise AssertionError(f"{label}: expected {expected}, got {got}")
    print(f"LIFECYCLE {label} ok")


def main():
    check(DISABLED, "before enable")
    for cycle in range(3):
        if addon_utils.enable(MODULE, default_set=False) is None:
            raise AssertionError("Extension failed to enable")
        check(ENABLED, f"enabled {cycle}")
        bpy.ops.rab.validate()
        addon_utils.disable(MODULE, default_set=False)
        check(DISABLED, f"disabled {cycle}")


try:
    main()
    sys.stdout.flush()
    os._exit(0)
except BaseException:
    traceback.print_exc()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(1)

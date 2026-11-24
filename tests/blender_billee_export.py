"""Create the BILLEE rover and export robot_description through the real Blender operators.

Run: blender --background --factory-startup --python tests/blender_billee_export.py -- <output_dir>
"""

import os
import sys
import traceback
from pathlib import Path

import bpy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import rover_arm_builder  # noqa: E402


def main():
    output = sys.argv[sys.argv.index("--") + 1]
    bpy.ops.wm.read_factory_settings(use_empty=True)
    rover_arm_builder.register()
    if bpy.ops.rab.create_billee() != {"FINISHED"}:
        raise RuntimeError("Create BILLEE Rover failed")
    scene = bpy.context.scene
    bpy.ops.rab.validate()
    print(f"DROPIN_STATUS {scene.rab.billee_status}")
    scene.rab.export_dir = output
    if bpy.ops.rab.export_package() != {"FINISHED"}:
        raise RuntimeError("Export failed")
    print(f"DROPIN_EXPORT {scene.rab.last_export}")
    arguments = sys.argv[sys.argv.index("--") + 1:]
    if len(arguments) > 1:
        scene.rab.last_export = ""
        scene.rab.export_dir = "~/Desktop/billee-export"
        bpy.ops.wm.save_as_mainfile(filepath=arguments[1])


try:
    main()
    sys.stdout.flush()
    os._exit(0)
except BaseException:
    traceback.print_exc()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(1)

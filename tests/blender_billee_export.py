"""Create the BILLEE rover and arm, then export them through the real Blender operators.

Run: blender --background --factory-startup --python tests/blender_billee_export.py -- <output_dir> [scene.blend]

Writes <output_dir>/assembly, <output_dir>/rover, and <output_dir>/arm.
"""

import os
import sys
import traceback
from pathlib import Path

import bpy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import rover_arm_builder  # noqa: E402


def export(data, scope, kind, folder):
    data.export_scope = scope
    if kind:
        data.active_module = next(i for i, m in enumerate(data.modules) if m.kind == kind)
    bpy.ops.rab.validate()
    print(f"EXPORT_STATUS {scope} {kind or 'all'}: {data.billee_status}")
    data.export_dir = folder
    if bpy.ops.rab.export_package() != {"FINISHED"}:
        raise RuntimeError(f"Export failed for {scope} {kind}")


def main():
    arguments = sys.argv[sys.argv.index("--") + 1:]
    output = arguments[0]
    bpy.ops.wm.read_factory_settings(use_empty=True)
    rover_arm_builder.register()
    if bpy.ops.rab.create_billee(with_arm=True) != {"FINISHED"}:
        raise RuntimeError("Create BILLEE model failed")
    data = bpy.context.scene.rab
    export(data, "assembly", None, os.path.join(output, "assembly"))
    export(data, "active", "rover", os.path.join(output, "rover"))
    export(data, "active", "arm", os.path.join(output, "arm"))
    if len(arguments) > 1:
        data.export_scope = "assembly"
        data.active_module = 0
        data.last_export = ""
        data.export_dir = "~/Desktop/billee-export"
        bpy.ops.rab.validate()
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

"""Create the BILLEE model and export it through the real Blender operators.

Run: blender --background --factory-startup --python tests/blender_billee_export.py -- <output_dir> [scene.blend] [--with-arm]

With --with-arm, it also exports the arm by itself into <output_dir>/standalone.
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
    after = sys.argv[sys.argv.index("--") + 1:]
    with_arm = "--with-arm" in after
    arguments = [item for item in after if not item.startswith("--")]
    output = arguments[0]
    bpy.ops.wm.read_factory_settings(use_empty=True)
    rover_arm_builder.register()
    if bpy.ops.rab.create_billee(with_arm=with_arm) != {"FINISHED"}:
        raise RuntimeError("Create BILLEE model failed")
    scene = bpy.context.scene
    data = scene.rab
    bpy.ops.rab.validate()
    print(f"DROPIN_STATUS {data.billee_status}")
    data.export_dir = output
    if bpy.ops.rab.export_package() != {"FINISHED"}:
        raise RuntimeError("Export failed")
    print(f"DROPIN_EXPORT {data.last_export}")

    if with_arm:
        data.export_scope = "active"
        data.active_module = next(i for i, m in enumerate(data.modules) if m.kind == "arm")
        data.export_dir = os.path.join(output, "standalone")
        if bpy.ops.rab.export_package() != {"FINISHED"}:
            raise RuntimeError("Standalone arm export failed")
        print(f"ARM_EXPORT {data.last_export}")
        data.export_scope = "assembly"
        data.active_module = 0

    if len(arguments) > 1:
        data.last_export = ""
        data.export_dir = "~/Desktop/billee-export"
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

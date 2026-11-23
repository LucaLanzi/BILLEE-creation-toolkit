"""Generate the demo .blend and ROS 2 package. All robot values are illustrative.

Run: blender --background --factory-startup --python examples/generate_demo.py -- [output_dir]
"""

import os
import sys
from pathlib import Path

import bpy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import rover_arm_builder  # noqa: E402
from rover_arm_builder.blender.demo_scene import build_demo  # noqa: E402
from rover_arm_builder.blender.extract import extract_project, validate_scene  # noqa: E402
from rover_arm_builder.core.validate import Target  # noqa: E402
from rover_arm_builder.exporters.package import write_package  # noqa: E402


def main():
    args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    output = Path(args[0]).resolve() if args else ROOT / "examples" / "output"
    output.mkdir(parents=True, exist_ok=True)

    bpy.ops.wm.read_factory_settings(use_empty=True)
    rover_arm_builder.register()
    scene = bpy.context.scene
    build_demo(scene)
    bpy.context.view_layer.update()

    _project, report = validate_scene(scene)
    blockers = [i.message for t in Target for i in report.blockers(t)]
    if blockers:
        raise SystemExit("Demo is not valid: " + "; ".join(blockers))

    project, issues = extract_project(scene)
    if issues:
        raise SystemExit(f"Demo extraction failed: {issues}")
    package = output / scene.rab.package_name
    write_package(project, package, scene.rab.package_name, Target.PHYSICS)
    scene.rab.export_dir = "//"
    blend = output / "demo_rover_arm.blend"
    bpy.ops.wm.save_as_mainfile(filepath=str(blend), compress=True)
    print(f"DEMO_BLEND {blend}")
    print(f"DEMO_PACKAGE {package}")
    sys.stdout.flush()
    os._exit(0)


try:
    main()
except BaseException as exc:
    print(f"DEMO_FAILED {exc}")
    import traceback

    traceback.print_exc()
    sys.stdout.flush()
    os._exit(1)

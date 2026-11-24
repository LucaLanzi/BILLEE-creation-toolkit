"""Capture README screenshots from the real Robot tab and test the modal joint drag.

Run through: python scripts/capture_docs.py
Uses Blender's event simulation, so real mouse and keyboard input does not affect the window.
"""

import faulthandler
import os
import sys
import traceback
from pathlib import Path

import bpy
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import rover_arm_builder  # noqa: E402
from rover_arm_builder.blender import authoring as A  # noqa: E402
from rover_arm_builder.blender import panels  # noqa: E402

OUTPUT = Path(sys.argv[sys.argv.index("--") + 1])
OUTPUT.mkdir(parents=True, exist_ok=True)
focus = {"panel": None}
results = []


def focus_panels():
    """Show one sidebar section at a time so each picture stays readable."""
    for cls in panels.CLASSES:
        if not issubclass(cls, bpy.types.Panel) or cls is panels.RAB_PT_main:
            continue
        original = getattr(cls, "poll", None)

        def make(original, name):
            def poll(_cls, context):
                if focus["panel"] not in (None, name):
                    return False
                return True if original is None else original(context)

            return classmethod(poll)

        cls.poll = make(original, cls.__name__)
        cls.bl_options = {o for o in getattr(cls, "bl_options", set()) if o != "DEFAULT_CLOSED"}


def areas():
    window = bpy.context.window_manager.windows[0]
    area = next(a for a in window.screen.areas if a.type == "VIEW_3D")
    return (window, area, next(r for r in area.regions if r.type == "WINDOW"),
            next(r for r in area.regions if r.type == "UI"))


def click(window, x, y):
    window.event_simulate("MOUSEMOVE", "NOTHING", x=x, y=y)
    yield 0.12
    window.event_simulate("LEFTMOUSE", "PRESS", x=x, y=y)
    yield 0.08
    window.event_simulate("LEFTMOUSE", "RELEASE", x=x, y=y)
    yield 0.2


def crop(path, area):
    image = bpy.data.images.load(str(path), check_existing=False)
    width, height = image.size
    pixels = np.empty(width * height * 4, dtype=np.float32)
    image.pixels.foreach_get(pixels)
    pixels = pixels.reshape(height, width, 4)
    x0, y0 = max(area.x, 0), max(area.y, 0)
    x1, y1 = min(area.x + area.width, width), min(area.y + area.height, height)
    part = pixels[y0:y1, x0:x1].copy()
    part[:, :, 3] = 1.0
    out = bpy.data.images.new("capture", x1 - x0, y1 - y0, alpha=False)
    out.pixels.foreach_set(part.ravel())
    out.filepath_raw = str(path)
    out.file_format = "PNG"
    out.save()
    bpy.data.images.remove(out)
    bpy.data.images.remove(image)


def shot(name, panel):
    window, area, _region, _ui = areas()
    focus["panel"] = panel
    for item in window.screen.areas:
        item.tag_redraw()
    yield 0.9
    path = OUTPUT / f"{name}.png"
    with bpy.context.temp_override(window=window, area=area):
        bpy.ops.screen.screenshot(filepath=str(path))
    crop(path, area)
    results.append((f"captured {name}", path.is_file() and path.stat().st_size > 20000))
    yield 0.1


def select(project, tab, link_name=None, joint_name=None):
    project.ui_tab = tab
    if tab == "assembly":
        return None
    project.active_module = next(i for i, m in enumerate(project.modules) if m.kind == tab)
    module = project.modules[project.active_module]
    if link_name:
        index = next(i for i, l in enumerate(module.links) if l.name == link_name)
        module.active_link = index
        A.select_link_objects(bpy.context.view_layer, module.links[index])
    if joint_name:
        module.active_joint = next(i for i, j in enumerate(module.joints) if j.name == joint_name)
    return module


def steps_billee():
    """One picture of the drop-in export, from a real BILLEE rover scene."""
    yield from open_robot_tab()
    if not results[-1][1]:
        return
    window, area, region, _ui = areas()
    project = window.scene.rab
    select(project, "rover", "base")
    with bpy.context.temp_override(window=window, area=area, region=region):
        bpy.ops.rab.validate()
    results.append(("BILLEE rover is ready to export", project.billee_status.startswith("Ready")))
    yield from shot("11-billee-dropin", "RAB_PT_export")


def steps():
    yield from open_robot_tab()
    if not results[-1][1]:
        return
    yield from demo_steps()


def open_robot_tab():
    window, area, region, ui = areas()
    space = area.spaces.active
    space.show_region_ui = True
    space.shading.color_type = "OBJECT"
    space.overlay.show_floor = True
    with bpy.context.temp_override(window=window, area=area, region=region):
        bpy.ops.view3d.view_all()
    yield 0.8

    for offset in range(14, ui.height, 16):
        yield from click(window, ui.x + ui.width - 9, ui.y + ui.height - offset)
        if ui.active_panel_category == "Robot":
            break
    results.append(("Robot tab opened by a real click", ui.active_panel_category == "Robot"))
    if ui.active_panel_category != "Robot":
        return

    edge_y = ui.y + ui.height // 2
    window.event_simulate("MOUSEMOVE", "NOTHING", x=ui.x + 1, y=edge_y)
    yield 0.15
    window.event_simulate("LEFTMOUSE", "PRESS", x=ui.x + 1, y=edge_y)
    yield 0.1
    window.event_simulate("MOUSEMOVE", "NOTHING", x=ui.x - 130, y=edge_y)
    yield 0.15
    window.event_simulate("LEFTMOUSE", "RELEASE", x=ui.x - 130, y=edge_y)
    yield 0.3
    window, area, region, ui = areas()
    with bpy.context.temp_override(window=window, area=area, region=region):
        bpy.ops.view3d.view_all()
    yield 0.5


def demo_steps():
    window, area, region, _ui = areas()
    project = window.scene.rab
    select(project, "rover", "front_axle", "front_axle_joint")
    yield from shot("01-rover-links", "RAB_PT_links")

    select(project, "rover", "chassis")
    yield from shot("02-components-mass", "RAB_PT_components")

    module = select(project, "arm", "forearm_link", "elbow_pitch_joint")
    module.joints[0].preview = 0.7
    module.joints[1].preview = 1.0
    yield from shot("03-joint-preview", "RAB_PT_joint")

    module = project.modules[project.active_module]
    A.reset_preview(module)
    module.active_frame = next(i for i, f in enumerate(module.frames) if f.kind == "tool")
    yield from shot("04-frames", "RAB_PT_frames")

    select(project, "assembly")
    project.mount_parent_module = project.modules[0].uid
    project.mount_child_module = project.modules[1].uid
    project.mount_parent_frame = project.modules[0].frames[0].uid
    project.mount_child_frame = project.modules[1].frames[0].uid
    yield from shot("05-assembly", "RAB_PT_assembly")

    project.show_collision = True
    yield from shot("06-collision-overlay", "RAB_PT_display")
    project.show_collision = False

    yield from shot("07-mass-summary", "RAB_PT_mass_summary")

    module = select(project, "arm", "forearm_link", "elbow_pitch_joint")
    joint = next(j for j in module.joints if j.name == "elbow_pitch_joint")
    joint.has_velocity = False
    with bpy.context.temp_override(window=window, area=area, region=region):
        bpy.ops.rab.validate()
    yield from shot("08-readiness-issue", "RAB_PT_readiness")
    joint = next(j for j in project.modules[project.active_module].joints if j.name == "elbow_pitch_joint")
    joint.has_velocity = True
    with bpy.context.temp_override(window=window, area=area, region=region):
        bpy.ops.rab.validate()
    yield from shot("09-readiness-ready", "RAB_PT_readiness")

    project.package_name = "my_robot_description"
    project.export_dir = "~/ros_ws/src"
    yield from shot("10-export", "RAB_PT_export")

    focus["panel"] = "RAB_PT_joint"
    joint = next(j for j in project.modules[project.active_module].joints if j.name == "elbow_pitch_joint")
    start_x, start_y = region.x + region.width // 2, region.y + region.height // 2
    window.event_simulate("MOUSEMOVE", "NOTHING", x=start_x, y=start_y)
    yield 0.2
    with bpy.context.temp_override(window=window, area=area, region=region):
        bpy.ops.rab.joint_drag("INVOKE_DEFAULT")
    yield 0.2
    window.event_simulate("MOUSEMOVE", "NOTHING", x=start_x + 60, y=start_y)
    yield 0.3
    dragged = next(j for j in project.modules[project.active_module].joints if j.name == "elbow_pitch_joint").preview
    results.append(("mouse drag previews the joint", abs(dragged - 0.6) < 0.05))
    window.event_simulate("ESC", "PRESS", x=start_x + 60, y=start_y)
    yield 0.1
    window.event_simulate("ESC", "RELEASE", x=start_x + 60, y=start_y)
    yield 0.3
    restored = next(j for j in project.modules[project.active_module].joints if j.name == "elbow_pitch_joint").preview
    results.append(("Esc cancels the drag and restores the pose", abs(restored) < 1e-6))


sequence = None


def finish(code):
    for name, passed in results:
        print(f"CAPTURE_RESULT {'PASS' if passed else 'FAIL'} {name}")
    print(f"CAPTURE_DONE {code}")
    sys.stdout.flush()
    os._exit(code)


def tick():
    global sequence
    try:
        if sequence is None:
            sequence = steps_billee() if "billee" in sys.argv else steps()
        return next(sequence)
    except StopIteration:
        finish(0 if results and all(p for _, p in results) else 1)
    except BaseException:
        traceback.print_exc()
        finish(1)
    return None


faulthandler.dump_traceback_later(100, exit=True, file=sys.__stderr__)
focus_panels()
try:
    rover_arm_builder.register()
except BaseException:
    traceback.print_exc()
    finish(1)
bpy.app.timers.register(tick, first_interval=2.0)

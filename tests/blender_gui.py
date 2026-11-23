"""Windowed test: proves each interface panel drew, then checks undo, redo, and the workspace.

Run through: python scripts/check.py --gui
It needs a display, opens a real window, and closes itself.
"""

import faulthandler
import os
import sys
import traceback
from collections import Counter
from pathlib import Path

import bpy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import rover_arm_builder  # noqa: E402
from rover_arm_builder.blender import panels  # noqa: E402

OUTPUT = Path(sys.argv[sys.argv.index("--") + 1])
OUTPUT.mkdir(parents=True, exist_ok=True)

COMMON = {"RAB_PT_main", "RAB_PT_display", "RAB_PT_mass_summary", "RAB_PT_readiness", "RAB_PT_export"}
BUILDER = COMMON | {"RAB_PT_links", "RAB_PT_components", "RAB_PT_joint", "RAB_PT_frames"}
EXPECTED = {"rover": BUILDER, "arm": BUILDER, "assembly": COMMON | {"RAB_PT_assembly"}}
TABS = tuple(EXPECTED)

DRAWN = Counter()
DRAW_ERRORS = []
state = {"step": 0, "results": []}


def _counted(original, name):
    def draw(self, context):
        DRAWN[name] += 1
        try:
            original(self, context)
        except Exception:
            DRAW_ERRORS.append(f"{name}: {traceback.format_exc()}")
            raise

    return draw


def instrument_panels():
    """Count real draw calls. Opening closed panels ensures their bodies are exercised."""
    for cls in panels.CLASSES:
        if not issubclass(cls, bpy.types.Panel):
            continue
        cls.draw = _counted(cls.draw, cls.__name__)
        cls.bl_options = {o for o in getattr(cls, "bl_options", set()) if o != "DEFAULT_CLOSED"}


def finish(code):
    for error in DRAW_ERRORS:
        print(f"GUI_DRAW_ERROR {error}")
    for name, passed in state["results"]:
        print(f"GUI_RESULT {'PASS' if passed else 'FAIL'} {name}")
    print(f"GUI_DONE {code}")
    sys.stdout.flush()
    os._exit(code)


def view_context():
    window = bpy.context.window_manager.windows[0]
    area = next(a for a in window.screen.areas if a.type == "VIEW_3D")
    region = next(r for r in area.regions if r.type == "WINDOW")
    return window, area, region


def redraw(window):
    """Request a normal redraw. It happens after the current timer callback returns."""
    for area in window.screen.areas:
        area.tag_redraw()


def prepare():
    window, area, region = view_context()
    space = area.spaces.active
    space.show_region_ui = True
    space.shading.color_type = "OBJECT"
    with bpy.context.temp_override(window=window, area=area, region=region):
        bpy.ops.view3d.view_all()


def select_tab(tab):
    window, area, region = view_context()
    project = window.scene.rab
    project.ui_tab = tab
    project.show_collision = tab == "assembly"
    if tab in {"rover", "arm"}:
        project.active_module = next(i for i, m in enumerate(project.modules) if m.kind == tab)
        module = project.modules[project.active_module]
        module.active_link = min(1, len(module.links) - 1)
    with bpy.context.temp_override(window=window, area=area, region=region):
        bpy.ops.rab.validate()
    DRAWN.clear()
    redraw(window)


def capture(tab):
    window, area, _region = view_context()
    missing = sorted(EXPECTED[tab] - set(DRAWN))
    if missing:
        print(f"GUI_MISSING {tab}: {missing}")
    state["results"].append((f"{tab}: all {len(EXPECTED[tab])} panels drew", not missing and not DRAW_ERRORS))
    path = OUTPUT / f"{tab}.png"
    with bpy.context.temp_override(window=window, area=area):
        bpy.ops.screen.screenshot(filepath=str(path))
    state["results"].append((f"{tab}: screenshot saved", path.is_file() and path.stat().st_size > 10000))


def undo_redo():
    window, area, region = view_context()
    project = window.scene.rab
    project.ui_tab = "arm"
    project.active_module = next(i for i, m in enumerate(project.modules) if m.kind == "arm")
    module_uid = project.modules[project.active_module].uid

    def count():
        data = bpy.context.window_manager.windows[0].scene.rab
        return len(next(m for m in data.modules if m.uid == module_uid).links)

    before = count()
    with bpy.context.temp_override(window=window, area=area, region=region):
        bpy.ops.ed.undo_push(message="before add")
        bpy.ops.rab.add_link("EXEC_DEFAULT", True, link_name="undo_probe", joint_type="fixed")
        added = count()
        bpy.ops.ed.undo()
        undone = count()
        bpy.ops.ed.redo()
        redone = count()
        bpy.ops.ed.undo()
    state["results"].append(("Undo and redo of Add Link", (added, undone, redone) == (before + 1, before, before + 1)))


def workspace():
    window, area, region = view_context()
    with bpy.context.temp_override(window=window, area=area, region=region):
        state["workspace_result"] = bpy.ops.rab.setup_workspace()


def workspace_check():
    """Blender applies a workspace switch on the next event-loop pass."""
    window = bpy.context.window_manager.windows[0]
    state["results"].append(("Robot Builder workspace opens",
                             state["workspace_result"] == {"FINISHED"} and window.workspace.name == "Robot Builder"))


def tick():
    try:
        step = state["step"]
        state["step"] += 1
        if step == 0:
            prepare()
            return 0.5
        index = step - 1
        if index < len(TABS) * 2:
            tab = TABS[index // 2]
            if index % 2 == 0:
                select_tab(tab)
                return 1.5
            capture(tab)
            return 0.2
        if index == len(TABS) * 2:
            undo_redo()
            workspace()
            return 1.0
        workspace_check()
        finish(0 if all(passed for _, passed in state["results"]) and not DRAW_ERRORS else 1)
    except BaseException:
        traceback.print_exc()
        finish(1)
    return None


# Blender 5.3 exposes the active sidebar tab as read-only, and a new window opens on "Tool".
# For this test only, the panels join that tab so they are drawn without simulated clicks.
faulthandler.dump_traceback_later(60, exit=True, file=sys.__stderr__)
panels.RobotPanel.bl_category = "Tool"
instrument_panels()
try:
    rover_arm_builder.register()
except BaseException:
    traceback.print_exc()
    finish(1)
bpy.app.timers.register(tick, first_interval=2.0)

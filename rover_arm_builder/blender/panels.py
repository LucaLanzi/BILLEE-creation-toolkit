"""Native sidebar interface: Rover, Arm, and Assembly."""

import json

import bpy
from bpy.types import Panel, UIList

from ..core.readiness import Stage, evaluate
from ..core.validate import Severity, Target
from . import authoring as A
from .extract import validate_scene
from .operators import active_component, active_frame, active_joint, active_link, active_module

STAGE_ICONS = {
    Stage.MISSING_INPUTS: "ERROR",
    Stage.AUTHORED: "GREASEPENCIL",
    Stage.EXPORTED: "EXPORT",
    Stage.INTERNALLY_VALIDATED: "CHECKMARK",
    Stage.EXTERNALLY_VALIDATED: "CHECKMARK",
    Stage.TESTED_IN_RUNTIME: "CHECKMARK",
    Stage.NOT_CHECKED: "QUESTION",
}
SEVERITY_ICONS = {"error": "ERROR", "warning": "INFO", "info": "QUESTION"}
TARGET_LABELS = {
    Target.VISUALIZATION: "Visualization",
    Target.PLANNING: "Arm planning",
    Target.PHYSICS: "Physics simulation",
}


def _depths(module):
    parents = {j.child_uid: j.parent_uid for j in module.joints}
    depths = {}
    for link in module.links:
        depth, current, seen = 0, link.uid, set()
        while current in parents and current not in seen:
            seen.add(current)
            current = parents[current]
            depth += 1
        depths[link.uid] = depth
    return depths


class RAB_UL_modules(UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_property, index=0, flt_flag=0):
        icons = {"rover": "AUTO", "arm": "CON_KINEMATIC", "accessory": "OBJECT_DATA"}
        layout.prop(item, "name", text="", emboss=False, icon=icons.get(item.kind, "OBJECT_DATA"))
        layout.label(text=item.prefix)

    def filter_items(self, context, data, propname):
        items = getattr(data, propname)
        tab = data.ui_tab
        if tab == "assembly":
            return [self.bitflag_filter_item] * len(items), []
        return [self.bitflag_filter_item if m.kind == tab else 0 for m in items], []


class RAB_UL_links(UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_property, index=0, flt_flag=0):
        depth = _depths(data).get(item.uid, 0)
        row = layout.row(align=True)
        for _ in range(depth):
            row.separator(factor=1.5)
        joint = next((j for j in data.joints if j.child_uid == item.uid), None)
        row.prop(item, "name", text="", emboss=False, icon="EMPTY_ARROWS" if depth == 0 else "BONE_DATA")
        if joint is not None:
            row.label(text=joint.type)


class RAB_UL_components(UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_property, index=0, flt_flag=0):
        layout.prop(item, "name", text="", emboss=False, icon="MESH_CUBE")
        layout.label(text="No mass" if item.mass_mode == "none" else f"{item.mass:.3g} kg" if item.mass_mode != "density" else "Density")


class RAB_UL_joints(UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_property, index=0, flt_flag=0):
        layout.prop(item, "name", text="", emboss=False, icon="DRIVER")
        layout.label(text=f"{item.actuation} / {item.role}")


class RAB_UL_frames(UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_property, index=0, flt_flag=0):
        layout.prop(item, "name", text="", emboss=False, icon="EMPTY_AXIS")
        layout.label(text=item.kind)


class RAB_UL_issues(UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_property, index=0, flt_flag=0):
        row = layout.row(align=True)
        row.label(text=f"{item.severity.upper()}: {item.message}", icon=SEVERITY_ICONS.get(item.severity, "INFO"))
        row.operator("rab.focus_issue", text="", icon="RESTRICT_SELECT_OFF").entity = item.entity


class RobotPanel:
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Robot"


class BuilderPanel(RobotPanel):
    @classmethod
    def poll(cls, context):
        project = context.scene.rab
        module = active_module(context)
        return project.ui_tab in {"rover", "arm"} and module is not None and module.kind == project.ui_tab


class RAB_PT_main(RobotPanel, Panel):
    bl_label = "Rover & Arm Builder"

    def draw(self, context):
        layout = self.layout
        project = context.scene.rab
        layout.prop(project, "ui_tab", expand=True)
        layout.prop(project, "robot_name", text="Robot")
        row = layout.row()
        row.template_list("RAB_UL_modules", "", project, "modules", project, "active_module", rows=2)
        column = row.column(align=True)
        kind = project.ui_tab if project.ui_tab in {"rover", "arm"} else "accessory"
        column.operator("rab.new_module", text="", icon="ADD").kind = kind
        column.operator("rab.duplicate_module", text="", icon="DUPLICATE")
        module = active_module(context)
        if module is not None:
            layout.prop(module, "prefix", text="Prefix")
        else:
            layout.operator("rab.create_demo", icon="AUTO")
        layout.operator("rab.setup_workspace", icon="WORKSPACE")


class RAB_PT_links(BuilderPanel, Panel):
    bl_label = "Links"
    bl_parent_id = "RAB_PT_main"

    def draw(self, context):
        layout = self.layout
        module = active_module(context)
        row = layout.row()
        row.template_list("RAB_UL_links", "", module, "links", module, "active_link", rows=4)
        column = row.column(align=True)
        column.operator("rab.add_link", text="", icon="ADD")
        column.operator("rab.remove_link", text="", icon="REMOVE")
        box = layout.box()
        _note(box, context, "Move link frame, keep geometry in place:")
        row = box.row(align=True)
        row.operator("rab.reframe_link", text="To Cursor").mode = "CURSOR"
        row.operator("rab.reframe_link", text="To Origin").mode = "ORIGIN"
        row.operator("rab.reframe_link", text="To Center").mode = "CENTER"
        _note(box, context, "To move a link with its geometry, move its frame object.", "INFO")


class RAB_PT_components(BuilderPanel, Panel):
    bl_label = "Components and Mass"
    bl_parent_id = "RAB_PT_main"

    def draw(self, context):
        layout = self.layout
        module = active_module(context)
        link = active_link(module)
        if link is None:
            layout.label(text="Select a link.")
            return
        row = layout.row()
        row.template_list("RAB_UL_components", "", link, "components", link, "active_component", rows=3)
        column = row.column(align=True)
        column.operator("rab.add_component", text="", icon="ADD")
        column.operator("rab.remove_component", text="", icon="REMOVE")
        layout.operator("rab.adopt_object", icon="MESH_DATA")
        component = active_component(link)
        if component is None:
            return

        layout.prop(component, "shape")
        if component.shape == "BOX":
            layout.prop(component, "size")
        elif component.shape == "CYLINDER":
            layout.prop(component, "radius")
            layout.prop(component, "length")
        elif component.shape == "SPHERE":
            layout.prop(component, "radius")
        if component.shape != "MESH":
            layout.prop(component, "allow_mesh_fallback")

        box = layout.box()
        _note(box, context, "Physical mass (independent of materials)", "PHYSICS")
        box.prop(component, "mass_mode")
        if component.mass_mode in {"measured", "estimated", "manual"}:
            box.prop(component, "mass")
        if component.mass_mode == "density":
            box.prop(component, "density")
            kg = A.component_mass_kg(component, context.scene.unit_settings.scale_length)
            box.label(text=f"Computed mass: {kg:.4g} kg")
        if component.mass_mode in {"measured", "estimated", "density"}:
            if component.shape == "MESH":
                box.label(text="Mesh parts need Manual Override mass.", icon="ERROR")
            _note(box, context, "Assumes a uniform solid. Hollow parts need manual values.", "INFO")
        if component.mass_mode == "manual":
            box.prop(component, "manual_com")
            grid = box.grid_flow(columns=3, align=True)
            for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
                grid.prop(component, key)
            _note(box, context, "Inertia is about the COM, in component axes (kg*m^2).")
            if component.needs_review:
                box.prop(component, "needs_review", icon="ERROR")
        if component.mass_mode != "none":
            box.prop(component, "overlap_confirmed")

        box = layout.box()
        box.label(text="Display and collision", icon="MOD_PHYSICS")
        row = box.row()
        row.prop(component, "export_visual")
        row.prop(component, "use_as_collision")
        row = box.row(align=True)
        row.operator("rab.add_collision", text="Box Proxy").shape = "BOX"
        row.operator("rab.add_collision", text="Cylinder").shape = "CYLINDER"
        row.operator("rab.add_collision", text="Sphere").shape = "SPHERE"
        for proxy in component.collisions:
            box.label(text=proxy.name, icon="MESH_ICOSPHERE")


class RAB_PT_joint(BuilderPanel, Panel):
    bl_label = "Joints and Motion"
    bl_parent_id = "RAB_PT_main"

    def draw(self, context):
        layout = self.layout
        module = active_module(context)
        layout.template_list("RAB_UL_joints", "", module, "joints", module, "active_joint", rows=3)
        joint = active_joint(module)
        if joint is None:
            layout.label(text="Add a link to create a joint.")
            return
        layout.prop(joint, "type")
        layout.prop(joint, "actuation", text="Actuation")
        layout.prop(joint, "role")
        layout.prop(joint.origin, "location", text="Joint Position")
        layout.prop(joint.origin, "rotation", text="Joint Rotation")
        if joint.type == "fixed":
            return

        layout.prop(joint, "axis")
        angular = joint.type != "prismatic"
        box = layout.box()
        box.label(text="Limits (" + ("radians, rad/s, N*m" if angular else "length, m/s, N") + ")", icon="DRIVER_DISTANCE")
        if joint.type != "continuous":
            box.prop(joint, "has_position_limits")
            if joint.has_position_limits:
                row = box.row(align=True)
                row.prop(joint, "lower")
                row.prop(joint, "upper")
        row = box.row(align=True)
        row.prop(joint, "has_velocity", text="")
        sub = row.row(align=True)
        sub.enabled = joint.has_velocity
        sub.prop(joint, "velocity")
        row = box.row(align=True)
        row.prop(joint, "has_effort", text="")
        sub = row.row(align=True)
        sub.enabled = joint.has_effort
        sub.prop(joint, "effort")
        row = box.row(align=True)
        row.prop(joint, "damping")
        row.prop(joint, "friction")

        box = layout.box()
        box.label(text="Motion preview (never exported)", icon="PLAY")
        row = box.row(align=True)
        row.prop(joint, "preview", slider=False)
        row.operator("rab.joint_drag", text="", icon="ORIENTATION_GIMBAL")
        row = box.row(align=True)
        row.operator("rab.preview_reset", icon="LOOP_BACK")
        row.operator("rab.pose_store", icon="BOOKMARKS")
        for pose in module.poses:
            box.operator("rab.pose_apply", text=f"Preview: {pose.name}").pose_name = pose.name
        box.operator("rab.apply_zero", icon="ERROR")


class RAB_PT_frames(BuilderPanel, Panel):
    bl_label = "Frames"
    bl_parent_id = "RAB_PT_main"

    def draw(self, context):
        layout = self.layout
        module = active_module(context)
        row = layout.row()
        row.template_list("RAB_UL_frames", "", module, "frames", module, "active_frame", rows=2)
        row.column(align=True).operator("rab.add_frame", text="", icon="ADD")
        frame = active_frame(module)
        if frame is None:
            return
        owner = A.find(module.links, frame.link_uid)
        layout.label(text=f"Owner link: {owner.name if owner else 'missing'}")
        layout.prop(frame, "kind")
        layout.prop(frame.pose, "location")
        layout.prop(frame.pose, "rotation")
        row = layout.row(align=True)
        row.operator("rab.frame_place", text="To Cursor").mode = "CURSOR"
        row.operator("rab.frame_place", text="To Origin").mode = "ORIGIN"
        row.operator("rab.frame_place", text="To Center").mode = "CENTER"
        row = layout.row(align=True)
        row.operator("rab.copy_pose", icon="COPYDOWN")
        row.operator("rab.paste_pose", icon="PASTEDOWN")


class RAB_PT_assembly(RobotPanel, Panel):
    bl_label = "Assembly"
    bl_parent_id = "RAB_PT_main"

    @classmethod
    def poll(cls, context):
        return context.scene.rab.ui_tab == "assembly"

    def draw(self, context):
        layout = self.layout
        project = context.scene.rab
        column = layout.column()
        column.enabled = not project.mount_preview_active
        column.label(text="1-2. Choose modules")
        column.prop(project, "mount_parent_module", text="Parent")
        column.prop(project, "mount_child_module", text="Child")
        column.label(text="3-4. Choose mounting frames")
        column.prop(project, "mount_parent_frame", text="On Parent")
        column.prop(project, "mount_child_frame", text="On Child")
        layout.label(text="5. Align (offset in parent frame)")
        layout.prop(project, "mount_offset_location", text="Move")
        layout.prop(project, "mount_offset_rotation", text="Rotate")
        if project.mount_preview_active:
            row = layout.row(align=True)
            row.operator("rab.mount_confirm", icon="CHECKMARK")
            row.operator("rab.mount_cancel", icon="X")
        else:
            layout.operator("rab.mount_preview", icon="HIDE_OFF")

        if len(project.mounts):
            box = layout.box()
            box.label(text="Current mounts", icon="LINKED")
            for mount in project.mounts:
                parent = A.find(project.modules, mount.parent_module_uid)
                child = A.find(project.modules, mount.child_module_uid)
                row = box.row()
                row.label(text=f"{child.name if child else '?'} on {parent.name if parent else '?'}")
                row.operator("rab.detach", text="", icon="UNLINKED").uid = mount.uid


class RAB_PT_display(RobotPanel, Panel):
    bl_label = "Display"
    bl_parent_id = "RAB_PT_main"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        layout = self.layout
        project = context.scene.rab
        layout.prop(project, "display_mode", text="")
        grid = layout.grid_flow(columns=2, align=True)
        for key in ("show_frames", "show_axes", "show_limits", "show_com", "show_collision"):
            grid.prop(project, key, toggle=True)
        _note(layout, context, "The overlay is visual only. It does not test for collisions.", "INFO")


class RAB_PT_mass_summary(RobotPanel, Panel):
    bl_label = "Mass Summary"
    bl_parent_id = "RAB_PT_main"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        scale = scene.unit_settings.scale_length
        for kind, label in (("rover", "Rover"), ("arm", "Arm")):
            total, _com = A.mass_summary([m for m in scene.rab.modules if m.kind == kind], scale)
            layout.label(text=f"{label}: {total:.3f} kg (estimate)")
        total, com = A.mass_summary(scene.rab.modules, scale)
        layout.label(text=f"Assembly: {total:.3f} kg (estimate)")
        if com is not None:
            layout.label(text=f"COM ({com.x * scale:.3f}, {com.y * scale:.3f}, {com.z * scale:.3f}) m")
            _note(layout, context, "COM depends on the displayed pose.", "INFO")
        _note(layout, context, "Does not establish tipping stability.", "INFO")


class RAB_PT_readiness(RobotPanel, Panel):
    bl_label = "Readiness"
    bl_parent_id = "RAB_PT_main"

    def draw(self, context):
        layout = self.layout
        project = context.scene.rab
        layout.operator("rab.validate", icon="VIEWZOOM")
        cached = _LastReport.get(context.scene)
        if cached is None:
            layout.label(text="Not checked. Press Check Model.", icon="QUESTION")
        else:
            for status in cached:
                row = layout.row()
                row.label(text=TARGET_LABELS[status.target], icon=STAGE_ICONS[status.stage])
                row.label(text=status.stage.value)
        if len(project.issues):
            layout.template_list("RAB_UL_issues", "", project, "issues", project, "active_issue", rows=3)
            if 0 <= project.active_issue < len(project.issues):
                issue = project.issues[project.active_issue]
                box = layout.box()
                _note(box, context, issue.message)
                _note(box, context, "Fix: " + issue.action)
                if issue.blocks:
                    _note(box, context, f"Blocks: {issue.blocks}")
                box.operator("rab.focus_issue", text="Select and Open Settings").entity = issue.entity


class RAB_PT_export(RobotPanel, Panel):
    bl_label = "Export"
    bl_parent_id = "RAB_PT_main"

    def draw(self, context):
        layout = self.layout
        project = context.scene.rab
        layout.prop(project, "export_scope", text="Export")
        layout.prop(project, "export_target", text="Ready For")
        layout.prop(project, "package_name", text="Package")
        layout.prop(project, "export_dir", text="Folder")
        layout.operator("rab.export_package", icon="EXPORT")
        if project.last_export:
            _note(layout, context, f"Last: {project.last_export}", "CHECKMARK")


def _note(layout, context, text, icon="NONE"):
    """Draw text wrapped to the sidebar width, so it is never cut off."""
    region = context.region
    scale = context.preferences.system.ui_scale or 1.0
    width = max(18, int((region.width / scale - 60) / 6.2)) if region is not None else 44
    column = layout.column(align=True)
    column.scale_y = 0.85
    for index, line in enumerate(_wrap(text, width)):
        column.label(text=line, icon=icon if index == 0 else "BLANK1" if icon != "NONE" else "NONE")


def _wrap(text, width=44):
    words, lines, line = text.split(), [], ""
    for word in words:
        if len(line) + len(word) + 1 > width and line:
            lines.append(line)
            line = word
        else:
            line = f"{line} {word}".strip()
    if line:
        lines.append(line)
    return lines


class _LastReport:
    """Readiness computed after an explicit check. Panel drawing never runs validation."""

    _by_scene = {}

    @classmethod
    def store(cls, scene, statuses):
        cls._by_scene[scene.name] = (A.model_version(), statuses)

    @classmethod
    def get(cls, scene):
        """Return statuses only while the model is unchanged since they were computed."""
        entry = cls._by_scene.get(scene.name)
        if entry is None or entry[0] != A.model_version():
            return None
        return entry[1]

    @classmethod
    def clear(cls):
        cls._by_scene.clear()


def refresh_readiness(scene):
    _project, report = validate_scene(scene)
    try:
        records = json.loads(scene.rab.readiness_json)
    except ValueError:
        records = {}
    statuses = evaluate(report, records if isinstance(records, dict) else {})
    _LastReport.store(scene, statuses)
    return statuses


CLASSES = (
    RAB_UL_modules,
    RAB_UL_links,
    RAB_UL_components,
    RAB_UL_joints,
    RAB_UL_frames,
    RAB_UL_issues,
    RAB_PT_main,
    RAB_PT_links,
    RAB_PT_components,
    RAB_PT_joint,
    RAB_PT_frames,
    RAB_PT_assembly,
    RAB_PT_display,
    RAB_PT_mass_summary,
    RAB_PT_readiness,
    RAB_PT_export,
)


def register():
    for cls in CLASSES:
        bpy.utils.register_class(cls)


def unregister():
    _LastReport.clear()
    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)


__all__ = ["CLASSES", "Severity", "refresh_readiness"]

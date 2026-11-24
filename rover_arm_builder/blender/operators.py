"""Blender operators. Scene-changing operators support undo and redo."""

import json
import os
from pathlib import Path

import bpy
from bpy.props import BoolProperty, EnumProperty, FloatProperty, FloatVectorProperty, IntProperty, StringProperty
from bpy.types import Operator
from mathutils import Matrix, Vector

from ..core import model as X
from ..core.readiness import Stage, evaluate, record_stage
from ..core.billee_template import build_billee_arm, build_billee_assembly, build_billee_project
from ..core.validate import Severity, Target
from ..exporters.billee import WiringError, analyze, write_arm_only, write_dropin
from ..exporters.billee_arm import analyze_arm
from ..exporters.package import PackageConflict, write_package
from ..exporters.urdf import ExportBlocked
from . import authoring as A
from .extract import extract_project, validate_scene
from .props import ACTUATION, FRAME_KINDS, JOINT_TYPES, MODULE_KINDS, SHAPES

UNDO = {"REGISTER", "UNDO"}
_billee_checked_version = None


def billee_status_is_current():
    """True while the stored drop-in check still describes the current model."""
    return _billee_checked_version == A.model_version()


def active_module(context):
    project = context.scene.rab
    if 0 <= project.active_module < len(project.modules):
        return project.modules[project.active_module]
    return None


def active_link(module):
    if module is not None and 0 <= module.active_link < len(module.links):
        return module.links[module.active_link]
    return None


def active_component(link):
    if link is not None and 0 <= link.active_component < len(link.components):
        return link.components[link.active_component]
    return None


def active_joint(module):
    if module is not None and 0 <= module.active_joint < len(module.joints):
        return module.joints[module.active_joint]
    return None


def active_frame(module):
    if module is not None and 0 <= module.active_frame < len(module.frames):
        return module.frames[module.active_frame]
    return None


def _billee_selection(data):
    """The one module to export, or None for the complete assembly."""
    if data.export_format != "billee" or data.export_scope != "active":
        return None
    if 0 <= data.active_module < len(data.modules):
        return [data.modules[data.active_module].uid]
    return None


def run_validation(scene):
    """Validate the scene, publish its issues to the interface, and return the report."""
    global _billee_checked_version
    order = {Severity.ERROR: 0, Severity.WARNING: 1, Severity.INFO: 2}
    data = scene.rab
    selected = _billee_selection(data)
    model, report = validate_scene(scene, selected)
    if data.export_format == "billee":
        scoped = model if selected is None else X.Project(model.name, [model.module(selected[0])])
        if selected is not None and scoped.modules[0].kind is X.ModuleKind.ARM:
            arm = analyze_arm(scoped, report.robot) if report.robot is not None else None
            if arm is not None:
                report.issues.extend(arm.issues)
            ready = arm is not None and arm.ok and report.ok(Target.PHYSICS)
            data.billee_status = "Ready to export the arm's MoveIt package" if ready else "Arm has issues to fix"
        else:
            plan = analyze(scoped, report)
            report.issues.extend(plan.issues)
            blockers = len(plan.errors) + len(report.blockers(Target.PHYSICS))
            label = "Ready: rover and arm packages" if plan.arm is not None else "Ready to replace robot_description"
            data.billee_status = label if plan.ready else f"{blockers} issue(s) to fix"
        _billee_checked_version = A.model_version()
    data.issues.clear()
    for issue in sorted(report.issues, key=lambda i: (order[i.severity], i.code, i.message)):
        item = data.issues.add()
        item.severity = issue.severity.value
        item.code = issue.code
        item.entity = issue.entity_id
        item.message = issue.message
        item.action = issue.action
        item.blocks = ", ".join(sorted(t.value for t in issue.blocks))
    data.active_issue = 0
    return report


def readiness(scene, report):
    try:
        records = json.loads(scene.rab.readiness_json)
    except ValueError:
        records = {}
    return evaluate(report, records if isinstance(records, dict) else {})


def focus_entity(context, uid):
    """Select the entity behind a validation issue and open its settings."""
    scene = context.scene
    project = scene.rab
    layer = context.view_layer

    def show(module_index, module, link=None):
        project.active_module = module_index
        project.ui_tab = module.kind if module.kind in {"rover", "arm"} else "assembly"
        if link is not None:
            A.select_link_objects(layer, link)

    for mount_index, mount in enumerate(project.mounts):
        if mount.uid == uid:
            project.active_mount = mount_index
            project.ui_tab = "assembly"
            return True
    for module_index, module in enumerate(project.modules):
        if module.uid == uid:
            show(module_index, module)
            return True
        for joint_index, joint in enumerate(module.joints):
            if joint.uid == uid:
                link = A.find(module.links, joint.child_uid)
                show(module_index, module, link)
                module.active_joint = joint_index
                return True
        for frame_index, frame in enumerate(module.frames):
            if frame.uid == uid:
                show(module_index, module, A.find(module.links, frame.link_uid))
                module.active_frame = frame_index
                return True
        for link_index, link in enumerate(module.links):
            if link.uid == uid:
                show(module_index, module, link)
                module.active_link = link_index
                return True
            for component_index, component in enumerate(link.components):
                if component.uid == uid or any(p.uid == uid for p in component.collisions):
                    show(module_index, module, link)
                    module.active_link = link_index
                    link.active_component = component_index
                    return True
    return False


class RAB_OT_new_module(Operator):
    bl_idname = "rab.new_module"
    bl_label = "New Module"
    bl_description = "Create a rover, arm, or accessory module with a root link"
    bl_options = UNDO

    kind: EnumProperty(name="Type", items=MODULE_KINDS, default="rover")
    module_name: StringProperty(name="Name", default="")
    prefix: StringProperty(name="Export Prefix", default="", description="Prepended to exported names")

    def invoke(self, context, event):
        project = context.scene.rab
        count = sum(1 for m in project.modules if m.kind == self.kind) + 1
        self.module_name = f"{self.kind.capitalize()} {count}"
        self.prefix = f"{self.kind}_" if count == 1 else f"{self.kind}{count}_"
        return context.window_manager.invoke_props_dialog(self)

    def execute(self, context):
        project = context.scene.rab
        prefix = self.prefix or f"{self.kind}_"
        if any(m.prefix == prefix for m in project.modules):
            self.report({"ERROR"}, f"Prefix {prefix} is already used by another module.")
            return {"CANCELLED"}
        A.create_module(context.scene, self.module_name or self.kind.capitalize(), self.kind, prefix)
        project.ui_tab = self.kind if self.kind in {"rover", "arm"} else "assembly"
        return {"FINISHED"}


class RAB_OT_duplicate_module(Operator):
    bl_idname = "rab.duplicate_module"
    bl_label = "Duplicate Module"
    bl_description = "Make an editable copy with new IDs and a unique export prefix. The source is unchanged"
    bl_options = UNDO

    @classmethod
    def poll(cls, context):
        return active_module(context) is not None

    def execute(self, context):
        module = A.duplicate_module(context.scene, context.scene.rab.active_module)
        self.report({"INFO"}, f"Created {module.name} with prefix {module.prefix}")
        return {"FINISHED"}


class RAB_OT_add_link(Operator):
    bl_idname = "rab.add_link"
    bl_label = "Add Link"
    bl_description = "Add a child link connected to the selected link by a joint"
    bl_options = UNDO

    link_name: StringProperty(name="Link Name", default="link")
    joint_type: EnumProperty(name="Motion", items=JOINT_TYPES, default="revolute")
    actuation: EnumProperty(name="Physical Actuation", items=ACTUATION, default="actuated")
    location: FloatVectorProperty(name="Joint Position", size=3, subtype="TRANSLATION",
                                  description="Joint position in the parent link frame")
    axis: FloatVectorProperty(name="Axis", size=3, subtype="XYZ", default=(0, 0, 1))

    @classmethod
    def poll(cls, context):
        return active_link(active_module(context)) is not None

    def invoke(self, context, event):
        module = active_module(context)
        self.link_name = f"link_{len(module.links)}"
        return context.window_manager.invoke_props_dialog(self)

    def execute(self, context):
        module = active_module(context)
        parent = active_link(module)
        if any(l.name == self.link_name for l in module.links):
            self.report({"ERROR"}, f"This module already has a link named {self.link_name}.")
            return {"CANCELLED"}
        if self.joint_type != "fixed" and Vector(self.axis).length < 1e-9:
            self.report({"ERROR"}, "The joint axis cannot be zero.")
            return {"CANCELLED"}
        _link, joint = A.add_link(module, self.link_name, parent, f"{self.link_name}_joint", self.joint_type,
                                  Matrix.Translation(self.location), tuple(self.axis))
        joint.actuation = "fixed" if self.joint_type == "fixed" else (
            self.actuation if self.actuation != "fixed" else "passive")
        return {"FINISHED"}


class RAB_OT_remove_link(Operator):
    bl_idname = "rab.remove_link"
    bl_label = "Remove Link"
    bl_description = "Remove the selected link if it has no child links"
    bl_options = UNDO

    @classmethod
    def poll(cls, context):
        return active_link(active_module(context)) is not None

    def execute(self, context):
        module = active_module(context)
        try:
            A.remove_leaf_link(context.scene, module, active_link(module).uid)
        except ValueError as exc:
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        return {"FINISHED"}


class RAB_OT_add_component(Operator):
    bl_idname = "rab.add_component"
    bl_label = "Add Component"
    bl_description = "Add a physical part to the selected link"
    bl_options = UNDO

    component_name: StringProperty(name="Name", default="component")
    shape: EnumProperty(name="Shape", items=SHAPES[:3], default="BOX")
    preset: EnumProperty(
        name="Part Type",
        items=[("generic", "Generic", "Any part"), ("body", "Chassis / Body", "Main structure"),
               ("wheel", "Wheel", "Cylinder with its axle along Y"),
               ("motor", "Motor Housing", "Motor body on the carrying link"),
               ("sensor", "Sensor / Equipment", "Small equipment"), ("antenna", "Antenna", "Thin mast")],
        default="generic",
    )

    @classmethod
    def poll(cls, context):
        return active_link(active_module(context)) is not None

    def invoke(self, context, event):
        self.component_name = f"component_{len(active_link(active_module(context)).components)}"
        return context.window_manager.invoke_props_dialog(self)

    def execute(self, context):
        module = active_module(context)
        link = active_link(module)
        presets = {
            "body": ("BOX", {"size": (0.4, 0.3, 0.1)}, None),
            "wheel": ("CYLINDER", {"radius": 0.08, "length": 0.05}, (1.5707963267948966, 0, 0)),
            "motor": ("CYLINDER", {"radius": 0.03, "length": 0.06}, (1.5707963267948966, 0, 0)),
            "sensor": ("BOX", {"size": (0.05, 0.05, 0.03)}, None),
            "antenna": ("CYLINDER", {"radius": 0.008, "length": 0.25}, None),
        }
        shape, dims, rotation = presets.get(self.preset, (self.shape, {}, None))
        local = None
        if rotation is not None:
            from mathutils import Euler

            local = Euler(rotation, "XYZ").to_matrix().to_4x4()
        A.add_component(module, link, self.component_name, shape, local=local, **dims)
        return {"FINISHED"}


class RAB_OT_adopt_object(Operator):
    bl_idname = "rab.adopt_object"
    bl_label = "Assign Selected Mesh"
    bl_description = "Use the active mesh as a component of the selected link. Mass needs a manual value"
    bl_options = UNDO

    @classmethod
    def poll(cls, context):
        obj = context.active_object
        return (active_link(active_module(context)) is not None and obj is not None
                and obj.type == "MESH" and obj.rab.kind == "NONE")

    def execute(self, context):
        module = active_module(context)
        obj = context.active_object
        A.add_component(module, active_link(module), obj.name.replace(" ", "_"), "MESH", obj=obj)
        return {"FINISHED"}


class RAB_OT_remove_component(Operator):
    bl_idname = "rab.remove_component"
    bl_label = "Remove Component"
    bl_description = "Delete the selected component and its scene objects"
    bl_options = UNDO

    @classmethod
    def poll(cls, context):
        return active_component(active_link(active_module(context))) is not None

    def execute(self, context):
        link = active_link(active_module(context))
        A.remove_component(link, link.active_component)
        return {"FINISHED"}


class RAB_OT_add_collision(Operator):
    bl_idname = "rab.add_collision"
    bl_label = "Add Collision Shape"
    bl_description = "Add a primitive collision proxy. It never changes the estimated mass"
    bl_options = UNDO

    shape: EnumProperty(name="Shape", items=SHAPES[:3], default="BOX")

    @classmethod
    def poll(cls, context):
        return active_component(active_link(active_module(context))) is not None

    def execute(self, context):
        module = active_module(context)
        link = active_link(module)
        component = active_component(link)
        name = f"{component.name}_collision_{len(component.collisions) + 1}"
        local = component.object.matrix_basis.copy() if component.object is not None else None
        A.add_collision_proxy(module, link, component, name, self.shape, size=tuple(component.size),
                              radius=component.radius, length=component.length, local=local)
        return {"FINISHED"}


class RAB_OT_add_frame(Operator):
    bl_idname = "rab.add_frame"
    bl_label = "Add Frame"
    bl_description = "Add a named mounting, tool, or reference frame on the selected link"
    bl_options = UNDO

    frame_name: StringProperty(name="Name", default="mount")
    kind: EnumProperty(name="Type", items=FRAME_KINDS, default="mount")

    @classmethod
    def poll(cls, context):
        return active_link(active_module(context)) is not None

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)

    def execute(self, context):
        module = active_module(context)
        if any(f.name == self.frame_name for f in module.frames):
            self.report({"ERROR"}, f"This module already has a frame named {self.frame_name}.")
            return {"CANCELLED"}
        A.add_frame(module, active_link(module), self.frame_name, self.kind)
        return {"FINISHED"}


class RAB_OT_frame_place(Operator):
    bl_idname = "rab.frame_place"
    bl_label = "Place Frame"
    bl_description = "Move the selected attachment frame"
    bl_options = UNDO

    mode: EnumProperty(
        name="Place At",
        items=[("CURSOR", "3D Cursor", "Use the 3D cursor position and orientation"),
               ("ORIGIN", "Active Object Origin", "Use the active object's origin and axes"),
               ("CENTER", "Active Geometry Center", "Use the bounding-box center, with the object's axes")],
        default="CURSOR",
    )

    @classmethod
    def poll(cls, context):
        return active_frame(active_module(context)) is not None

    def execute(self, context):
        module = active_module(context)
        frame = active_frame(module)
        owner = A.find(module.links, frame.link_uid)
        target = target_matrix(context, self.mode)
        if target is None:
            self.report({"ERROR"}, "Select an object to snap to.")
            return {"CANCELLED"}
        context.view_layer.update()
        A.set_pose(frame.pose, A.rigid(owner.frame_object.matrix_world).inverted() @ target)
        A.apply_all(context.scene)
        return {"FINISHED"}


def target_matrix(context, mode):
    if mode == "CURSOR":
        return A.rigid(context.scene.cursor.matrix)
    obj = context.active_object
    if obj is None:
        return None
    world = A.rigid(obj.matrix_world)
    if mode == "CENTER":
        corners = [Vector(c) for c in obj.bound_box]
        center = sum(corners, Vector((0, 0, 0))) / 8
        world = world.copy()
        world.translation = obj.matrix_world @ center
    return world


class RAB_OT_reframe_link(Operator):
    bl_idname = "rab.reframe_link"
    bl_label = "Move Link Frame (Keep Geometry)"
    bl_description = ("Move the link frame while the geometry, child joints, frames, and center of mass "
                      "stay where they are in the world")
    bl_options = UNDO

    mode: EnumProperty(
        name="Move To",
        items=[("CURSOR", "3D Cursor", "Use the 3D cursor position and orientation"),
               ("ORIGIN", "Active Object Origin", "Use the active object's origin and axes"),
               ("CENTER", "Active Geometry Center", "Use the bounding-box center, with the object's axes")],
        default="CURSOR",
    )

    @classmethod
    def poll(cls, context):
        module = active_module(context)
        return active_link(module) is not None and not A.is_previewing(module)

    def execute(self, context):
        module = active_module(context)
        target = target_matrix(context, self.mode)
        if target is None:
            self.report({"ERROR"}, "Select an object to snap to.")
            return {"CANCELLED"}
        A.reframe_link(context.scene, module, active_link(module), target)
        return {"FINISHED"}


class RAB_OT_copy_pose(Operator):
    bl_idname = "rab.copy_pose"
    bl_label = "Copy Frame Pose"
    bl_description = "Copy the selected frame's world pose"

    clipboard = None

    @classmethod
    def poll(cls, context):
        frame = active_frame(active_module(context))
        return frame is not None and frame.object is not None

    def execute(self, context):
        context.view_layer.update()
        frame = active_frame(active_module(context))
        RAB_OT_copy_pose.clipboard = A.rigid(frame.object.matrix_world)
        return {"FINISHED"}


class RAB_OT_paste_pose(Operator):
    bl_idname = "rab.paste_pose"
    bl_label = "Paste Frame Pose"
    bl_description = "Place the selected frame at the copied world pose"
    bl_options = UNDO

    @classmethod
    def poll(cls, context):
        return RAB_OT_copy_pose.clipboard is not None and active_frame(active_module(context)) is not None

    def execute(self, context):
        module = active_module(context)
        frame = active_frame(module)
        owner = A.find(module.links, frame.link_uid)
        context.view_layer.update()
        A.set_pose(frame.pose, A.rigid(owner.frame_object.matrix_world).inverted() @ RAB_OT_copy_pose.clipboard)
        A.apply_all(context.scene)
        return {"FINISHED"}


class RAB_OT_preview_reset(Operator):
    bl_idname = "rab.preview_reset"
    bl_label = "Return to Zero"
    bl_description = "Return every joint in this module to its reference pose"
    bl_options = UNDO

    @classmethod
    def poll(cls, context):
        return active_module(context) is not None

    def execute(self, context):
        A.reset_preview(active_module(context))
        return {"FINISHED"}


class RAB_OT_joint_drag(Operator):
    bl_idname = "rab.joint_drag"
    bl_label = "Drag Joint Preview"
    bl_description = "Move the mouse to preview the joint. Click to keep, Esc or right-click to cancel"
    bl_options = {"REGISTER", "UNDO", "BLOCKING"}

    value: FloatProperty(name="Value")

    @classmethod
    def poll(cls, context):
        joint = active_joint(active_module(context))
        return joint is not None and joint.type != "fixed"

    def invoke(self, context, event):
        joint = active_joint(active_module(context))
        self.start_mouse = event.mouse_x
        self.start_value = joint.preview
        context.window_manager.modal_handler_add(self)
        context.workspace.status_text_set("Move mouse: preview | Click: keep | Esc: cancel")
        return {"RUNNING_MODAL"}

    def modal(self, context, event):
        module = active_module(context)
        joint = active_joint(module)
        if event.type == "MOUSEMOVE":
            step = 0.01 if joint.type != "prismatic" else 0.001
            value = self.start_value + (event.mouse_x - self.start_mouse) * step
            if joint.has_position_limits:
                value = max(joint.lower, min(joint.upper, value))
            joint.preview = value
            if context.area is not None:
                context.area.tag_redraw()
        elif event.type == "LEFTMOUSE" and event.value == "PRESS":
            self.value = joint.preview
            context.workspace.status_text_set(None)
            return {"FINISHED"}
        elif event.type in {"RIGHTMOUSE", "ESC"} and event.value == "PRESS":
            joint.preview = self.start_value
            context.workspace.status_text_set(None)
            if context.area is not None:
                context.area.tag_redraw()
            return {"CANCELLED"}
        return {"RUNNING_MODAL"}

    def execute(self, context):
        active_joint(active_module(context)).preview = self.value
        return {"FINISHED"}


class RAB_OT_apply_zero(Operator):
    bl_idname = "rab.apply_zero"
    bl_label = "Set Current Position as New Zero"
    bl_description = ("Redefine this joint's zero at the previewed position. The joint origin changes, limits "
                      "and named poses shift so the physical range stays the same")
    bl_options = UNDO

    @classmethod
    def poll(cls, context):
        joint = active_joint(active_module(context))
        return joint is not None and joint.type != "fixed" and abs(joint.preview) > 1e-9

    def invoke(self, context, event):
        return context.window_manager.invoke_confirm(self, event)

    def execute(self, context):
        module = active_module(context)
        A.apply_new_zero(module, active_joint(module))
        return {"FINISHED"}


class RAB_OT_pose_store(Operator):
    bl_idname = "rab.pose_store"
    bl_label = "Save Named Pose"
    bl_description = "Save the current preview values as a named pose, such as home or stow"
    bl_options = UNDO

    pose_name: StringProperty(name="Pose Name", default="home")

    @classmethod
    def poll(cls, context):
        return active_module(context) is not None

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)

    def execute(self, context):
        A.store_named_pose(active_module(context), self.pose_name)
        return {"FINISHED"}


class RAB_OT_pose_apply(Operator):
    bl_idname = "rab.pose_apply"
    bl_label = "Preview Pose"
    bl_description = "Show this named pose. It does not change the exported zero pose"
    bl_options = UNDO

    pose_name: StringProperty()

    def execute(self, context):
        if not A.apply_named_pose(active_module(context), self.pose_name):
            self.report({"ERROR"}, f"Pose {self.pose_name} no longer exists.")
            return {"CANCELLED"}
        return {"FINISHED"}


class RAB_OT_mount_preview(Operator):
    bl_idname = "rab.mount_preview"
    bl_label = "Preview Alignment"
    bl_description = "Show the child module aligned to the parent frame without mounting it"
    bl_options = UNDO

    def execute(self, context):
        try:
            A.begin_mount_preview(context.scene)
        except ValueError as exc:
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        return {"FINISHED"}


class RAB_OT_mount_cancel(Operator):
    bl_idname = "rab.mount_cancel"
    bl_label = "Cancel"
    bl_description = "Discard the preview and restore the module's previous placement"
    bl_options = UNDO

    def execute(self, context):
        A.cancel_mount_preview(context.scene)
        return {"FINISHED"}


class RAB_OT_mount_confirm(Operator):
    bl_idname = "rab.mount_confirm"
    bl_label = "Confirm Mount"
    bl_description = "Attach the child module with a fixed joint at the previewed alignment"
    bl_options = UNDO

    def execute(self, context):
        try:
            A.confirm_mount_preview(context.scene)
        except ValueError as exc:
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        run_validation(context.scene)
        return {"FINISHED"}


class RAB_OT_detach(Operator):
    bl_idname = "rab.detach"
    bl_label = "Detach"
    bl_description = "Remove this mount. The module keeps its geometry, joints, and world position"
    bl_options = UNDO

    uid: StringProperty()

    def execute(self, context):
        A.detach_mount(context.scene, self.uid)
        run_validation(context.scene)
        return {"FINISHED"}


class RAB_OT_validate(Operator):
    bl_idname = "rab.validate"
    bl_label = "Check Model"
    bl_description = "Check the model for each readiness target. This does not run ROS, MoveIt, or Gazebo"

    def execute(self, context):
        report = run_validation(context.scene)
        from .panels import refresh_readiness

        refresh_readiness(context.scene)
        errors = sum(1 for i in report.issues if i.severity is Severity.ERROR)
        warnings = sum(1 for i in report.issues if i.severity is Severity.WARNING)
        self.report({"WARNING"} if errors else {"INFO"}, f"{errors} error(s), {warnings} warning(s)")
        return {"FINISHED"}


class RAB_OT_focus_issue(Operator):
    bl_idname = "rab.focus_issue"
    bl_label = "Show"
    bl_description = "Select the affected item and open its settings"
    bl_options = {"REGISTER", "UNDO"}

    entity: StringProperty()

    def execute(self, context):
        if not focus_entity(context, self.entity):
            self.report({"WARNING"}, "That item no longer exists. Run Check Model again.")
            return {"CANCELLED"}
        return {"FINISHED"}


def scoped_project(project, scope, active_uid):
    if scope != "active":
        return project
    module = project.module(active_uid)
    return X.Project(name=f"{project.name}_{module.kind.value}", modules=[module], mounts=[])


class RAB_OT_export_package(Operator):
    bl_idname = "rab.export_package"
    bl_label = "Export ROS 2 Package"
    bl_description = "Validate, then write a portable ROS 2 description package. The scene is not changed"

    @classmethod
    def poll(cls, context):
        return len(context.scene.rab.modules) > 0

    def execute(self, context):
        scene = context.scene
        data = scene.rab
        if data.mount_preview_active:
            self.report({"ERROR"}, "Confirm or cancel the mount preview before exporting.")
            return {"CANCELLED"}
        folder = os.path.expanduser(bpy.path.abspath(data.export_dir))
        if not folder or folder.startswith("//") or not os.path.isabs(folder):
            self.report({"ERROR"}, "Save the .blend file or choose an absolute export folder.")
            return {"CANCELLED"}
        if data.export_format == "billee":
            return self.export_billee(context, folder)
        window_manager = context.window_manager
        window_manager.progress_begin(0, 3)
        try:
            window_manager.progress_update(1)
            report = run_validation(scene)
            target = Target(data.export_target)
            if not report.ok(target):
                count = len(report.blockers(target))
                self.report({"ERROR"}, f"Export blocked: {count} issue(s) for {target.value}. See Readiness.")
                return {"CANCELLED"}
            project, issues = extract_project(scene)
            if issues:
                self.report({"ERROR"}, f"Export blocked: {issues[0][2]}")
                return {"CANCELLED"}
            active = data.modules[data.active_module].uid if len(data.modules) else ""
            scoped = scoped_project(project, data.export_scope, active)
            window_manager.progress_update(2)
            destination = Path(folder) / data.package_name
            _path, result = write_package(scoped, destination, data.package_name, target)
        except (ExportBlocked, PackageConflict, ValueError, OSError) as exc:
            self.report({"ERROR"}, f"Export failed: {exc}")
            return {"CANCELLED"}
        finally:
            window_manager.progress_end()

        if data.export_scope == "assembly":
            try:
                records = json.loads(data.readiness_json)
            except ValueError:
                records = {}
            for ready in Target:
                if result.ok(ready):
                    records = record_stage(records, result.robot, ready, Stage.EXPORTED)
                    records = record_stage(records, result.robot, ready, Stage.INTERNALLY_VALIDATED)
            data.readiness_json = json.dumps(records, sort_keys=True)
        data.last_export = str(destination)
        self.report({"INFO"}, f"Exported to {destination}")
        return {"FINISHED"}


def _export_billee(self, context, folder):
    """Write a complete robot_description folder. The software repo is only read."""
    scene = context.scene
    data = scene.rab
    window_manager = context.window_manager
    window_manager.progress_begin(0, 3)
    try:
        window_manager.progress_update(1)
        report = run_validation(scene)
        project, issues = extract_project(scene)
        selected = _billee_selection(data)
        owners = None if selected is None else {selected[0]}
        relevant = issues if owners is None else [i for i in report.issues if i.severity is Severity.ERROR]
        if relevant or any(i.severity is Severity.ERROR for i in report.issues):
            self.report({"ERROR"}, f"Export blocked: {data.billee_status}. See Readiness.")
            return {"CANCELLED"}
        window_manager.progress_update(2)
        if selected is not None:
            module = project.module(selected[0])
            project = X.Project(project.name, [module])
            if module.kind is X.ModuleKind.ARM:
                path, _plan = write_arm_only(project, folder)
                data.last_export = str(path)
                self.report({"INFO"}, f"Exported the standalone arm package to {path}")
                return {"FINISHED"}
        wiring = bpy.path.abspath(data.billee_repo) if data.billee_repo else None
        result = write_dropin(project, folder, wiring)
    except (ExportBlocked, PackageConflict, WiringError, ValueError, OSError) as exc:
        self.report({"ERROR"}, f"Export failed: {exc}")
        return {"CANCELLED"}
    finally:
        window_manager.progress_end()
    data.last_export = str(result.path.parent if result.moveit_path else result.path)
    packages = "robot_description and billee_arm_moveit_config" if result.moveit_path else "robot_description"
    self.report({"INFO"}, f"Exported {packages} to {result.path.parent} using the {result.wiring}")
    return {"FINISHED"}


RAB_OT_export_package.export_billee = _export_billee


class RAB_OT_create_billee(Operator):
    bl_idname = "rab.create_billee"
    bl_label = "Create BILLEE Rover"
    bl_description = "Start from BILLEE's six-wheel layout. Chassis shapes and masses are placeholders"
    bl_options = UNDO

    with_arm: BoolProperty(name="Include Arm", default=False,
                           description="Also add the five-axis arm, mounted on the rover")

    @classmethod
    def poll(cls, context):
        return not len(context.scene.rab.modules)

    def execute(self, context):
        from .demo_scene import build_demo

        build_demo(context.scene, build_billee_assembly() if self.with_arm else build_billee_project())
        context.scene.rab.export_format = "billee"
        context.scene.rab.package_name = "robot_description"
        run_validation(context.scene)
        self.report({"INFO"}, "BILLEE model created. Shapes, masses, and limits are placeholders.")
        return {"FINISHED"}


class RAB_OT_add_billee_arm(Operator):
    bl_idname = "rab.add_billee_arm"
    bl_label = "Add BILLEE Arm"
    bl_description = "Add the five-axis arm as its own module. Mount it to a rover in the Assembly tab"
    bl_options = UNDO

    @classmethod
    def poll(cls, context):
        return not any(m.kind == "arm" for m in context.scene.rab.modules)

    def execute(self, context):
        from .demo_scene import add_module

        add_module(context.scene, build_billee_arm())
        context.scene.rab.export_format = "billee"
        context.scene.rab.ui_tab = "arm"
        return {"FINISHED"}


class RAB_OT_setup_workspace(Operator):
    bl_idname = "rab.setup_workspace"
    bl_label = "Open Robot Builder Workspace"
    bl_description = "Create or open a workspace with the Robot sidebar visible"

    def execute(self, context):
        existing = bpy.data.workspaces.get("Robot Builder")
        if existing is None:
            before = set(bpy.data.workspaces)
            bpy.ops.workspace.duplicate()
            created = next(iter(set(bpy.data.workspaces) - before), None)
            if created is None:
                self.report({"ERROR"}, "Blender could not create the workspace.")
                return {"CANCELLED"}
            created.name = "Robot Builder"
            existing = created
        context.window.workspace = existing
        for screen in existing.screens:
            for area in screen.areas:
                if area.type == "VIEW_3D":
                    area.spaces.active.show_region_ui = True
                    area.spaces.active.shading.color_type = "OBJECT"
        self.report({"INFO"}, "Open the Robot tab in the viewport sidebar.")
        return {"FINISHED"}


class RAB_OT_create_demo(Operator):
    bl_idname = "rab.create_demo"
    bl_label = "Create Demo Robot"
    bl_description = "Add an example rover and arm. All dimensions and masses are illustrative"
    bl_options = UNDO

    @classmethod
    def poll(cls, context):
        return not len(context.scene.rab.modules)

    def execute(self, context):
        from .demo_scene import build_demo

        build_demo(context.scene)
        context.scene.rab.export_format = "generic"
        run_validation(context.scene)
        self.report({"INFO"}, "Demo created. Values are illustrative, not measurements.")
        return {"FINISHED"}


CLASSES = (
    RAB_OT_create_billee,
    RAB_OT_add_billee_arm,
    RAB_OT_create_demo,
    RAB_OT_new_module,
    RAB_OT_duplicate_module,
    RAB_OT_add_link,
    RAB_OT_remove_link,
    RAB_OT_add_component,
    RAB_OT_adopt_object,
    RAB_OT_remove_component,
    RAB_OT_add_collision,
    RAB_OT_add_frame,
    RAB_OT_frame_place,
    RAB_OT_reframe_link,
    RAB_OT_copy_pose,
    RAB_OT_paste_pose,
    RAB_OT_preview_reset,
    RAB_OT_joint_drag,
    RAB_OT_apply_zero,
    RAB_OT_pose_store,
    RAB_OT_pose_apply,
    RAB_OT_mount_preview,
    RAB_OT_mount_cancel,
    RAB_OT_mount_confirm,
    RAB_OT_detach,
    RAB_OT_validate,
    RAB_OT_focus_issue,
    RAB_OT_export_package,
    RAB_OT_setup_workspace,
)


def register():
    for cls in CLASSES:
        bpy.utils.register_class(cls)


def unregister():
    RAB_OT_copy_pose.clipboard = None
    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)


__all__ = ["CLASSES", "IntProperty"]

"""Versioned scene data saved in the .blend file.

Stored units: lengths are Blender units, so Blender's unit-aware fields display them
correctly. Meters = Blender units * scene.unit_settings.scale_length, applied once in
extract.py. Mass is kg, density is kg/m^3, inertia is kg*m^2, and angles are radians.
"""

import uuid

import bpy
from bpy.props import (
    BoolProperty,
    CollectionProperty,
    EnumProperty,
    FloatProperty,
    FloatVectorProperty,
    IntProperty,
    PointerProperty,
    StringProperty,
)
from bpy.types import Collection, Object, PropertyGroup

SCHEMA_VERSION = 1


def new_uid():
    return uuid.uuid4().hex


def _pose_changed(self, context):
    from . import authoring

    authoring.on_metadata_changed(self.id_data)


def _joint_changed(self, context):
    from . import authoring

    authoring.on_metadata_changed(self.id_data)


def _preview_changed(self, context):
    from . import authoring

    authoring.on_preview_changed(self.id_data)


def _shape_changed(self, context):
    from . import authoring

    authoring.on_shape_changed(self)


def _link_selected(self, context):
    from . import authoring

    authoring.on_active_link_changed(self, context)


def _mount_preview_changed(self, context):
    from . import authoring

    authoring.apply_mount_preview(self.id_data)


_enum_cache = {}


def _cached(key, items):
    """Blender requires dynamic enum strings to stay referenced from Python."""
    _enum_cache[key] = items or [("NONE", "None available", "")]
    return _enum_cache[key]


def _module_items(self, kinds, key):
    return _cached(key, [(m.uid, m.name, f"{m.kind} module") for m in self.modules if m.kind in kinds])


def _parent_module_items(self, context):
    return _module_items(self, {"rover", "accessory", "arm"}, "parent_modules")


def _child_module_items(self, context):
    return _module_items(self, {"arm", "accessory"}, "child_modules")


def _frame_items(self, module_uid, key):
    module = next((m for m in self.modules if m.uid == module_uid), None)
    frames = [] if module is None else [(f.uid, f.name, f"{f.kind} frame") for f in module.frames]
    return _cached(key, frames)


def _parent_frame_items(self, context):
    return _frame_items(self, self.mount_parent_module, "parent_frames")


def _child_frame_items(self, context):
    return _frame_items(self, self.mount_child_module, "child_frames")


JOINT_TYPES = [
    ("fixed", "Rigidly attached (fixed)", "No relative motion between parent and child"),
    ("revolute", "Rotates within limits (revolute)", "Rotation about the axis between lower and upper limits"),
    ("continuous", "Rotates freely (continuous)", "Unlimited rotation about the axis, such as a wheel"),
    ("prismatic", "Slides within limits (prismatic)", "Linear travel along the axis between limits"),
]
ACTUATION = [
    ("actuated", "Actuated", "Driven by a motor or other actuator"),
    ("passive", "Passive", "Moves freely, with no actuator, such as a suspension pivot"),
    ("fixed", "Fixed", "Does not move"),
]
ROLES = [
    ("arm", "Arm / manipulation", "Planned as part of the arm"),
    ("base_drive", "Base drive", "Drives the rover, such as a wheel motor"),
    ("steering", "Steering", "Steers the rover"),
    ("passive_suspension", "Passive suspension", "Unactuated suspension motion"),
    ("other", "Other equipment", "Controlled separately from the arm and base"),
    ("unassigned", "Unassigned", "No planning or control role yet"),
]
SHAPES = [
    ("BOX", "Box", "Box with X, Y, Z dimensions"),
    ("CYLINDER", "Cylinder", "Circular cylinder with its axis along local Z"),
    ("SPHERE", "Sphere", "Sphere"),
    ("MESH", "Mesh", "Existing mesh geometry, exported as STL"),
]
MASS_MODES = [
    ("none", "No mass", "Display-only geometry. Blocks physics export for this link"),
    ("measured", "Measured mass", "Weighed mass with a chosen uniform distribution shape"),
    ("estimated", "Estimated mass", "Estimated mass with a chosen uniform distribution shape"),
    ("density", "Density x volume", "Uniform solid density multiplied by the shape volume"),
    ("manual", "Manual override", "Enter mass, center of mass, and inertia directly"),
]
MODULE_KINDS = [
    ("rover", "Rover", "Mobile base module"),
    ("arm", "Arm", "Manipulator module"),
    ("accessory", "Accessory", "Other mountable equipment"),
]
FRAME_KINDS = [
    ("mount", "Mount", "Attachment point between modules"),
    ("tool", "Tool", "End-effector or tool frame"),
    ("reference", "Reference", "Named reference frame"),
]


class RAB_ObjectTag(PropertyGroup):
    """Identifies what a Blender object represents. Duplicates are detected by owner lookup."""

    uid: StringProperty(name="Entity ID", options={"HIDDEN"})
    kind: EnumProperty(
        name="Role",
        items=[
            ("NONE", "None", ""),
            ("LINK", "Link frame", ""),
            ("COMPONENT", "Component", ""),
            ("COLLISION", "Collision proxy", ""),
            ("FRAME", "Attachment frame", ""),
        ],
        default="NONE",
    )


class RAB_Pose(PropertyGroup):
    location: FloatVectorProperty(
        name="Position", size=3, subtype="TRANSLATION", precision=5, update=_pose_changed,
        description="Position relative to the parent frame",
    )
    rotation: FloatVectorProperty(
        name="Rotation (roll, pitch, yaw)", size=3, subtype="EULER", update=_pose_changed,
        description="Fixed-axis rotation: R = Rz(yaw) * Ry(pitch) * Rx(roll)",
    )


class ShapeFields:
    shape: EnumProperty(name="Shape", items=SHAPES, default="BOX", update=_shape_changed)
    size: FloatVectorProperty(
        name="Size", size=3, subtype="XYZ_LENGTH", default=(0.1, 0.1, 0.1), min=1e-6,
        precision=5, update=_shape_changed, description="Box dimensions along local X, Y, Z",
    )
    radius: FloatProperty(
        name="Radius", subtype="DISTANCE", default=0.05, min=1e-6, precision=5, update=_shape_changed,
    )
    length: FloatProperty(
        name="Length", subtype="DISTANCE", default=0.1, min=1e-6, precision=5, update=_shape_changed,
        description="Cylinder length along local Z",
    )
    allow_mesh_fallback: BoolProperty(
        name="Export as Mesh if Deformed",
        description="If scale makes this shape non-analytic, export its true mesh instead of blocking",
        default=False,
    )


class RAB_CollisionProxy(ShapeFields, PropertyGroup):
    """Collision-only geometry. It never contributes mass."""

    uid: StringProperty(options={"HIDDEN"})
    name: StringProperty(name="Name", default="collision")
    object: PointerProperty(type=Object, name="Object")


class RAB_Component(ShapeFields, PropertyGroup):
    uid: StringProperty(options={"HIDDEN"})
    name: StringProperty(name="Name", default="component")
    object: PointerProperty(type=Object, name="Object")
    export_visual: BoolProperty(name="Visual", default=True, description="Export this shape as visual geometry")
    use_as_collision: BoolProperty(
        name="Use Shape for Collision", default=True,
        description="Also export this shape as collision geometry. This does not add mass",
    )
    collisions: CollectionProperty(type=RAB_CollisionProxy)
    active_collision: IntProperty(default=0)
    color: FloatVectorProperty(name="Display Color", size=4, subtype="COLOR", min=0, max=1, default=(0.5, 0.5, 0.55, 1))

    mass_mode: EnumProperty(name="Mass Source", items=MASS_MODES, default="none", update=_joint_changed)
    mass: FloatProperty(name="Mass (kg)", default=0.0, min=0.0, precision=4, update=_joint_changed)
    density: FloatProperty(name="Density (kg/m^3)", default=0.0, min=0.0, precision=2, update=_joint_changed)
    assumption: StringProperty(name="Assumption", default="Uniform-density solid")
    overlap_confirmed: BoolProperty(name="Overlap Is Intended", default=False)
    needs_review: BoolProperty(
        name="Manual Values Need Review", default=False,
        description="Dimensions changed after manual mass values were entered",
    )
    manual_com: FloatVectorProperty(name="Center of Mass", size=3, subtype="TRANSLATION", precision=5)
    ixx: FloatProperty(name="Ixx", precision=7)
    ixy: FloatProperty(name="Ixy", precision=7)
    ixz: FloatProperty(name="Ixz", precision=7)
    iyy: FloatProperty(name="Iyy", precision=7)
    iyz: FloatProperty(name="Iyz", precision=7)
    izz: FloatProperty(name="Izz", precision=7)


class RAB_Link(PropertyGroup):
    uid: StringProperty(options={"HIDDEN"})
    name: StringProperty(name="Name", default="link")
    frame_object: PointerProperty(type=Object, name="Link Frame")
    components: CollectionProperty(type=RAB_Component)
    active_component: IntProperty(default=0)


class RAB_Joint(PropertyGroup):
    uid: StringProperty(options={"HIDDEN"})
    name: StringProperty(name="Name", default="joint")
    type: EnumProperty(name="Motion", items=JOINT_TYPES, default="fixed", update=_joint_changed)
    parent_uid: StringProperty(options={"HIDDEN"})
    child_uid: StringProperty(options={"HIDDEN"})
    origin: PointerProperty(type=RAB_Pose)
    axis: FloatVectorProperty(
        name="Axis", size=3, subtype="XYZ", default=(0, 0, 1), update=_joint_changed,
        description="Motion axis in the joint frame. Normalized on export",
    )
    has_position_limits: BoolProperty(name="Position Limits Set", default=False, update=_joint_changed)
    lower: FloatProperty(name="Lower", default=0.0, precision=4, update=_joint_changed)
    upper: FloatProperty(name="Upper", default=0.0, precision=4, update=_joint_changed)
    has_velocity: BoolProperty(name="Velocity Limit Set", default=False, update=_joint_changed)
    velocity: FloatProperty(name="Velocity Limit", default=0.0, min=0.0, precision=4, update=_joint_changed,
                            description="rad/s for rotation, m/s for sliding")
    has_effort: BoolProperty(name="Effort Limit Set", default=False, update=_joint_changed)
    effort: FloatProperty(name="Effort Limit", default=0.0, min=0.0, precision=4, update=_joint_changed,
                          description="N*m for rotation, N for sliding")
    damping: FloatProperty(name="Damping", default=0.0, min=0.0, precision=4, update=_joint_changed)
    friction: FloatProperty(name="Friction", default=0.0, min=0.0, precision=4, update=_joint_changed)
    actuation: EnumProperty(name="Physical Actuation", items=ACTUATION, default="fixed", update=_joint_changed)
    role: EnumProperty(name="Control Role", items=ROLES, default="unassigned", update=_joint_changed)
    preview: FloatProperty(
        name="Preview", default=0.0, update=_preview_changed,
        description="Temporary display position. Never changes the exported zero pose",
    )


class RAB_Frame(PropertyGroup):
    uid: StringProperty(options={"HIDDEN"})
    name: StringProperty(name="Name", default="frame")
    link_uid: StringProperty(options={"HIDDEN"})
    kind: EnumProperty(name="Type", items=FRAME_KINDS, default="mount")
    pose: PointerProperty(type=RAB_Pose)
    object: PointerProperty(type=Object, name="Frame Object")


class RAB_PoseValue(PropertyGroup):
    joint_uid: StringProperty(options={"HIDDEN"})
    value: FloatProperty(name="Value")


class RAB_NamedPose(PropertyGroup):
    name: StringProperty(name="Name", default="pose")
    values: CollectionProperty(type=RAB_PoseValue)


class RAB_Module(PropertyGroup):
    uid: StringProperty(options={"HIDDEN"})
    name: StringProperty(name="Name", default="Module")
    kind: EnumProperty(name="Type", items=MODULE_KINDS, default="rover")
    prefix: StringProperty(name="Export Prefix", default="", description="Prepended to exported names")
    root_link_uid: StringProperty(options={"HIDDEN"})
    collection: PointerProperty(type=Collection)
    links: CollectionProperty(type=RAB_Link)
    joints: CollectionProperty(type=RAB_Joint)
    frames: CollectionProperty(type=RAB_Frame)
    poses: CollectionProperty(type=RAB_NamedPose)
    active_link: IntProperty(default=0, update=_link_selected)
    active_joint: IntProperty(default=0)
    active_frame: IntProperty(default=0)
    active_pose: IntProperty(default=0)


class RAB_Mount(PropertyGroup):
    uid: StringProperty(options={"HIDDEN"})
    parent_module_uid: StringProperty(options={"HIDDEN"})
    parent_frame_uid: StringProperty(options={"HIDDEN"})
    child_module_uid: StringProperty(options={"HIDDEN"})
    child_frame_uid: StringProperty(options={"HIDDEN"})
    offset: PointerProperty(type=RAB_Pose)


class RAB_Issue(PropertyGroup):
    severity: StringProperty()
    code: StringProperty()
    entity: StringProperty()
    message: StringProperty()
    action: StringProperty()
    blocks: StringProperty()


def _display_changed(self, context):
    from . import authoring

    authoring.apply_display(self.id_data)


class RAB_Project(PropertyGroup):
    schema_version: IntProperty(default=SCHEMA_VERSION, options={"HIDDEN"})
    robot_name: StringProperty(name="Robot Name", default="my_robot")
    modules: CollectionProperty(type=RAB_Module)
    mounts: CollectionProperty(type=RAB_Mount)
    active_module: IntProperty(default=0)
    active_mount: IntProperty(default=0)

    ui_tab: EnumProperty(
        name="Interface",
        items=[("rover", "Rover", "Build the rover"), ("arm", "Arm", "Build the arm"),
               ("assembly", "Assembly", "Mount modules and export")],
        default="rover",
    )
    display_mode: EnumProperty(
        name="Display",
        items=[("both", "Visual + Collision", "Show both"), ("visual", "Visual Only", "Hide collision shapes"),
               ("collision", "Collision Only", "Show only collision shapes")],
        default="both", update=_display_changed,
    )
    show_frames: BoolProperty(name="Frames", default=True, description="Show labeled XYZ frames")
    show_axes: BoolProperty(name="Joint Axes", default=True, description="Show joint axis arrows")
    show_limits: BoolProperty(name="Limits", default=True, description="Show angular arcs and travel guides")
    show_com: BoolProperty(name="Center of Mass", default=True, description="Show component and link COM markers")
    show_collision: BoolProperty(name="Collision Overlay", default=False,
                                 description="Translucent display of collision shapes. This is not a collision test")

    issues: CollectionProperty(type=RAB_Issue)
    active_issue: IntProperty(default=0)
    readiness_json: StringProperty(default="{}", options={"HIDDEN"})
    package_name: StringProperty(name="Package Name", default="my_robot_description")
    export_dir: StringProperty(name="Export Folder", subtype="DIR_PATH", default="//")
    export_scope: EnumProperty(
        name="Export",
        items=[("assembly", "Complete Robot", "All modules and their mounts"),
               ("active", "Active Module Only", "Only the selected rover or arm")],
        default="assembly",
    )
    export_target: EnumProperty(
        name="Require Ready For",
        items=[("visualization", "Visualization", ""), ("planning", "Arm Planning", ""),
               ("physics", "Physics Simulation", "")],
        default="visualization",
    )
    last_export: StringProperty(name="Last Export", default="")
    export_format: EnumProperty(
        name="Export For",
        items=[("billee", "BILLEE Rover (drop-in)", "A complete robot_description folder for billee-software-2027"),
               ("generic", "Generic ROS 2 Package", "A standalone description package with its own launch files")],
        default="billee",
    )
    billee_repo: StringProperty(
        name="Software Repo", subtype="DIR_PATH", default="",
        description="Optional billee-software-2027 folder to read current wiring from. It is never modified. "
                    "Empty finds ~/billee-software-2027, then uses the bundled copy",
    )
    billee_status: StringProperty(default="", options={"HIDDEN"})

    mount_parent_module: EnumProperty(name="Rover / Parent", items=_parent_module_items)
    mount_parent_frame: EnumProperty(name="Parent Frame", items=_parent_frame_items)
    mount_child_module: EnumProperty(name="Arm / Child", items=_child_module_items)
    mount_child_frame: EnumProperty(name="Child Frame", items=_child_frame_items)
    mount_offset_location: FloatVectorProperty(
        name="Offset", size=3, subtype="TRANSLATION", precision=5, update=_mount_preview_changed,
        description="Offset expressed in the parent mounting frame",
    )
    mount_offset_rotation: FloatVectorProperty(
        name="Offset Rotation", size=3, subtype="EULER", update=_mount_preview_changed,
    )
    mount_preview_active: BoolProperty(default=False, options={"HIDDEN"})
    mount_saved_matrix: FloatVectorProperty(size=(4, 4), subtype="MATRIX", options={"HIDDEN"})


CLASSES = (
    RAB_Issue,
    RAB_ObjectTag,
    RAB_Pose,
    RAB_CollisionProxy,
    RAB_Component,
    RAB_Link,
    RAB_Joint,
    RAB_Frame,
    RAB_PoseValue,
    RAB_NamedPose,
    RAB_Module,
    RAB_Mount,
    RAB_Project,
)


def register():
    for cls in CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.Scene.rab = PointerProperty(type=RAB_Project)
    bpy.types.Object.rab = PointerProperty(type=RAB_ObjectTag)


def unregister():
    del bpy.types.Object.rab
    del bpy.types.Scene.rab
    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)

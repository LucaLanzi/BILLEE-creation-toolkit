"""Headless Blender tests.

Run: blender --background --factory-startup --python tests/blender_headless.py
"""

import math
import os
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import bpy
from mathutils import Euler, Matrix, Vector

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import rover_arm_builder  # noqa: E402
from rover_arm_builder.blender import authoring as A  # noqa: E402
from rover_arm_builder.blender import extract as E  # noqa: E402
from rover_arm_builder.blender import sync as S  # noqa: E402
from rover_arm_builder.core.validate import Target  # noqa: E402
from rover_arm_builder.exporters.urdf import robot_to_urdf  # noqa: E402


def clear_scene(scale=1.0):
    scene = bpy.context.scene
    scene.rab.mounts.clear()
    scene.rab.modules.clear()
    for obj in list(bpy.data.objects):
        bpy.data.objects.remove(obj)
    for collection in list(bpy.data.collections):
        bpy.data.collections.remove(collection)
    for mesh in list(bpy.data.meshes):
        bpy.data.meshes.remove(mesh)
    scene.unit_settings.scale_length = scale
    scene.rab.robot_name = "test_robot"
    return scene


def update():
    bpy.context.view_layer.update()


def urdf_of(scene, module_ids=None, name=None):
    update()
    _, report = E.validate_scene(scene, module_ids, name)
    errors = [i.message for i in report.issues if Target.VISUALIZATION in i.blocks]
    if errors:
        raise AssertionError(errors)
    return robot_to_urdf(report.robot, "pkg")


def geometry(urdf, link):
    node = next(l for l in ET.fromstring(urdf).findall("link") if l.get("name") == link)
    return node.find("visual")


def floats(text):
    return [float(v) for v in text.split()]


def link_world(urdf, target):
    from rover_arm_builder.core.transforms import Transform

    root = ET.fromstring(urdf)
    by_child = {}
    for joint in root.findall("joint"):
        o = joint.find("origin")
        by_child[joint.find("child").get("link")] = (
            joint.find("parent").get("link"),
            Transform.from_xyz_rpy(tuple(floats(o.get("xyz"))), tuple(floats(o.get("rpy")))),
        )
    pose = Transform()
    current = target
    while current in by_child:
        parent, step = by_child[current]
        pose = step @ pose
        current = parent
    return pose


def two_link(scene, prefix="m_", name="Arm", kind="arm"):
    module = A.create_module(scene, name, kind, prefix)
    base = module.links[0]
    A.add_component(module, base, "body", "BOX", size=(0.2, 0.2, 0.1))
    base = module.links[0]
    base.components[0].mass_mode = "estimated"
    base.components[0].mass = 1.0
    origin = Matrix.Translation((0.1, 0.0, 0.2))
    link, joint = A.add_link(module, "upper", module.links[0], "j1", "revolute", origin, (0, 1, 0))
    joint.has_position_limits = True
    joint.lower, joint.upper = -1.0, 1.0
    joint.has_velocity, joint.velocity = True, 1.0
    joint.has_effort, joint.effort = True, 2.0
    link = module.links[1]
    A.add_component(module, link, "arm", "BOX", size=(0.05, 0.05, 0.3), local=Matrix.Translation((0, 0, 0.15)))
    component = module.links[1].components[0]
    component.mass_mode = "estimated"
    component.mass = 0.5
    update()
    return module


class Base(unittest.TestCase):
    def setUp(self):
        self.scene = clear_scene()


class LifecycleTests(unittest.TestCase):
    def test_disable_and_reenable(self):
        count = len(bpy.app.handlers.depsgraph_update_post)
        rover_arm_builder.unregister()
        self.assertFalse(hasattr(bpy.types.Scene, "rab"))
        self.assertEqual(len(bpy.app.handlers.depsgraph_update_post), count - 1)
        rover_arm_builder.register()
        self.assertTrue(hasattr(bpy.types.Scene, "rab"))
        self.assertEqual(len(bpy.app.handlers.depsgraph_update_post), count)


class UnitAndShapeTests(Base):
    def _box_size(self, scale, size):
        scene = clear_scene(scale)
        module = A.create_module(scene, "M", "rover", "m_")
        A.add_component(module, module.links[0], "body", "BOX", size=size)
        return geometry(urdf_of(scene), "m_base").find("geometry/box").get("size")

    def test_100_mm_box_is_0_1_m_at_any_unit_scale(self):
        self.assertEqual(self._box_size(0.001, (100, 100, 100)), "0.1 0.1 0.1")
        self.assertEqual(self._box_size(1.0, (0.1, 0.1, 0.1)), "0.1 0.1 0.1")
        self.assertEqual(self._box_size(0.01, (10, 10, 10)), "0.1 0.1 0.1")

    def test_rotated_object_keeps_local_dimensions(self):
        module = A.create_module(self.scene, "M", "rover", "m_")
        local = Euler((0.4, 0.7, 1.1)).to_matrix().to_4x4()
        A.add_component(module, module.links[0], "body", "BOX", size=(0.3, 0.2, 0.1), local=local)
        visual = geometry(urdf_of(self.scene), "m_base")
        self.assertEqual(visual.find("geometry/box").get("size"), "0.3 0.2 0.1")

    def test_nonzero_parent_inverse_is_handled(self):
        module = A.create_module(self.scene, "M", "rover", "m_")
        component = A.add_component(module, module.links[0], "body", "BOX", size=(0.1, 0.1, 0.1))
        obj = component.object
        obj.matrix_parent_inverse = Matrix.Translation((5, 0, 0))
        obj.matrix_basis = Matrix.Translation((-4.9, 0, 0))
        visual = geometry(urdf_of(self.scene), "m_base")
        self.assertAlmostEqual(floats(visual.find("origin").get("xyz"))[0], 0.1, places=5)

    def test_stretched_sphere_blocks_or_uses_explicit_mesh(self):
        module = A.create_module(self.scene, "M", "rover", "m_")
        component = A.add_component(module, module.links[0], "ball", "SPHERE", radius=0.1)
        component.object.scale = (2, 1, 1)
        update()
        _, report = E.validate_scene(self.scene)
        self.assertIn("non_analytic_shape", report.codes())
        self.assertFalse(report.ok(Target.VISUALIZATION))
        module.links[0].components[0].allow_mesh_fallback = True
        visual = geometry(urdf_of(self.scene), "m_base")
        self.assertIsNotNone(visual.find("geometry/mesh"))

    def test_elliptic_cylinder_rejected_and_axial_scale_allowed(self):
        module = A.create_module(self.scene, "M", "rover", "m_")
        component = A.add_component(module, module.links[0], "can", "CYLINDER", radius=0.1, length=0.2)
        component.object.scale = (1, 1, 2)
        cylinder = geometry(urdf_of(self.scene), "m_base").find("geometry/cylinder")
        self.assertAlmostEqual(float(cylinder.get("length")), 0.4, places=6)
        module.links[0].components[0].object.scale = (2, 1, 1)
        update()
        self.assertIn("non_analytic_shape", E.validate_scene(self.scene)[1].codes())

    def test_reflection_is_explicit(self):
        module = A.create_module(self.scene, "M", "rover", "m_")
        component = A.add_component(module, module.links[0], "body", "BOX", size=(0.1, 0.2, 0.3))
        component.object.scale = (-1, 1, 1)
        update()
        self.assertIn("non_analytic_shape", E.validate_scene(self.scene)[1].codes())

    def test_modifier_is_not_exported_as_clean_primitive(self):
        module = A.create_module(self.scene, "M", "rover", "m_")
        component = A.add_component(module, module.links[0], "body", "BOX", size=(0.1, 0.2, 0.3))
        component.object.modifiers.new("bevel", "BEVEL")
        update()
        self.assertIn("non_analytic_shape", E.validate_scene(self.scene)[1].codes())

    def test_scaled_link_frame_is_blocked(self):
        module = A.create_module(self.scene, "M", "rover", "m_")
        module.links[0].frame_object.scale = (2, 2, 2)
        update()
        self.assertIn("link_frame_scaled", E.validate_scene(self.scene)[1].codes())


class MotionTests(Base):
    def test_joint_rotates_about_pivot_away_from_object_origin(self):
        module = two_link(self.scene)
        tip = module.links[1].components[0].object
        module.joints[0].preview = math.pi / 2
        update()
        world = tip.matrix_world.to_translation()
        self.assertAlmostEqual(world.x, 0.1 + 0.15, places=5)
        self.assertAlmostEqual(world.z, 0.2, places=5)

    def test_non_axis_aligned_joint_axis(self):
        module = two_link(self.scene)
        axis = Vector((1, 1, 0)).normalized()
        module.joints[0].axis = axis
        module.joints[0].preview = math.pi
        update()
        expected = Matrix.Rotation(math.pi, 3, axis) @ Vector((0, 0, 0.15)) + Vector((0.1, 0, 0.2))
        got = module.links[1].components[0].object.matrix_world.to_translation()
        self.assertLess((got - expected).length, 1e-5)

    def test_preview_does_not_change_export(self):
        module = two_link(self.scene)
        before = urdf_of(self.scene)
        module.joints[0].preview = 0.8
        update()
        self.assertEqual(urdf_of(self.scene), before)
        A.reset_preview(module)
        update()
        self.assertEqual(urdf_of(self.scene), before)

    def test_viewport_move_updates_numeric_origin(self):
        module = two_link(self.scene)
        module.links[1].frame_object.location = (0.3, 0.0, 0.25)
        update()
        A.pull_from_objects(self.scene)
        self.assertAlmostEqual(module.joints[0].origin.location[0], 0.3, places=5)

    def test_viewport_sync_ignores_preview(self):
        module = two_link(self.scene)
        module.joints[0].preview = 0.5
        update()
        self.assertFalse(A.pull_from_objects(self.scene))
        self.assertAlmostEqual(module.joints[0].origin.rotation[1], 0.0, places=6)

    def test_named_pose_round_trip(self):
        module = two_link(self.scene)
        module.joints[0].preview = 0.6
        A.store_named_pose(module, "stow")
        A.reset_preview(module)
        self.assertEqual(module.joints[0].preview, 0.0)
        A.apply_named_pose(module, "stow")
        self.assertAlmostEqual(module.joints[0].preview, 0.6, places=6)

    def test_reframe_preserves_world_geometry_joints_and_com(self):
        module = two_link(self.scene)
        frame = A.add_frame(module, module.links[1], "tip", "tool", Matrix.Translation((0, 0, 0.3)))
        third, joint = A.add_link(module, "third", module.links[1], "j2", "fixed", Matrix.Translation((0, 0, 0.3)))
        update()
        link = module.links[1]
        body = link.components[0].object
        before = {
            "body": body.matrix_world.copy(),
            "third": module.links[2].frame_object.matrix_world.copy(),
            "frame": module.frames[0].object.matrix_world.copy(),
        }
        _, report = E.validate_scene(self.scene)
        com_before = link_world(robot_to_urdf(report.robot, "pkg"), "m_upper") @ type(link_world(robot_to_urdf(report.robot, "pkg"), "m_upper"))(t=report.robot.link("m_upper").mass.com)
        new_world = link.frame_object.matrix_world @ Matrix.Translation((0.02, -0.01, 0.05)) @ Euler((0.3, 0.2, 0.5)).to_matrix().to_4x4()
        A.reframe_link(self.scene, module, link, new_world)
        update()
        self.assertLess((link.frame_object.matrix_world.to_translation() - new_world.to_translation()).length, 1e-5)
        for key, obj in (("body", body), ("third", module.links[2].frame_object), ("frame", module.frames[0].object)):
            for r in range(3):
                for c in range(4):
                    self.assertAlmostEqual(obj.matrix_world[r][c], before[key][r][c], places=4, msg=key)
        _, report = E.validate_scene(self.scene)
        urdf = robot_to_urdf(report.robot, "pkg")
        pose = link_world(urdf, "m_upper")
        com_after = pose @ type(pose)(t=report.robot.link("m_upper").mass.com)
        for i in range(3):
            self.assertAlmostEqual(com_after.t[i], com_before.t[i], places=4)


class AssemblyTests(Base):
    def _assembled(self):
        rover = two_link(self.scene, "r_", "Rover", "rover")
        arm = two_link(self.scene, "a_", "Arm", "arm")
        rover = self.scene.rab.modules[0]
        arm = self.scene.rab.modules[1]
        parent = A.add_frame(rover, rover.links[0], "mount", "mount",
                             Matrix.Translation((0.05, 0.02, 0.05)) @ Euler((0.1, 0.2, 0.7)).to_matrix().to_4x4())
        child = A.add_frame(arm, arm.links[1], "plate", "mount",
                            Matrix.Translation((0.01, -0.02, 0.03)) @ Euler((-0.3, 0.2, 0.4)).to_matrix().to_4x4())
        A.add_mount(self.scene, rover, parent, arm, child)
        update()
        return self.scene.rab.modules[0], self.scene.rab.modules[1]

    def test_frames_align_in_scene_and_export(self):
        rover, arm = self._assembled()
        a = rover.frames[0].object.matrix_world
        b = arm.frames[0].object.matrix_world
        for r in range(3):
            for c in range(4):
                self.assertAlmostEqual(a[r][c], b[r][c], places=4)
        urdf = urdf_of(self.scene)
        pa, pb = link_world(urdf, "r_mount"), link_world(urdf, "a_plate")
        for i in range(3):
            self.assertAlmostEqual(pa.t[i], pb.t[i], places=5)

    def test_moving_whole_assembly_does_not_change_export(self):
        rover, _ = self._assembled()
        before = urdf_of(self.scene)
        rover.links[0].frame_object.matrix_world = Matrix.Translation((3, -2, 1)) @ Euler((0, 0, 1.2)).to_matrix().to_4x4()
        update()
        self.assertEqual(urdf_of(self.scene), before)

    def test_detach_keeps_world_pose_and_rejects_cycle(self):
        rover, arm = self._assembled()
        root = arm.links[0].frame_object
        world = root.matrix_world.copy()
        A.detach_mount(self.scene, self.scene.rab.mounts[0].uid)
        update()
        self.assertIsNone(root.parent)
        for r in range(3):
            for c in range(4):
                self.assertAlmostEqual(root.matrix_world[r][c], world[r][c], places=5)
        A.add_mount(self.scene, rover, rover.frames[0], arm, arm.frames[0])
        with self.assertRaises(ValueError):
            A.add_mount(self.scene, arm, arm.frames[0], rover, rover.frames[0])


class PersistenceTests(Base):
    def test_proxy_adds_no_mass(self):
        module = two_link(self.scene)
        before = E.validate_scene(self.scene)[1].robot.link("m_upper").mass.mass
        A.add_collision_proxy(module, module.links[1], module.links[1].components[0], "guard", "BOX", size=(0.2, 0.2, 0.4))
        update()
        robot = E.validate_scene(self.scene)[1].robot
        self.assertEqual(robot.link("m_upper").mass.mass, before)
        self.assertEqual(len(robot.link("m_upper").collisions), 1)

    def test_rename_object_keeps_reference(self):
        module = two_link(self.scene)
        before = urdf_of(self.scene)
        module.links[1].components[0].object.name = "renamed_in_outliner"
        self.assertEqual(urdf_of(self.scene), before)

    def test_deleted_object_is_reported(self):
        module = two_link(self.scene)
        bpy.data.objects.remove(module.links[1].components[0].object)
        update()
        report = E.validate_scene(self.scene)[1]
        self.assertIn("missing_object", report.codes())

    def test_duplicate_object_is_not_treated_as_owner(self):
        module = two_link(self.scene)
        source = module.links[1].components[0].object
        clone = source.copy()
        module.collection.objects.link(clone)
        self.assertEqual(clone.rab.uid, source.rab.uid)
        self.assertIsNone(A.owner_of_object(self.scene, clone))
        self.assertIsNotNone(A.owner_of_object(self.scene, source))

    def test_selection_sync_points_to_owner(self):
        module = two_link(self.scene)
        module.active_link = 0
        self.assertTrue(S.sync_selection_from_active(self.scene, module.links[1].components[0].object))
        self.assertEqual(module.active_link, 1)
        self.assertEqual(module.active_joint, 0)

    def test_dimension_change_flags_manual_values_for_review(self):
        module = two_link(self.scene)
        component = module.links[1].components[0]
        component.mass_mode = "manual"
        component.mass = 0.5
        component.ixx, component.iyy, component.izz = 0.01, 0.01, 0.01
        component.size = (0.1, 0.1, 0.1)
        self.assertTrue(component.needs_review)
        self.assertAlmostEqual(component.ixx, 0.01, places=6)

    def test_export_leaves_scene_state_unchanged(self):
        module = two_link(self.scene)
        obj = module.links[1].components[0].object
        obj.modifiers.new("bevel", "BEVEL")
        module.links[1].components[0].allow_mesh_fallback = True
        bpy.context.view_layer.objects.active = obj
        obj.select_set(True)
        update()
        state = (obj.matrix_world.copy(), len(obj.modifiers), len(obj.data.vertices), bpy.context.mode,
                 bpy.context.view_layer.objects.active, len(bpy.data.meshes))
        first = urdf_of(self.scene)
        second = urdf_of(self.scene)
        self.assertEqual(first, second)
        self.assertEqual(state, (obj.matrix_world.copy(), len(obj.modifiers), len(obj.data.vertices), bpy.context.mode,
                                 bpy.context.view_layer.objects.active, len(bpy.data.meshes)))
        self.assertTrue(obj.select_get())

    def test_save_and_reload_preserves_export(self):
        two_link(self.scene)
        before = urdf_of(self.scene)
        with tempfile.TemporaryDirectory() as folder:
            path = os.path.join(folder, "robot with spaces.blend")
            bpy.ops.wm.save_as_mainfile(filepath=path)
            bpy.ops.wm.open_mainfile(filepath=path)
            after = urdf_of(bpy.context.scene)
            bpy.ops.wm.read_factory_settings(use_empty=False)
        self.assertEqual(after, before)


class DemoAndOperatorTests(Base):
    def _demo(self):
        from rover_arm_builder.blender.demo_scene import build_demo

        core = build_demo(self.scene)
        update()
        return core

    def test_scene_demo_matches_core_demo(self):
        from rover_arm_builder.core.validate import validate_project

        core = self._demo()
        expected = validate_project(core).robot
        _, report = E.validate_scene(self.scene)
        self.assertEqual([i.message for t in Target for i in report.blockers(t)], [])
        self.assertEqual([i.message for i in report.issues], [], "The demo should have no warnings")
        got = report.robot
        self.assertEqual({l.name for l in got.links}, {l.name for l in expected.links})
        self.assertEqual({j.name for j in got.joints}, {j.name for j in expected.joints})
        for joint in expected.joints:
            other = got.joint(joint.name)
            self.assertEqual((other.parent, other.child, other.type), (joint.parent, joint.child, joint.type))
            for i in range(3):
                self.assertAlmostEqual(other.origin.t[i], joint.origin.t[i], places=6, msg=joint.name)
        for link in expected.links:
            if link.mass is None:
                self.assertIsNone(got.link(link.name).mass)
                continue
            mine, theirs = got.link(link.name).mass, link.mass
            self.assertAlmostEqual(mine.mass, theirs.mass, places=5, msg=link.name)
            for a, b in zip(mine.inertia.matrix(), theirs.inertia.matrix()):
                for x, y in zip(a, b):
                    self.assertAlmostEqual(x, y, places=6, msg=link.name)

    def test_duplicate_module_is_independent(self):
        self._demo()
        before = urdf_of(self.scene)
        index = next(i for i, m in enumerate(self.scene.rab.modules) if m.kind == "arm")
        copy = A.duplicate_module(self.scene, index)
        copy_uid, prefix = copy.uid, copy.prefix
        self.assertEqual(prefix, "arm2_")
        arm = self.scene.rab.modules[index]
        copy = A.find(self.scene.rab.modules, copy_uid)
        self.assertFalse({l.uid for l in arm.links} & {l.uid for l in copy.links})
        self.assertFalse({l.frame_object.name for l in arm.links} & {l.frame_object.name for l in copy.links})
        copy.joints[0].lower = -0.123
        self.assertNotAlmostEqual(arm.joints[0].lower, -0.123, places=3)
        self.scene.rab.modules.remove(len(self.scene.rab.modules) - 1)
        for obj in list(self.scene.collection.children[-1].objects):
            bpy.data.objects.remove(obj)
        bpy.data.collections.remove(self.scene.collection.children[-1])
        self.assertEqual(urdf_of(self.scene), before)

    def test_mount_preview_cancel_restores_and_confirm_mounts(self):
        rover = two_link(self.scene, "r_", "Rover", "rover")
        arm = two_link(self.scene, "a_", "Arm", "arm")
        rover, arm = self.scene.rab.modules[0], self.scene.rab.modules[1]
        A.add_frame(rover, rover.links[0], "top", "mount", Matrix.Translation((0, 0, 0.05)))
        A.add_frame(arm, arm.links[0], "plate", "mount")
        rover, arm = self.scene.rab.modules[0], self.scene.rab.modules[1]
        project = self.scene.rab
        project.mount_parent_module = rover.uid
        project.mount_parent_frame = rover.frames[0].uid
        project.mount_child_module = arm.uid
        project.mount_child_frame = arm.frames[0].uid
        root = arm.links[0].frame_object
        root.matrix_world = Matrix.Translation((2, 2, 2))
        update()
        A.begin_mount_preview(self.scene)
        update()
        self.assertAlmostEqual(root.matrix_world.translation.z, 0.05, places=5)
        self.assertEqual(len(project.mounts), 0)
        A.cancel_mount_preview(self.scene)
        update()
        self.assertAlmostEqual(root.matrix_world.translation.x, 2.0, places=5)
        project.mount_offset_location = (0, 0, 0.01)
        A.begin_mount_preview(self.scene)
        A.confirm_mount_preview(self.scene)
        update()
        self.assertEqual(len(project.mounts), 1)
        self.assertAlmostEqual(root.matrix_world.translation.z, 0.06, places=5)

    def test_apply_new_zero_keeps_physical_range(self):
        module = two_link(self.scene)
        joint = module.joints[0]
        A.store_named_pose(module, "home")
        joint.preview = 0.4
        update()
        world = module.links[1].components[0].object.matrix_world.copy()
        self.assertTrue(A.apply_new_zero(module, joint))
        update()
        self.assertAlmostEqual(joint.preview, 0.0)
        self.assertAlmostEqual(joint.lower, -1.4, places=5)
        self.assertAlmostEqual(joint.upper, 0.6, places=5)
        self.assertAlmostEqual(module.poses["home"].values[0].value, -0.4, places=5)
        now = module.links[1].components[0].object.matrix_world
        for r in range(3):
            for c in range(4):
                self.assertAlmostEqual(now[r][c], world[r][c], places=4)

    def test_validate_and_export_operators(self):
        self.scene.rab.robot_name = "op_robot"
        self.assertEqual(bpy.ops.rab.create_demo(), {"FINISHED"})
        self.assertEqual(bpy.ops.rab.validate(), {"FINISHED"})
        with tempfile.TemporaryDirectory() as folder:
            self.scene.rab.export_dir = folder
            self.scene.rab.package_name = "op_description"
            self.scene.rab.export_target = "physics"
            self.assertEqual(bpy.ops.rab.export_package(), {"FINISHED"})
            self.assertTrue(os.path.isfile(os.path.join(folder, "op_description", "package.xml")))
            stages = __import__("json").loads(self.scene.rab.readiness_json)
            self.assertIn("EXPORTED", stages["physics"]["stages"])
            self.assertNotIn("EXTERNALLY_VALIDATED", stages["physics"]["stages"])

    def test_export_folder_expands_home_directory(self):
        self.assertEqual(bpy.ops.rab.create_demo(), {"FINISHED"})
        previous = os.environ.get("HOME")
        with tempfile.TemporaryDirectory() as folder:
            os.environ["HOME"] = folder
            try:
                self.scene.rab.export_dir = "~/ros ws/src"
                self.scene.rab.package_name = "home_description"
                self.assertEqual(bpy.ops.rab.export_package(), {"FINISHED"})
                self.assertTrue(os.path.isfile(os.path.join(folder, "ros ws", "src", "home_description", "package.xml")))
            finally:
                os.environ["HOME"] = previous

    def test_readiness_goes_stale_after_edit(self):
        import json

        from rover_arm_builder.core.readiness import Stage, evaluate, record_stage

        module = two_link(self.scene)
        _, report = E.validate_scene(self.scene)
        records = record_stage({}, report.robot, Target.VISUALIZATION, Stage.EXTERNALLY_VALIDATED)
        records = json.loads(json.dumps(records))
        first = next(s for s in evaluate(report, records) if s.target is Target.VISUALIZATION)
        self.assertIs(first.stage, Stage.EXTERNALLY_VALIDATED)
        module.joints[0].origin.location = (0.2, 0.0, 0.2)
        update()
        _, changed = E.validate_scene(self.scene)
        second = next(s for s in evaluate(changed, records) if s.target is Target.VISUALIZATION)
        self.assertIs(second.stage, Stage.AUTHORED)

    def test_displayed_readiness_clears_after_model_edit_but_not_preview(self):
        from rover_arm_builder.blender.panels import _LastReport, refresh_readiness

        module = two_link(self.scene)
        update()
        refresh_readiness(self.scene)
        self.assertIsNotNone(_LastReport.get(self.scene))
        module.joints[0].preview = 0.3
        self.assertIsNotNone(_LastReport.get(self.scene))
        A.reset_preview(module)
        module.joints[0].velocity = 2.5
        self.assertIsNone(_LastReport.get(self.scene))

    def test_focus_issue_selects_joint(self):
        from rover_arm_builder.blender.operators import focus_entity, run_validation

        module = two_link(self.scene)
        module.joints[0].has_velocity = False
        report = run_validation(self.scene)
        issue = next(i for i in report.issues if i.code == "missing_limit")
        module.active_joint = 0
        self.assertTrue(focus_entity(bpy.context, issue.entity_id))
        self.assertEqual(self.scene.rab.ui_tab, "arm")
        self.assertTrue(module.links[1].components[0].object.select_get())


def main():
    rover_arm_builder.register()
    suite = unittest.defaultTestLoader.loadTestsFromModule(sys.modules[__name__])
    result = unittest.TextTestRunner(stream=sys.stdout, verbosity=1).run(suite)
    rover_arm_builder.unregister()
    sys.stdout.flush()
    os._exit(0 if result.wasSuccessful() else 1)


main()

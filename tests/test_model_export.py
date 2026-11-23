import copy
import math
import struct
import unittest
import xml.etree.ElementTree as ET

from rover_arm_builder.core import demo
from rover_arm_builder.core import model as X
from rover_arm_builder.core import transforms as T
from rover_arm_builder.core.mass import box_inertia, combine, MassContributor
from rover_arm_builder.core.validate import Severity, Target, validate_project
from rover_arm_builder.exporters.stl import mesh_to_stl_bytes
from rover_arm_builder.exporters.urdf import ExportBlocked, export_urdf


def floats(text):
    return tuple(float(v) for v in text.split())


def parsed_link_poses(urdf):
    """Independent forward kinematics read back from the URDF text."""
    root = ET.fromstring(urdf)
    by_child = {}
    for joint in root.findall("joint"):
        origin = joint.find("origin")
        pose = T.Transform.from_xyz_rpy(floats(origin.get("xyz")), floats(origin.get("rpy")))
        by_child[joint.find("child").get("link")] = (joint.find("parent").get("link"), pose)

    def pose_of(link):
        if link not in by_child:
            return T.Transform()
        parent, pose = by_child[link]
        return pose_of(parent) @ pose

    return {link.get("name"): pose_of(link.get("name")) for link in root.findall("link")}


class DemoExportTests(unittest.TestCase):
    def setUp(self):
        self.project = demo.build_demo_project()

    def test_demo_is_ready_for_every_target(self):
        report = validate_project(self.project)
        self.assertEqual([i.message for i in report.issues], [])
        for target in Target:
            self.assertTrue(report.ok(target), target)

    def test_export_is_deterministic_and_leaves_model_unchanged(self):
        before = copy.deepcopy(self.project)
        first, _ = export_urdf(self.project, "demo_description", Target.PHYSICS)
        second, _ = export_urdf(self.project, "demo_description", Target.PHYSICS)
        self.assertEqual(first, second)
        self.assertEqual(before, self.project)

    def test_mount_frames_coincide_in_exported_urdf(self):
        rover = self.project.module("mod_rover")
        rover.frames[0].pose = T.Transform.from_xyz_rpy((0.12, -0.03, 0.08), (0.2, -0.1, 0.7))
        arm = self.project.module("mod_arm")
        arm.frames[0].pose = T.Transform.from_xyz_rpy((0.01, 0.02, -0.03), (-0.4, 0.3, 0.5))
        urdf, _ = export_urdf(self.project, "demo_description")
        poses = parsed_link_poses(urdf)
        a, b = poses["rover_arm_mount"], poses["arm_base_mount"]
        for i in range(3):
            self.assertAlmostEqual(a.t[i], b.t[i], places=9)
            for j in range(3):
                self.assertAlmostEqual(a.R[i][j], b.R[i][j], places=9)

    def test_mount_offset_is_applied_in_rover_mount_frame(self):
        self.project.mounts[0].offset = T.Transform.from_xyz_rpy((0.0, 0.0, 0.01))
        urdf, _ = export_urdf(self.project, "demo_description")
        poses = parsed_link_poses(urdf)
        self.assertAlmostEqual(poses["arm_base_mount"].t[2] - poses["rover_arm_mount"].t[2], 0.01, places=9)

    def test_standalone_exports_are_separate(self):
        rover_urdf, _ = export_urdf(self.project, "pkg", module_ids=["mod_rover"], name="rover")
        arm_urdf, _ = export_urdf(self.project, "pkg", module_ids=["mod_arm"], name="arm")
        rover_links = {l.get("name") for l in ET.fromstring(rover_urdf).findall("link")}
        arm_links = {l.get("name") for l in ET.fromstring(arm_urdf).findall("link")}
        self.assertIn("rover_chassis", rover_links)
        self.assertFalse(any(n.startswith("arm_") for n in rover_links))
        self.assertIn("arm_upper_link", arm_links)
        self.assertFalse(any(n.startswith("rover_") for n in arm_links))

    def test_motor_housing_does_not_spin_with_wheel(self):
        report = validate_project(self.project)
        wheel_joint = report.robot.joint("rover_left_front_wheel_joint")
        self.assertEqual(wheel_joint.parent, "rover_front_axle")
        self.assertAlmostEqual(report.robot.link("rover_left_front_wheel").mass.mass, 0.5)
        self.assertAlmostEqual(report.robot.link("rover_front_axle").mass.mass, 0.6 + 2 * 0.25)

    def test_driven_wheel_is_not_passive(self):
        robot = validate_project(self.project).robot
        wheel = robot.joint("rover_left_rear_wheel_joint")
        self.assertIs(wheel.actuation, X.Actuation.ACTUATED)
        self.assertIs(wheel.role, X.ControlRole.BASE_DRIVE)

    def test_exported_inertia_matches_independent_composite(self):
        urdf, _ = export_urdf(self.project, "pkg", Target.PHYSICS)
        root = ET.fromstring(urdf)
        link = next(l for l in root.findall("link") if l.get("name") == "arm_forearm_link")
        node = link.find("inertial")
        self.assertAlmostEqual(float(node.find("mass").get("value")), 0.45, places=9)
        com_z = (0.4 * 0.1 + 0.05 * 0.225) / 0.45
        self.assertAlmostEqual(floats(node.find("origin").get("xyz"))[2], com_z, places=9)
        ixx = (
            0.4 * (0.04**2 + 0.2**2) / 12 + 0.4 * (0.1 - com_z) ** 2
            + 0.4 * 0.05 * 0.025**2 + 0.05 * (0.225 - com_z) ** 2
        )
        self.assertAlmostEqual(float(node.find("inertia").get("ixx")), ixx, places=9)

    def test_materials_use_module_prefixes_without_mutating_source(self):
        urdf, _ = export_urdf(self.project, "pkg")
        root = ET.fromstring(urdf)
        declared = {m.get("name") for m in root.findall("material")}
        used = {m.get("name") for m in root.findall("link/visual/material")}
        self.assertEqual(declared, used)
        self.assertIn("rover_wheel_black", declared)
        self.assertIn("arm_wheel_black", declared)
        source = self.project.module("mod_arm").links[0].components[0].visuals[0]
        self.assertEqual(source.material.name, "arm_orange")

    def test_zero_rotation_export_has_si_box(self):
        urdf, _ = export_urdf(self.project, "pkg")
        link = next(l for l in ET.fromstring(urdf).findall("link") if l.get("name") == "rover_chassis")
        sizes = [v.find("geometry/box").get("size") for v in link.findall("visual") if v.find("geometry/box") is not None]
        self.assertEqual(sizes, ["0.6 0.4 0.15"])


class ValidationTests(unittest.TestCase):
    def setUp(self):
        self.project = demo.build_demo_project()
        self.arm = self.project.module("mod_arm")

    def test_missing_limit_names_joint_and_action(self):
        self.arm.joints[1].limit.velocity = None
        report = validate_project(self.project)
        issue = next(i for i in report.issues if i.code == "missing_limit")
        self.assertIn("velocity limit", issue.message)
        self.assertEqual(issue.entity_id, "jnt_elbow")
        self.assertIn("joint limits", issue.action)
        with self.assertRaises(ExportBlocked):
            export_urdf(self.project, "pkg")

    def test_closed_loop_is_explained_and_not_removed(self):
        extra = X.Joint("jnt_loop", "loop_joint", X.JointType.REVOLUTE, "lnk_arm_base", "lnk_fore",
                        axis=(0, 1, 0), limit=X.JointLimit(-1, 1, 1, 1), actuation=X.Actuation.PASSIVE)
        self.arm.joints.append(extra)
        report = validate_project(self.project)
        issue = next(i for i in report.issues if i.code == "closed_loop")
        self.assertIn("URDF", issue.message)
        self.assertIsNone(report.robot)
        self.assertEqual(len(self.arm.joints), 3)

    def test_joint_cycle_detected(self):
        self.arm.joints[0].parent_id = "lnk_fore"
        report = validate_project(self.project)
        self.assertIn("joint_cycle", report.codes())

    def test_mount_cycle_and_duplicate_rejected(self):
        self.project.mounts.append(X.Mount("mnt_back", "mod_arm", "frm_tool", "mod_rover", "frm_arm_mount"))
        self.assertIn("mount_cycle", validate_project(self.project).codes())
        self.project.mounts[1] = X.Mount("mnt_dup", "mod_rover", "frm_arm_mount", "mod_arm", "frm_arm_base_mount")
        self.assertIn("duplicate_mount", validate_project(self.project).codes())

    def test_zero_axis_and_nonfinite_blocked(self):
        self.arm.joints[0].axis = (0.0, 0.0, 0.0)
        self.assertIn("zero_axis", validate_project(self.project).codes())
        self.arm.joints[0].axis = (0.0, 0.0, 1.0)
        self.arm.joints[0].limit.upper = float("inf")
        self.assertIn("invalid_limit", validate_project(self.project).codes())

    def test_missing_mass_warns_for_display_but_blocks_physics(self):
        fore = next(l for l in self.arm.links if l.id == "lnk_fore")
        for component in fore.components:
            component.mass = None
        report = validate_project(self.project)
        self.assertTrue(report.ok(Target.VISUALIZATION))
        self.assertFalse(report.ok(Target.PHYSICS))

    def test_invalid_manual_inertia_blocks_physics(self):
        upper = next(l for l in self.arm.links if l.id == "lnk_upper")
        upper.components[0].mass = X.MassSpec(X.MassMode.MANUAL, mass=1.0,
                                              manual_inertia=X.Inertia(1.0, 0, 0, 1.0, 0, 3.0))
        report = validate_project(self.project)
        self.assertIn("invalid_inertia", report.codes())
        self.assertFalse(report.ok(Target.PHYSICS))

    def test_collision_proxy_adds_no_mass(self):
        before = validate_project(self.project).robot.link("arm_upper_link").mass.mass
        upper = next(l for l in self.arm.links if l.id == "lnk_upper")
        upper.components[0].collisions.append(X.GeometryElement("col_proxy", "proxy", X.Box(0.2, 0.2, 0.4)))
        after = validate_project(self.project).robot.link("arm_upper_link").mass.mass
        self.assertEqual(before, after)

    def test_overlapping_mass_shapes_warn(self):
        upper = next(l for l in self.arm.links if l.id == "lnk_upper")
        twin = copy.deepcopy(upper.components[0])
        twin.id, twin.name = "cmp_twin", "twin"
        twin.visuals, twin.collisions = [], []
        upper.components.append(twin)
        report = validate_project(self.project)
        issue = next(i for i in report.issues if i.code == "mass_overlap")
        self.assertIs(issue.severity, Severity.WARNING)
        self.assertIn("approximate", issue.message)

    def test_duplicate_prefix_is_reported(self):
        copy_arm = X.instantiate_module(self.arm, "arm_")
        self.project.modules.append(copy_arm)
        self.assertIn("duplicate_name", validate_project(self.project, ["mod_arm", copy_arm.id]).codes())

    def test_unsafe_name_is_blocked(self):
        self.arm.links[1].name = 'bad"><x'
        self.assertIn("invalid_name", validate_project(self.project).codes())

    def test_instance_gets_fresh_ids_and_source_is_untouched(self):
        before = copy.deepcopy(self.arm)
        instance = X.instantiate_module(self.arm, "arm2_", "Second Arm")
        self.assertEqual(before, self.arm)
        source_ids = {l.id for l in self.arm.links} | {j.id for j in self.arm.joints} | {f.id for f in self.arm.frames}
        new_ids = {l.id for l in instance.links} | {j.id for j in instance.joints} | {f.id for f in instance.frames}
        self.assertFalse(source_ids & new_ids)
        self.assertEqual(set(instance.poses["stow"]), {j.id for j in instance.joints})
        instance.joints[0].limit.upper = 0.5
        self.assertEqual(self.arm.joints[0].limit.upper, 2.6)
        self.project.modules.append(instance)
        report = validate_project(self.project, ["mod_arm", instance.id])
        self.assertNotIn("duplicate_name", report.codes())


class StlTests(unittest.TestCase):
    def test_binary_layout_and_normal(self):
        mesh = X.Mesh("tri", (((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),))
        data = mesh_to_stl_bytes(mesh)
        self.assertEqual(len(data), 80 + 4 + 50)
        self.assertEqual(struct.unpack_from("<I", data, 80)[0], 1)
        self.assertEqual(struct.unpack_from("<3f", data, 84), (0.0, 0.0, 1.0))
        self.assertEqual(data, mesh_to_stl_bytes(mesh))


if __name__ == "__main__":
    unittest.main()

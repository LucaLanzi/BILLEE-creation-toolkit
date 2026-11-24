import copy
import math
import re
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from rover_arm_builder.core import demo
from rover_arm_builder.core import model as X
from rover_arm_builder.core.billee_template import build_billee_project
from rover_arm_builder.core.transforms import Transform
from rover_arm_builder.exporters import billee as B
from rover_arm_builder.exporters.package import PackageConflict
from rover_arm_builder.exporters.urdf import ExportBlocked
from rover_arm_builder.exporters.xacro import module_macro_xml


def core(files):
    return ET.fromstring(files["urdf/robot_core.urdf.xacro"])


def joint(root, name):
    return next(j for j in root.findall("joint") if j.get("name") == name)


def nums(text):
    return [float(v) for v in text.split()]


class DropinMapping(unittest.TestCase):
    def setUp(self):
        self.project = build_billee_project()
        self.rover = self.project.modules[0]
        self.wiring = str(B.BUNDLED)

    def build(self):
        return B.build_files(self.project, self.wiring)

    def test_template_is_ready_with_no_warnings(self):
        plan = B.analyze(self.project)
        self.assertEqual([i.message for i in plan.issues], [])
        self.assertTrue(plan.ready)

    def test_wheels_map_by_physical_position_not_authoring_order(self):
        self.rover.joints.reverse()
        for item, new in zip(self.rover.joints, "abcdef"):
            item.name = f"motor_{new}"
        for index, link in enumerate(self.rover.links[1:]):
            link.name = f"part_{index}"
        files, plan, _ = self.build()
        root = core(files)
        expected = {"joint_wheel_l3": (0.39, 0.335), "joint_wheel_l2": (0.0, 0.335), "joint_wheel_l1": (-0.5, 0.335),
                    "joint_wheel_r1": (0.39, -0.335), "joint_wheel_r2": (0.0, -0.335), "joint_wheel_r3": (-0.5, -0.335)}
        for name, (x, y) in expected.items():
            xyz = nums(joint(root, name).find("origin").get("xyz"))
            self.assertAlmostEqual(xyz[0], x, places=6, msg=name)
            self.assertAlmostEqual(xyz[1], y, places=6, msg=name)
        self.assertEqual(joint(root, "joint_wheel_r1").find("child").get("link"), "wheel_r_3")
        self.assertEqual(joint(root, "joint_wheel_l1").find("child").get("link"), "wheel_1")

    def test_required_frames_exist(self):
        files, _, _ = self.build()
        root = core(files)
        self.assertIsNotNone(next(l for l in root.findall("link") if l.get("name") == "base_footprint"))
        dummy = joint(root, "dummy_joint")
        self.assertEqual((dummy.find("parent").get("link"), dummy.find("child").get("link")), ("base_footprint", "base_link"))
        self.assertAlmostEqual(nums(dummy.find("origin").get("xyz"))[2], 0.11)
        self.assertIsNone(next(l for l in root.findall("link") if l.get("name") == "base_footprint").find("inertial"))

    def test_controller_geometry_comes_from_model(self):
        for item in self.rover.joints:
            sign = 1.0 if item.origin.t[1] > 0 else -1.0
            item.origin = Transform(t=(item.origin.t[0], sign * 0.4, item.origin.t[2]))
        files, plan, _ = self.build()
        text = files["config/controllers.yaml"].decode()
        self.assertIn("wheel_separation: 0.8", text)
        self.assertIn("wheel_radius: 0.11", text)
        self.assertIn("diff_drive_controller/DiffDriveController", text)

    def test_can_wiring_is_copied_byte_for_byte(self):
        files, _, _ = self.build()
        source = (B.BUNDLED / "urdf" / "ros2_control.urdf.xacro").read_bytes()
        self.assertEqual(files["urdf/ros2_control.urdf.xacro"], source)
        self.assertEqual(files["urdf/robot.urdf.xacro"], (B.BUNDLED / "urdf" / "robot.urdf.xacro").read_bytes())

    def test_every_wiring_file_is_present(self):
        files, _, _ = self.build()
        source = {p.relative_to(B.BUNDLED).as_posix() for p in B.BUNDLED.rglob("*") if p.is_file()}
        self.assertEqual(source - set(files), set())

    def test_reversed_wheel_axis_is_normalized_for_forward_drive(self):
        self.rover.joints[0].axis = (0.0, -1.0, 0.0)
        files, _, _ = self.build()
        for item in core(files).findall("joint"):
            if item.get("name").startswith("joint_wheel"):
                self.assertEqual(nums(item.find("axis").get("xyz")), [0.0, 1.0, 0.0])

    def test_origin_moves_to_turning_center_without_moving_the_robot(self):
        for item in self.rover.joints:
            item.origin = Transform(t=(item.origin.t[0] + 0.2, item.origin.t[1], item.origin.t[2] - 0.05))
        for frame in self.rover.frames:
            frame.pose = Transform(t=(frame.pose.t[0] + 0.0, frame.pose.t[1], frame.pose.t[2]))
        files, plan, _ = self.build()
        root = core(files)
        mid = nums(joint(root, "joint_wheel_l2").find("origin").get("xyz"))
        self.assertAlmostEqual(mid[0], 0.0, places=6)
        self.assertAlmostEqual(mid[2], 0.0, places=6)
        base = next(l for l in root.findall("link") if l.get("name") == "base_link")
        chassis = next(v for v in base.findall("visual") if v.get("name") == "chassis_visual")
        xyz = nums(chassis.find("origin").get("xyz"))
        self.assertAlmostEqual(xyz[0], -0.05 - 0.2, places=6)
        self.assertAlmostEqual(xyz[2], 0.2 + 0.05, places=6)

    def test_camera_frame_sets_camera_pose(self):
        self.rover.frames[0].pose = Transform.from_xyz_rpy((0.3, 0.02, 0.4), (0.0, 0.1, 0.0))
        files, _, _ = self.build()
        camera = files["urdf/camera.urdf.xacro"].decode()
        match = re.search(r'name="camera_joint".*?<origin xyz="([^"]+)" rpy="([^"]+)"', camera, re.S)
        self.assertEqual(nums(match.group(1)), [0.3, 0.02, 0.4])
        self.assertAlmostEqual(nums(match.group(2))[1], 0.1, places=9)
        names = {l.get("name") for l in core(files).findall("link")}
        self.assertNotIn("rover_camera_link", names)
        self.assertNotIn("camera_link", names)

    def test_missing_gazebo_links_are_removed_and_sensors_kept(self):
        files, _, _ = self.build()
        text = files["urdf/robot_gz.urdf.xacro"].decode()
        self.assertNotIn("suspension_1_right_1", text)
        self.assertIn('reference="wheel_r_3"', text)
        self.assertIn('name="zed2i"', text)
        self.assertIn("libign_ros2_control-system.so", text)
        self.assertIn("/gz/odom", text)

    def test_arm_joints_are_frozen_with_a_warning(self):
        arm = demo.build_arm()
        self.project.modules.append(arm)
        self.project.mounts.append(X.Mount("m", "mod_billee", "frm_arm_mount", "mod_arm", "frm_arm_base_mount"))
        files, plan, _ = self.build()
        self.assertEqual(set(plan.frozen), {"arm_shoulder_yaw_joint", "arm_elbow_pitch_joint"})
        self.assertEqual(joint(core(files), "arm_elbow_pitch_joint").get("type"), "fixed")
        self.assertIn("billee_frozen_joint", {i.code for i in plan.issues})

    def test_wrong_wheel_count_is_explained(self):
        with self.assertRaises(ExportBlocked) as caught:
            B.build_files(demo.build_demo_project(), self.wiring)
        self.assertIn("six driven wheels", str(caught.exception))

    def test_uneven_sides_and_radii_are_blocked(self):
        item = self.rover.joints[0]
        item.origin = Transform(t=(item.origin.t[0], -0.335, 0.0))
        self.assertIn("billee_wheel_sides", {i.code for i in B.analyze(self.project).issues})
        self.setUp()
        wheel = self.rover.links[1].components[0]
        wheel.mass.shape = X.Cylinder(0.13, 0.08)
        wheel.visuals[0].shape = X.Cylinder(0.13, 0.08)
        wheel.collisions[0].shape = X.Cylinder(0.13, 0.08)
        self.assertIn("billee_wheel_radius", {i.code for i in B.analyze(self.project).issues})

    def test_empty_prefix_is_valid_xacro(self):
        module = copy.deepcopy(self.rover)
        module.prefix = ""
        params = re.search(r'params="([^"]*)"', module_macro_xml(module, "pkg")).group(1)
        self.assertTrue(params.startswith("prefix:='' "))


class DropinWrite(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.output = Path(self.tmp.name) / "my exports"
        self.project = build_billee_project()
        self.wiring = str(B.BUNDLED)

    def test_writes_only_package_and_sidecars(self):
        result = B.write_dropin(self.project, self.output, self.wiring)
        self.assertEqual(result.path, self.output / "robot_description")
        self.assertEqual(sorted(p.name for p in self.output.iterdir()),
                         [".robot_description.manifest.json", "robot_description", "robot_description_export.md"])
        self.assertFalse((result.path / ".rab_manifest.json").exists())

    def test_reexport_replaces_unchanged_output(self):
        first = B.write_dropin(self.project, self.output, self.wiring)
        before = (first.path / "urdf" / "robot_core.urdf.xacro").read_bytes()
        B.write_dropin(self.project, self.output, self.wiring)
        self.assertEqual((first.path / "urdf" / "robot_core.urdf.xacro").read_bytes(), before)

    def test_refuses_to_write_over_a_real_software_package(self):
        existing = self.output / "robot_description"
        existing.mkdir(parents=True)
        (existing / "package.xml").write_text("team file")
        with self.assertRaises(PackageConflict):
            B.write_dropin(self.project, self.output, self.wiring)
        self.assertEqual((existing / "package.xml").read_text(), "team file")

    def test_edited_export_is_not_overwritten(self):
        result = B.write_dropin(self.project, self.output, self.wiring)
        path = result.path / "config" / "controllers.yaml"
        path.write_text("my edit")
        with self.assertRaises(PackageConflict):
            B.write_dropin(self.project, self.output, self.wiring)
        self.assertEqual(path.read_text(), "my edit")

    def test_source_wiring_is_not_modified(self):
        with tempfile.TemporaryDirectory() as folder:
            import shutil

            source = Path(folder) / "ros2_ws" / "src" / "robot_description"
            shutil.copytree(B.BUNDLED, source)
            before = {p: p.read_bytes() for p in source.rglob("*") if p.is_file()}
            B.write_dropin(self.project, self.output, folder)
            after = {p: p.read_bytes() for p in source.rglob("*") if p.is_file()}
            self.assertEqual(before, after)

    def test_live_wiring_is_preferred(self):
        with tempfile.TemporaryDirectory() as folder:
            import shutil

            source = Path(folder) / "ros2_ws" / "src" / "robot_description"
            shutil.copytree(B.BUNDLED, source)
            control = source / "urdf" / "ros2_control.urdf.xacro"
            control.write_text(control.read_text() + "<!-- changed by the team -->\n")
            files, _, label = B.build_files(self.project, folder)
            self.assertEqual(label, "local checkout")
            self.assertIn(b"changed by the team", files["urdf/ros2_control.urdf.xacro"])


if __name__ == "__main__":
    unittest.main()

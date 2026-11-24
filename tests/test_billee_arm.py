import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from rover_arm_builder.core import model as X
from rover_arm_builder.core.billee_template import build_billee_arm, build_billee_assembly
from rover_arm_builder.core.validate import validate_project
from rover_arm_builder.exporters import billee as B
from rover_arm_builder.exporters.billee_arm import PACKAGE, analyze_arm, build_moveit_files, real_hardware_gaps
from rover_arm_builder.exporters.package import PackageConflict

ARM_JOINTS = [f"joint_arm_{n}" for n in range(1, 6)]


def assembly_files():
    project = build_billee_assembly()
    plan = B.analyze(project)
    robot = B.rewrite_robot(plan.report.robot, plan)
    return build_moveit_files(robot, plan.arm), plan, robot


class ArmPlanning(unittest.TestCase):
    def test_chain_runs_base_to_tool_in_order(self):
        _, plan, _ = assembly_files()
        self.assertEqual(plan.arm.arm_joints, ARM_JOINTS)
        self.assertEqual((plan.arm.base_link, plan.arm.tip_link), ("arm_base", "arm_tool"))

    def test_controllers_follow_the_motor_systems(self):
        _, plan, _ = assembly_files()
        self.assertEqual(plan.arm.controllers, {
            "arm_can_controller": ["joint_arm_1", "joint_arm_2", "joint_arm_3"],
            "arm_usb_controller": ["joint_arm_4", "joint_arm_5"],
            "gripper_controller": ["joint_gripper"],
        })

    def test_srdf_has_arm_gripper_and_conservative_collisions(self):
        files, _, robot = assembly_files()
        srdf = ET.fromstring(files["config/robot.srdf"])
        groups = {g.get("name"): g for g in srdf.findall("group")}
        self.assertEqual(set(groups), {"arm", "gripper"})
        self.assertEqual(groups["arm"].find("chain").get("tip_link"), "arm_tool")
        self.assertEqual(srdf.find("end_effector").get("parent_group"), "arm")
        self.assertIsNone(srdf.find("virtual_joint"))
        adjacent = {tuple(sorted((j.parent, j.child))) for j in robot.joints}
        for item in srdf.findall("disable_collisions"):
            self.assertIn(tuple(sorted((item.get("link1"), item.get("link2")))), adjacent)
        pairs = {tuple(sorted((i.get("link1"), i.get("link2")))) for i in srdf.findall("disable_collisions")}
        self.assertNotIn(("arm_link_3", "base_link"), pairs)
        self.assertIn(("arm_base", "base_link"), pairs)

    def test_five_axis_arm_uses_position_only_ik(self):
        files, _, _ = assembly_files()
        self.assertIn(b"position_only_ik: true", files["config/kinematics.yaml"])

    def test_joint_limits_are_yaml_floats(self):
        files, _, _ = assembly_files()
        text = files["config/joint_limits.yaml"].decode()
        self.assertIn("max_velocity: 1.0\n", text)
        self.assertIn("max_acceleration: 2.0\n", text)

    def test_named_poses_are_exported(self):
        files, _, _ = assembly_files()
        srdf = ET.fromstring(files["config/robot.srdf"])
        states = {(s.get("group"), s.get("name")) for s in srdf.findall("group_state")}
        self.assertEqual(states, {("arm", "home"), ("arm", "stow"), ("gripper", "open"), ("gripper", "closed")})

    def test_assembly_package_depends_on_rover_model(self):
        files, _, _ = assembly_files()
        self.assertIn(b"<exec_depend>robot_description</exec_depend>", files["package.xml"])
        self.assertNotIn("urdf/arm.urdf", files)

    def test_hardware_gaps_clear_for_can_when_ids_are_set(self):
        project = build_billee_assembly()
        arm = project.modules[1]
        for joint, node in zip(arm.joints[:3], (1, 2, 3)):
            joint.drive = X.MotorDrive(X.DriveKind.CAN, "P20", node_id=node)
        plan = B.analyze(project)
        robot = B.rewrite_robot(plan.report.robot, plan)
        gaps = real_hardware_gaps(robot, plan.arm)
        self.assertFalse(any("CAN node ID" in gap for gap in gaps))
        self.assertTrue(any("RoboClaw address" in gap for gap in gaps))

    def test_missing_tool_frame_is_explained(self):
        project = build_billee_assembly()
        project.modules[1].frames = [f for f in project.modules[1].frames if f.kind is not X.FrameKind.TOOL]
        plan = B.analyze(project)
        self.assertFalse(plan.ready)
        self.assertIn("arm_tool", {i.code for i in plan.issues})


class ArmModularity(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.output = Path(self.tmp.name) / "exports"

    def test_assembly_writes_two_independent_packages(self):
        result = B.write_dropin(build_billee_assembly(), self.output, str(B.BUNDLED))
        self.assertEqual(result.moveit_path, self.output / PACKAGE)
        self.assertTrue((result.path / "package.xml").is_file())
        self.assertTrue((result.moveit_path / "launch" / "moveit.launch.py").is_file())
        self.assertIn("Real arm motors stay off", result.report_path.read_text())

    def test_rover_alone_does_not_create_an_arm_package(self):
        from rover_arm_builder.core.billee_template import build_billee_project

        result = B.write_dropin(build_billee_project(), self.output, str(B.BUNDLED))
        self.assertIsNone(result.moveit_path)
        self.assertFalse((self.output / PACKAGE).exists())

    def test_standalone_arm_has_its_own_model_and_mock_hardware(self):
        project = X.Project("arm", [build_billee_arm()])
        path, plan = B.write_arm_only(project, self.output)
        urdf = ET.parse(path / "urdf" / "arm.urdf").getroot()
        self.assertEqual(urdf.find("ros2_control/hardware/plugin").text, "mock_components/GenericSystem")
        self.assertEqual(len(urdf.findall("ros2_control/joint")), 6)
        names = {l.get("name") for l in urdf.findall("link")}
        self.assertIn("world", names)
        self.assertFalse(any(n.startswith("rover_") or n == "base_link" for n in names))
        self.assertNotIn(b"robot_description", (path / "package.xml").read_bytes())

    def test_edited_moveit_package_is_not_overwritten(self):
        result = B.write_dropin(build_billee_assembly(), self.output, str(B.BUNDLED))
        target = result.moveit_path / "config" / "kinematics.yaml"
        target.write_text("my tuning")
        with self.assertRaises(PackageConflict):
            B.write_dropin(build_billee_assembly(), self.output, str(B.BUNDLED))
        self.assertEqual(target.read_text(), "my tuning")

    def test_arm_module_validates_alone(self):
        project = X.Project("arm", [build_billee_arm()])
        report = validate_project(project)
        self.assertEqual([i.message for i in report.issues], [])
        self.assertTrue(analyze_arm(project, report.robot).ok)


if __name__ == "__main__":
    unittest.main()

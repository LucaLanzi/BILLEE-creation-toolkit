import re
import struct
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from rover_arm_builder.core import model as X
from rover_arm_builder.core.billee_template import build_billee_arm, build_billee_assembly, build_billee_project
from rover_arm_builder.core.transforms import Transform
from rover_arm_builder.exporters import billee as B
from rover_arm_builder.exporters import billee_arm as A
from rover_arm_builder.exporters.package import PackageConflict
from rover_arm_builder.exporters.stl import mesh_to_stl_bytes, tessellate
from rover_arm_builder.exporters.urdf import ExportBlocked

WIRING = B.BUNDLED_ROOT
ARM_JOINTS = [f"joint_arm_{n}" for n in range(1, 6)]
XACRO = "{http://www.ros.org/wiki/xacro}"


def axes():
    return A.parse_axes((WIRING / "arm_axes.yaml").read_text())


def macro(files):
    return ET.fromstring(files["urdf/arm_model.urdf.xacro"]).find(f"{XACRO}macro")


class AxesFile(unittest.TestCase):
    def test_reads_the_team_axes(self):
        parsed = {a.joint: a for a in axes()}
        self.assertEqual(parsed["joint_arm_1"].driver, "pxx")
        self.assertEqual(parsed["joint_arm_4"].driver, "roboclaw")
        self.assertEqual(parsed["solenoid"].output, "duty")
        self.assertEqual(parsed["joint_arm_1"].max_velocity, 0.3)
        self.assertFalse(parsed["joint_arm_1"].verified)


class ArmContract(unittest.TestCase):
    def setUp(self):
        self.project = X.Project("arm", [build_billee_arm()])

    def test_joints_map_by_order_from_base_to_tool(self):
        self.project.modules[0].joints.reverse()
        plan = A.analyze_arm(self.project, axes())
        self.assertTrue(plan.ready, [i.message for i in plan.errors])
        self.assertEqual(plan.joints["arm_axis_1"], "joint_arm_1")
        self.assertEqual(plan.joints["arm_axis_5"], "joint_arm_5")
        self.assertEqual(plan.joints["arm_end_effector_axis"], "end_effector")

    def test_model_follows_import_urdf_contract(self):
        plan = A.analyze_arm(self.project, axes())
        files = A.build_arm_files(plan, WIRING / "arm_description")
        root = ET.fromstring(files["urdf/arm_model.urdf.xacro"])
        properties = {p.get("name"): p.get("value") for p in root.findall(f"{XACRO}property")}
        self.assertEqual(properties["arm_model_root_link"], "base_link")
        self.assertEqual(properties["arm_model_tip_link"], "tool_frame")
        body = macro(files)
        self.assertEqual(root.find(f"{XACRO}macro").get("name"), "arm_model")
        links = {l.get("name") for l in body.findall("link")}
        self.assertIn("${prefix}base_link", links)
        self.assertIn("${prefix}tool_frame", links)
        movable = {j.get("name") for j in body.findall("joint") if j.get("type") == "revolute"}
        self.assertEqual(movable, set(ARM_JOINTS))

    def test_end_effector_is_fixed_for_tf_and_movable_for_moveit(self):
        plan = A.analyze_arm(self.project, axes())
        files = A.build_arm_files(plan, WIRING / "arm_description")
        joint = next(j for j in macro(files).findall("joint") if j.get("name") == "end_effector")
        self.assertEqual(joint.get("type"), "${arm_model_end_effector_type}")
        self.assertIn(b'name="arm_model_end_effector_type" default="fixed"', files["urdf/arm_model.urdf.xacro"])
        moveit = A.build_moveit_files(plan, None, None)
        self.assertIn(b'name="arm_model_end_effector_type" value="revolute"', moveit["urdf/moveit_model.urdf.xacro"])

    def test_visuals_are_real_meshes_and_collisions_stay_analytic(self):
        plan = A.analyze_arm(self.project, axes())
        files = A.build_arm_files(plan, WIRING / "arm_description")
        names = re.findall(rb"package://arm_description/meshes/([^\"]+)", files["urdf/arm_model.urdf.xacro"])
        self.assertTrue(names)
        for name in names:
            self.assertIn(f"meshes/{name.decode()}", files)
        collisions = macro(files).findall("link/collision/geometry")
        self.assertTrue(collisions)
        self.assertTrue(all(g.find("mesh") is None for g in collisions))

    def test_team_wiring_is_otherwise_unchanged(self):
        plan = A.analyze_arm(self.project, axes())
        files = A.build_arm_files(plan, WIRING / "arm_description")
        for name in ("urdf/arm.urdf.xacro", "urdf/arm_standalone.urdf.xacro", "launch/arm_description.launch.py", "setup.py"):
            self.assertEqual(files[name], (WIRING / "arm_description" / name).read_bytes(), name)

    def test_wrong_axis_count_is_explained(self):
        module = self.project.modules[0]
        module.joints[4].role = X.ControlRole.OTHER
        plan = A.analyze_arm(self.project, axes())
        self.assertIn("arm_axis_count", {i.code for i in plan.issues})
        self.assertFalse(plan.ready)

    def test_limits_beyond_arm_axes_are_blocked(self):
        self.project.modules[0].joints[0].limit.upper = 7.0
        plan = A.analyze_arm(self.project, axes())
        self.assertIn("arm_limits", {i.code for i in plan.issues})

    def test_missing_tool_frame_is_explained(self):
        module = self.project.modules[0]
        module.frames = [f for f in module.frames if f.kind is not X.FrameKind.TOOL]
        self.assertIn("arm_tool", {i.code for i in A.analyze_arm(self.project, axes()).issues})

    def test_explicit_bundled_path_is_labeled_bundled(self):
        self.assertIn("bundled snapshot", A.find_arm_wiring(WIRING)[2])
        self.assertIn("bundled snapshot", B.find_wiring(WIRING)[1])

    def test_unverified_axes_are_reported(self):
        plan = A.analyze_arm(self.project, axes())
        self.assertEqual(plan.unverified, [*ARM_JOINTS, "end_effector"])


class Tessellation(unittest.TestCase):
    def test_shapes_have_outward_normals(self):
        for shape in (X.Box(0.2, 0.1, 0.05), X.Cylinder(0.05, 0.1), X.Sphere(0.04)):
            triangles = tessellate(shape)
            self.assertTrue(triangles)
            for a, b, c in triangles:
                u = [b[i] - a[i] for i in range(3)]
                v = [c[i] - a[i] for i in range(3)]
                normal = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
                center = [(a[i] + b[i] + c[i]) / 3 for i in range(3)]
                self.assertGreater(sum(normal[i] * center[i] for i in range(3)), 0.0, type(shape).__name__)
            data = mesh_to_stl_bytes(X.Mesh("shape", triangles))
            self.assertEqual(struct.unpack_from("<I", data, 80)[0], len(triangles))


class MoveItPackage(unittest.TestCase):
    def setUp(self):
        self.project = build_billee_assembly()
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.output = Path(self.tmp.name) / "exports"

    def export(self):
        return A.write_assembly(self.project, self.output, WIRING)

    def test_assembly_writes_three_independent_packages(self):
        result = self.export()
        self.assertTrue((result.rover_path / "package.xml").is_file())
        self.assertTrue((result.arm_path / "urdf" / "arm_model.urdf.xacro").is_file())
        self.assertTrue((result.moveit_path / "launch" / "moveit.launch.py").is_file())
        core = (result.rover_path / "urdf" / "robot_core.urdf.xacro").read_text()
        self.assertNotIn("joint_arm", core)
        self.assertEqual(core.count("<ros2_control"), 0)

    def test_mount_pose_reaches_the_arm_package(self):
        self.project.mounts[0].offset = Transform.from_xyz_rpy((0.01, 0.0, 0.02))
        result = self.export()
        standalone = (result.arm_path / "urdf" / "arm_standalone.urdf.xacro").read_text()
        launch = (result.arm_path / "launch" / "arm_description.launch.py").read_text()
        self.assertIn('name="mount_xyz" default="0.06 0 0.32"', standalone)
        self.assertIn('"mount_xyz": ("0.06 0 0.32"', launch)
        model = (result.moveit_path / "urdf" / "moveit_model.urdf.xacro").read_text()
        self.assertIn('<origin xyz="0.06 0 0.32"', model)

    def test_srdf_plans_the_arm_and_is_conservative(self):
        result = self.export()
        srdf = ET.parse(result.moveit_path / "config" / "robot.srdf").getroot()
        groups = {g.get("name"): g for g in srdf.findall("group")}
        self.assertEqual(groups["arm"].find("chain").get("tip_link"), "arm_tool_link")
        self.assertEqual(groups["gripper"].find("joint").get("name"), "end_effector")
        self.assertIsNone(srdf.find("virtual_joint"))
        pairs = {tuple(sorted((i.get("link1"), i.get("link2")))) for i in srdf.findall("disable_collisions")}
        self.assertIn(("arm_base_link", "base_link"), pairs)
        self.assertNotIn(("arm_link_3", "base_link"), pairs)
        states = {(s.get("group"), s.get("name")) for s in srdf.findall("group_state")}
        self.assertEqual(states, {("arm", "home"), ("arm", "stow"), ("gripper", "open"), ("gripper", "closed")})

    def test_velocity_never_exceeds_arm_axes(self):
        self.project.modules[1].joints[0].limit.velocity = 5.0
        result = self.export()
        limits = (result.moveit_path / "config" / "joint_limits.yaml").read_text()
        self.assertRegex(limits, r"joint_arm_1:\n    has_velocity_limits: true\n    max_velocity: 0\.3\n")

    def test_moveit_uses_the_bridge_and_team_controllers(self):
        result = self.export()
        controllers = (result.moveit_path / "config" / "moveit_controllers.yaml").read_text()
        self.assertIn("billee_arm_controller", controllers)
        bridge = (result.moveit_path / "scripts" / "arm_moveit_bridge.py").read_text()
        self.assertIn("arm_can_controller/follow_joint_trajectory", bridge)
        self.assertIn("arm_roboclaw_controller/commands", bridge)
        launch = (result.moveit_path / "launch" / "moveit.launch.py").read_text()
        self.assertIn('"allow_unverified", default_value="false"', launch)

    def test_unmounted_arm_is_explained(self):
        self.project.mounts = []
        with self.assertRaises(B.WiringError):
            self.export()
        self.assertFalse(self.output.exists())

    def test_edited_package_is_not_overwritten(self):
        result = self.export()
        target = result.moveit_path / "config" / "kinematics.yaml"
        target.write_text("my tuning")
        with self.assertRaises(PackageConflict):
            self.export()
        self.assertEqual(target.read_text(), "my tuning")

    def test_blocked_arm_writes_nothing(self):
        self.project.modules[1].joints[0].limit.upper = 7.0
        with self.assertRaises(ExportBlocked):
            self.export()
        self.assertFalse(self.output.exists())


class Modularity(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.output = Path(self.tmp.name)

    def test_rover_alone_writes_only_robot_description(self):
        B.write_dropin(build_billee_project(), self.output, WIRING)
        self.assertFalse((self.output / "arm_description").exists())
        self.assertFalse((self.output / A.PACKAGE).exists())

    def test_arm_alone_needs_no_rover(self):
        result = A.write_arm(X.Project("arm", [build_billee_arm()]), self.output, WIRING)
        self.assertFalse((self.output / "robot_description").exists())
        package = (result.moveit_path / "package.xml").read_text()
        self.assertNotIn("robot_description", package)
        model = (result.moveit_path / "urdf" / "moveit_model.urdf.xacro").read_text()
        self.assertNotIn("robot_core", model)

    def test_rover_export_ignores_a_mounted_arm(self):
        plain, _, _ = B.build_files(build_billee_project(), WIRING)
        assembled, _, _ = B.build_files(build_billee_assembly(), WIRING)
        self.assertEqual(plain, assembled)


if __name__ == "__main__":
    unittest.main()

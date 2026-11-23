import json
import math
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from rover_arm_builder.core import demo
from rover_arm_builder.core import model as X
from rover_arm_builder.core.mass import MassContributor, box_inertia, combine
from rover_arm_builder.core.transforms import Transform
from rover_arm_builder.core.validate import Target
from rover_arm_builder.exporters.package import MANIFEST, PackageConflict, build_files, write_package
from rover_arm_builder.exporters.urdf import ExportBlocked
from tests.mini_xacro import expand_file


def nums(text):
    return [float(v) for v in text.split()]


def semantic(root):
    """Parsed, order-independent view of a robot for numeric comparison."""
    out = {}
    for link in root.findall("link"):
        entry = {}
        inertial = link.find("inertial")
        if inertial is not None:
            entry["mass"] = [float(inertial.find("mass").get("value"))]
            entry["com"] = nums(inertial.find("origin").get("xyz"))
            entry["inertia"] = [float(inertial.find("inertia").get(k)) for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")]
        for kind in ("visual", "collision"):
            for item in link.findall(kind):
                key = f"{kind}:{item.get('name')}"
                entry[key + ":xyz"] = nums(item.find("origin").get("xyz"))
                entry[key + ":rpy"] = nums(item.find("origin").get("rpy"))
                shape = item.find("geometry")[0]
                entry[key + ":shape"] = [v for a in sorted(shape.attrib) for v in nums(shape.get(a))]
        out[f"link:{link.get('name')}"] = entry
    for joint in root.findall("joint"):
        entry = {
            "xyz": nums(joint.find("origin").get("xyz")),
            "rpy": nums(joint.find("origin").get("rpy")),
            "type:" + joint.get("type"): [1.0],
            "parent:" + joint.find("parent").get("link"): [1.0],
            "child:" + joint.find("child").get("link"): [1.0],
        }
        if joint.find("axis") is not None:
            entry["axis"] = nums(joint.find("axis").get("xyz"))
        limit = joint.find("limit")
        if limit is not None:
            entry["limit"] = [float(limit.get(k)) for k in sorted(limit.attrib)]
        out[f"joint:{joint.get('name')}"] = entry
    return out


class XacroEquivalence(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.dest = Path(self.tmp.name) / "work space" / "demo_description"
        self.project = demo.build_demo_project()

    def assert_same(self, a, b):
        self.assertEqual(set(a), set(b))
        for key in a:
            self.assertEqual(set(a[key]), set(b[key]), key)
            for field in a[key]:
                self.assertEqual(len(a[key][field]), len(b[key][field]), f"{key} {field}")
                for x, y in zip(a[key][field], b[key][field]):
                    self.assertAlmostEqual(x, y, places=8, msg=f"{key} {field}")

    def test_expanded_assembly_matches_plain_urdf(self):
        write_package(self.project, self.dest, "demo_description")
        plain = ET.parse(self.dest / "urdf" / "demo_rover_arm.urdf").getroot()
        expanded = expand_file(self.dest / "urdf" / "assembly.urdf.xacro")
        self.assert_same(semantic(plain), semantic(expanded))

    def test_standalone_wrappers_match_their_modules(self):
        write_package(self.project, self.dest, "demo_description")
        expanded = expand_file(self.dest / "urdf" / "standalone_demo_arm.urdf.xacro")
        names = {l.get("name") for l in expanded.findall("link")}
        self.assertIn("arm_upper_link", names)
        self.assertFalse(any(n.startswith("rover_") for n in names))
        self.assertNotIn("arm_mount_joint", {j.get("name") for j in expanded.findall("joint")})

    def test_changed_parameters_keep_inertia_consistent(self):
        write_package(self.project, self.dest, "demo_description")
        overrides = {"upper_link_upper_body_mass": "1.4", "upper_link_upper_body_z": "0.5"}
        expanded = expand_file(self.dest / "urdf" / "standalone_demo_arm.urdf.xacro", overrides)
        link = next(l for l in expanded.findall("link") if l.get("name") == "arm_upper_link")
        inertia = link.find("inertial/inertia")
        expected = box_inertia(1.4, 0.05, 0.05, 0.5)
        self.assertAlmostEqual(float(link.find("inertial/mass").get("value")), 1.4)
        self.assertAlmostEqual(float(inertia.get("ixx")), expected.ixx, places=9)
        self.assertAlmostEqual(float(inertia.get("izz")), expected.izz, places=9)
        size = nums(link.find("visual/geometry/box").get("size"))
        self.assertEqual(size, [0.05, 0.05, 0.5])

    def test_density_parameter_drives_mass(self):
        arm = self.project.module("mod_arm")
        upper = next(l for l in arm.links if l.id == "lnk_upper")
        upper.components[0].mass = X.MassSpec(X.MassMode.DENSITY, upper.components[0].mass.origin,
                                              X.Box(0.05, 0.05, 0.25), density=1000.0)
        write_package(self.project, self.dest, "demo_description")
        plain = ET.parse(self.dest / "urdf" / "demo_rover_arm.urdf").getroot()
        expanded = expand_file(self.dest / "urdf" / "assembly.urdf.xacro")
        self.assert_same(semantic(plain), semantic(expanded))
        doubled = expand_file(self.dest / "urdf" / "assembly.urdf.xacro", {"upper_link_upper_body_density": "2000"})
        link = next(l for l in doubled.findall("link") if l.get("name") == "arm_upper_link")
        self.assertAlmostEqual(float(link.find("inertial/mass").get("value")), 2000 * 0.05 * 0.05 * 0.25)


class PackageTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.dest = Path(self.tmp.name) / "path with spaces" / "demo_description"
        self.project = demo.build_demo_project()

    def test_expected_files_and_no_absolute_paths(self):
        write_package(self.project, self.dest, "demo_description")
        for rel in ("package.xml", "CMakeLists.txt", "launch/display.launch.py", "rviz/display.rviz",
                    "config/moveit/robot.srdf", "config/ros2_controllers.yaml", "urdf/assembly.urdf.xacro"):
            self.assertTrue((self.dest / rel).is_file(), rel)
        for path in self.dest.rglob("*"):
            if path.is_file():
                self.assertNotIn(self.tmp.name.encode(), path.read_bytes(), path.name)
                self.assertNotIn(b"\r\n", path.read_bytes(), path.name)

    def test_repeat_export_is_byte_identical(self):
        first, _ = build_files(self.project, "demo_description")
        second, _ = build_files(self.project, "demo_description")
        self.assertEqual(first, second)
        write_package(self.project, self.dest, "demo_description")
        write_package(self.project, self.dest, "demo_description")

    def test_user_folder_survives_and_edits_stop_export(self):
        write_package(self.project, self.dest, "demo_description")
        user = self.dest / "config" / "user" / "mine.yaml"
        user.parent.mkdir(parents=True)
        user.write_text("keep: true\n")
        write_package(self.project, self.dest, "demo_description")
        self.assertEqual(user.read_text(), "keep: true\n")
        readme = self.dest / "README.md"
        readme.write_text("my edit\n")
        with self.assertRaises(PackageConflict) as caught:
            write_package(self.project, self.dest, "demo_description")
        self.assertIn("README.md", str(caught.exception))
        self.assertEqual(readme.read_text(), "my edit\n")

    def test_unrelated_folder_is_refused(self):
        self.dest.mkdir(parents=True)
        (self.dest / "notes.txt").write_text("mine")
        with self.assertRaises(PackageConflict):
            write_package(self.project, self.dest, "demo_description")
        self.assertEqual(sorted(p.name for p in self.dest.iterdir()), ["notes.txt"])

    def test_blocked_export_writes_nothing(self):
        self.project.module("mod_arm").joints[0].limit.effort = None
        with self.assertRaises(ExportBlocked):
            write_package(self.project, self.dest, "demo_description")
        self.assertFalse(self.dest.exists())

    def test_no_staging_left_behind(self):
        write_package(self.project, self.dest, "demo_description")
        self.assertEqual([p.name for p in self.dest.parent.iterdir()], ["demo_description"])
        self.assertIn("files", json.loads((self.dest / MANIFEST).read_text()))

    def test_srdf_plans_arm_only_with_full_rover_present(self):
        files, _ = build_files(self.project, "demo_description")
        srdf = ET.fromstring(files["config/moveit/robot.srdf"])
        chain = srdf.find("group/chain")
        self.assertEqual((chain.get("base_link"), chain.get("tip_link")), ("arm_base", "arm_tool_frame"))
        self.assertEqual({g.get("name") for g in srdf.findall("group_state")}, {"home", "stow"})
        passive = {p.get("name") for p in srdf.findall("passive_joint")}
        self.assertEqual(passive, {"rover_front_axle_joint"})
        urdf = ET.fromstring(files["urdf/demo_rover_arm.urdf"])
        self.assertIn("rover_chassis", {l.get("name") for l in urdf.findall("link")})

    def test_srdf_disables_only_adjacent_pairs(self):
        files, report = build_files(self.project, "demo_description")
        srdf = ET.fromstring(files["config/moveit/robot.srdf"])
        adjacent = {tuple(sorted((j.parent, j.child))) for j in report.robot.joints}
        for item in srdf.findall("disable_collisions"):
            self.assertEqual(item.get("reason"), "Adjacent")
            self.assertIn(tuple(sorted((item.get("link1"), item.get("link2")))), adjacent)
        pairs = {tuple(sorted((i.get("link1"), i.get("link2")))) for i in srdf.findall("disable_collisions")}
        self.assertNotIn(("arm_forearm_link", "rover_chassis"), pairs)

    def test_gazebo_additions_stay_out_of_portable_description(self):
        files, _ = build_files(self.project, "demo_description")
        self.assertNotIn(b"ros2_control", files["urdf/demo_rover_arm.urdf"])
        self.assertNotIn(b"ros2_control", files["urdf/assembly.urdf.xacro"])
        gazebo = ET.fromstring(files["urdf/gazebo.urdf.xacro"])
        self.assertEqual(gazebo.find("ros2_control/hardware/plugin").text, "ign_ros2_control/IgnitionSystem")
        self.assertEqual(gazebo.find("gazebo/plugin").get("filename"), "libign_ros2_control-system.so")
        self.assertIn(b"ignition.msgs.Clock", files["launch/gazebo.launch.py"])
        self.assertNotIn(b"moveit", files["package.xml"])
        joints ={j.get("name"): j for j in gazebo.find("ros2_control").findall("joint")}
        self.assertEqual(joints["rover_front_axle_joint"].findall("command_interface"), [])
        self.assertEqual(joints["rover_left_front_wheel_joint"].find("command_interface").get("name"), "velocity")
        self.assertEqual(joints["arm_elbow_pitch_joint"].find("command_interface").get("name"), "position")

    def test_bad_package_name_rejected(self):
        with self.assertRaises(ValueError):
            build_files(self.project, "Bad Name")


if __name__ == "__main__":
    unittest.main()

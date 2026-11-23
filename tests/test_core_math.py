import math
import random
import unittest

from rover_arm_builder.core import mass as M
from rover_arm_builder.core import transforms as T
from rover_arm_builder.core import units as U


def assert_mat_close(case, a, b, places=9):
    for r in range(3):
        for c in range(3):
            case.assertAlmostEqual(a[r][c], b[r][c], places=places, msg=f"entry {r},{c}")


def assert_vec_close(case, a, b, places=9):
    for i in range(3):
        case.assertAlmostEqual(a[i], b[i], places=places, msg=f"component {i}")


class UnitTests(unittest.TestCase):
    def test_100_mm_is_0_1_m(self):
        self.assertAlmostEqual(U.length_to_m(100.0, "mm"), 0.1, places=12)

    def test_equivalent_scene_scales_agree(self):
        in_mm_scene = U.blender_length_to_m(100.0, 0.001)
        in_m_scene = U.blender_length_to_m(0.1, 1.0)
        self.assertAlmostEqual(in_mm_scene, in_m_scene, places=12)

    def test_rejects_unknown_unit_and_bad_scale(self):
        with self.assertRaises(ValueError):
            U.length_to_m(1.0, "inch")
        with self.assertRaises(ValueError):
            U.blender_length_to_m(1.0, 0.0)


class TransformTests(unittest.TestCase):
    def test_rpy_order_is_fixed_axis_zyx(self):
        roll, pitch, yaw = 0.3, -0.7, 1.1
        expected = T.mat_mul(T.rot_z(yaw), T.mat_mul(T.rot_y(pitch), T.rot_x(roll)))
        assert_mat_close(self, T.rpy_to_matrix((roll, pitch, yaw)), expected)

    def test_rotations_do_not_commute(self):
        a = T.mat_mul(T.rot_x(math.pi / 2), T.rot_z(math.pi / 2))
        b = T.mat_mul(T.rot_z(math.pi / 2), T.rot_x(math.pi / 2))
        self.assertGreater(abs(a[0][0] - b[0][0]) + abs(a[0][2] - b[0][2]) + abs(a[2][1] - b[2][1]), 0.5)

    def test_rpy_round_trip_random(self):
        rng = random.Random(7)
        for _ in range(500):
            rpy = (rng.uniform(-3, 3), rng.uniform(-1.5, 1.5), rng.uniform(-3, 3))
            m = T.rpy_to_matrix(rpy)
            assert_mat_close(self, T.rpy_to_matrix(T.matrix_to_rpy(m)), m)

    def test_rpy_near_gimbal_lock_preserves_rotation(self):
        for pitch in (math.pi / 2, -math.pi / 2, math.pi / 2 - 1e-10, -math.pi / 2 + 1e-10):
            m = T.rpy_to_matrix((0.4, pitch, -0.9))
            assert_mat_close(self, T.rpy_to_matrix(T.matrix_to_rpy(m)), m, places=7)

    def test_inverse_and_relative(self):
        a = T.Transform.from_xyz_rpy((1, 2, 3), (0.1, 0.2, 0.3))
        b = T.Transform.from_xyz_rpy((-1, 0.5, 2), (1.0, -0.4, 0.7))
        rel = T.relative(a, b)
        composed = a @ rel
        assert_mat_close(self, composed.R, b.R)
        assert_vec_close(self, composed.t, b.t)

    def test_pivot_away_from_origin(self):
        pivot = T.Transform(T.IDENTITY3, (1.0, 0.0, 0.0))
        spin = T.Transform(T.rot_z(math.pi / 2))
        moved = (pivot @ spin @ pivot.inverse()).apply((2.0, 0.0, 0.0))
        assert_vec_close(self, moved, (1.0, 1.0, 0.0))

    def test_non_axis_aligned_rotation_keeps_axis_fixed(self):
        axis = T.normalized((1.0, 2.0, -0.5))
        assert_vec_close(self, T.mat_vec(T.axis_angle_to_matrix(axis, 1.3), axis), axis)

    def test_mount_aligns_frames_with_offset_rotated_frames(self):
        T_W_rover_mount = T.Transform.from_xyz_rpy((0.4, -0.1, 0.5), (0.2, 0.1, 1.2))
        T_root_arm_mount = T.Transform.from_xyz_rpy((0.03, 0.02, -0.05), (-0.5, 0.3, 0.8))
        T_W_root = T.mount_arm_root(T_W_rover_mount, T.Transform(), T_root_arm_mount)
        aligned = T_W_root @ T_root_arm_mount
        assert_mat_close(self, aligned.R, T_W_rover_mount.R)
        assert_vec_close(self, aligned.t, T_W_rover_mount.t)

    def test_classify_linear(self):
        R = T.rpy_to_matrix((0.3, 0.4, 0.5))
        self.assertEqual(T.classify_linear(R).kind, T.AffineKind.RIGID)
        uniform = T.mat_mul(R, ((2, 0, 0), (0, 2, 0), (0, 0, 2)))
        self.assertEqual(T.classify_linear(uniform).kind, T.AffineKind.UNIFORM_SCALE)
        stretched = T.mat_mul(R, ((2, 0, 0), (0, 1, 0), (0, 0, 1)))
        report = T.classify_linear(stretched)
        self.assertEqual(report.kind, T.AffineKind.NONUNIFORM_SCALE)
        assert_mat_close(self, report.rotation, R)
        mirrored = T.mat_mul(R, ((-1, 0, 0), (0, 1, 0), (0, 0, 1)))
        self.assertEqual(T.classify_linear(mirrored).kind, T.AffineKind.REFLECTION)
        sheared = T.mat_mul(((2, 0, 0), (0, 1, 0), (0, 0, 1)), R)
        self.assertEqual(T.classify_linear(sheared).kind, T.AffineKind.SHEAR)
        flat = ((1, 0, 0), (0, 1, 0), (0, 0, 0))
        self.assertEqual(T.classify_linear(flat).kind, T.AffineKind.SINGULAR)


class MassTests(unittest.TestCase):
    def test_spec_box_values(self):
        i = M.box_inertia(2.0, 0.2, 0.1, 0.05)
        self.assertAlmostEqual(i.ixx, 0.002083333333, places=10)
        self.assertAlmostEqual(i.iyy, 0.007083333333, places=10)
        self.assertAlmostEqual(i.izz, 0.008333333333, places=10)

    def test_cylinder_and_sphere(self):
        c = M.cylinder_inertia(3.0, 0.1, 0.4)
        self.assertAlmostEqual(c.ixx, 3.0 * (3 * 0.01 + 0.16) / 12, places=12)
        self.assertAlmostEqual(c.izz, 3.0 * 0.01 / 2, places=12)
        s = M.sphere_inertia(5.0, 0.2)
        self.assertAlmostEqual(s.ixx, 0.08, places=12)

    def test_two_halves_equal_whole_box(self):
        half = M.box_inertia(1.0, 0.1, 0.1, 0.1)
        result = M.combine(
            [M.MassContributor(1.0, (-0.05, 0, 0), half), M.MassContributor(1.0, (0.05, 0, 0), half)]
        )
        whole = M.box_inertia(2.0, 0.2, 0.1, 0.1)
        self.assertAlmostEqual(result.mass, 2.0)
        assert_vec_close(self, result.com, (0, 0, 0))
        assert_mat_close(self, result.inertia.matrix(), whole.matrix(), places=12)

    def test_composite_matches_point_mass_integration(self):
        rng = random.Random(11)
        points = [
            (rng.uniform(0.1, 1.0), (rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-1, 1)))
            for _ in range(12)
        ]
        tiny = M.Inertia(1e-12, 0, 0, 1e-12, 0, 1e-12)
        result = M.combine([M.MassContributor(m, p, tiny) for m, p in points])
        total = sum(m for m, _ in points)
        c = tuple(sum(m * p[i] for m, p in points) / total for i in range(3))
        ixy = -sum(m * (p[0] - c[0]) * (p[1] - c[1]) for m, p in points)
        izz = sum(m * ((p[0] - c[0]) ** 2 + (p[1] - c[1]) ** 2) for m, p in points) + 12 * 1e-12
        self.assertAlmostEqual(result.inertia.ixy, ixy, places=9)
        self.assertAlmostEqual(result.inertia.izz, izz, places=9)

    def test_rotated_box_has_expected_off_diagonal(self):
        base = M.box_inertia(2.0, 0.2, 0.1, 0.05)
        rotated = base.rotated(T.rot_z(math.pi / 4))
        self.assertAlmostEqual(rotated.ixx, (base.ixx + base.iyy) / 2, places=12)
        self.assertAlmostEqual(rotated.iyy, (base.ixx + base.iyy) / 2, places=12)
        self.assertAlmostEqual(rotated.ixy, (base.ixx - base.iyy) / 2, places=12)
        self.assertAlmostEqual(rotated.izz, base.izz, places=12)

    def test_principal_moments_are_rotation_invariant(self):
        base = M.box_inertia(2.0, 0.2, 0.1, 0.05)
        rotated = base.rotated(T.rpy_to_matrix((0.5, -0.8, 1.2)))
        expected = sorted((base.ixx, base.iyy, base.izz))
        for got, want in zip(M.principal_moments(rotated), expected):
            self.assertAlmostEqual(got, want, places=10)
        self.assertEqual(M.inertia_problems(2.0, rotated), [])

    def test_invalid_inertia_is_explained(self):
        triangle = M.Inertia(1.0, 0, 0, 1.0, 0, 3.0)
        self.assertIn("triangle", M.inertia_problems(1.0, triangle)[0])
        indefinite = M.Inertia(1.0, 2.0, 0, 1.0, 0, 1.0)
        self.assertIn("positive definite", M.inertia_problems(1.0, indefinite)[0])
        self.assertTrue(M.inertia_problems(0.0, M.box_inertia(1, 1, 1, 1)))
        self.assertTrue(M.inertia_problems(float("nan"), M.box_inertia(1, 1, 1, 1)))


if __name__ == "__main__":
    unittest.main()

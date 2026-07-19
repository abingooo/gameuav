import math
import unittest

import numpy as np

from strategy.smpf.runtime.contracts import CameraIntrinsics, ObjectSphere
from strategy.smpf.runtime.geometry import (
    body_from_color_transform,
    body_from_color_via_infra1,
    deproject_color_pixel,
    rigid_transform,
    robust_depth_estimate,
    rotation_matrix_from_quaternion,
    sphere_from_aligned_bbox,
    transform_points,
    validate_extrinsic_transform,
    validate_polyline,
)
from strategy.smpf.runtime.scene_memory import SemanticSceneMemory


class SmpfGeometryTest(unittest.TestCase):
    def test_color_center_deprojects_along_optical_axis(self):
        intrinsics = CameraIntrinsics(100.0, 100.0, 50.0, 40.0, 100, 80)
        point = deproject_color_pixel((50.0, 40.0), 2.0, intrinsics)
        np.testing.assert_allclose(point, (0.0, 0.0, 2.0), atol=1e-9)

    def test_body_from_color_composes_vins_and_realsense_extrinsics(self):
        body_from_depth = rigid_transform(np.eye(3), (0.20, 0.0, 0.0))
        color_from_depth = rigid_transform(np.eye(3), (0.01, 0.0, 0.0))
        body_from_color = body_from_color_transform(body_from_depth, color_from_depth)
        point_body = transform_points(body_from_color, (0.0, 0.0, 1.0))
        np.testing.assert_allclose(point_body, (0.19, 0.0, 1.0), atol=1e-9)

    def test_body_from_color_includes_depth_to_infra1_extrinsic(self):
        body_from_infra1 = rigid_transform(np.eye(3), (0.20, 0.0, 0.0))
        infra1_from_depth = rigid_transform(np.eye(3), (0.02, 0.0, 0.0))
        color_from_depth = rigid_transform(np.eye(3), (0.01, 0.0, 0.0))
        body_from_color = body_from_color_via_infra1(
            body_from_infra1,
            infra1_from_depth,
            color_from_depth,
        )
        point_body = transform_points(body_from_color, (0.0, 0.0, 1.0))
        np.testing.assert_allclose(point_body, (0.21, 0.0, 1.0), atol=1e-9)

    def test_quaternion_rotation_uses_full_body_attitude(self):
        half = math.pi / 4.0
        rotation = rotation_matrix_from_quaternion((0.0, 0.0, math.sin(half), math.cos(half)))
        transform = rigid_transform(rotation, (1.0, 2.0, 3.0))
        point = transform_points(transform, (1.0, 0.0, 0.0))
        np.testing.assert_allclose(point, (1.0, 3.0, 3.0), atol=1e-8)

    def test_robust_depth_rejects_invalid_and_extreme_samples(self):
        depth = np.full((20, 20), 2.0, dtype=float)
        depth[9, 9] = 0.0
        depth[10, 10] = 9.0
        depth[11, 11] = np.nan
        estimate = robust_depth_estimate(depth, [(10, 10)], window_radius=2)
        self.assertAlmostEqual(estimate.value_m, 2.0)
        self.assertGreater(estimate.sample_count, 10)
        self.assertLess(estimate.std_m, 0.05)

    def test_sphere_model_uses_dynamic_intrinsics_and_uncertainty_margin(self):
        intrinsics = CameraIntrinsics(100.0, 100.0, 50.0, 50.0, 100, 100)
        depth = np.full((100, 100), 2.0, dtype=float)
        sphere, estimate = sphere_from_aligned_bbox(
            "chair",
            (40, 40, 60, 60),
            depth,
            intrinsics,
            np.eye(4),
            safety_margin_m=0.3,
        )
        np.testing.assert_allclose(sphere.center, (0.0, 0.0, 2.0), atol=1e-9)
        self.assertGreaterEqual(sphere.radius, math.sqrt(0.2 ** 2 + 0.2 ** 2) + 0.3)
        self.assertAlmostEqual(estimate.value_m, 2.0)

    def test_segment_collision_is_rejected_even_when_endpoints_are_clear(self):
        obstacle = ObjectSphere("pole", (0.0, 0.0, 0.0), 0.5)
        result = validate_polyline([(-2.0, 0.0, 0.0), (2.0, 0.0, 0.0)], [obstacle])
        self.assertFalse(result.valid)
        self.assertTrue(any(issue.kind == "segment_collision" for issue in result.issues))

    def test_clear_detour_is_accepted(self):
        obstacle = ObjectSphere("pole", (0.0, 0.0, 0.0), 0.5)
        result = validate_polyline(
            [(-2.0, 0.0, 0.0), (-1.0, 1.0, 0.0), (1.0, 1.0, 0.0), (2.0, 0.0, 0.0)],
            [obstacle],
        )
        self.assertTrue(result.valid, result.issues)
        self.assertGreater(result.minimum_clearance_m, 0.0)

    def test_current_origin_can_be_exempt_from_future_flight_bounds(self):
        result = validate_polyline(
            [(0.0, 0.0, 0.0), (1.0, 0.0, 0.2), (2.0, 0.0, 0.2)],
            bounds={"z_min": 0.1, "z_max": 3.0},
            bounds_start_index=1,
        )
        self.assertTrue(result.valid, result.issues)

    def test_implausible_sensor_translation_is_rejected(self):
        transform = np.eye(4)
        transform[0, 3] = 2.6
        with self.assertRaises(ValueError):
            validate_extrinsic_transform(transform, 0.75, name="body_from_infra1")

    def test_calibrated_sensor_translation_is_accepted(self):
        transform = np.eye(4)
        transform[:3, 3] = (0.24, -0.02, 0.03)
        accepted = validate_extrinsic_transform(transform, 0.75)
        np.testing.assert_allclose(accepted, transform)

    def test_future_guidepoint_cannot_use_origin_bounds_exemption(self):
        result = validate_polyline(
            [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.2)],
            bounds={"z_min": 0.1, "z_max": 3.0},
            bounds_start_index=1,
        )
        self.assertFalse(result.valid)
        self.assertTrue(any(issue.kind == "bounds" and issue.index == 1 for issue in result.issues))


class SemanticSceneMemoryTest(unittest.TestCase):
    def test_static_observations_fuse_conservatively(self):
        memory = SemanticSceneMemory(association_distance_m=0.5, ttl_sec=10.0)
        first = ObjectSphere("chair", (1.0, 0.0, 0.0), 0.4, frame_id="world")
        second = ObjectSphere("chair", (1.2, 0.0, 0.0), 0.4, frame_id="world")
        memory.update(first, 1.0)
        memory.update(second, 2.0)
        snapshot = memory.snapshot()
        self.assertEqual(len(snapshot), 1)
        self.assertGreater(snapshot[0].radius, 0.4)
        self.assertGreater(snapshot[0].center[0], 1.0)
        self.assertLess(snapshot[0].center[0], 1.2)

    def test_labels_do_not_cross_associate_and_entries_expire(self):
        memory = SemanticSceneMemory(association_distance_m=1.0, ttl_sec=2.0)
        memory.update(ObjectSphere("chair", (0, 0, 0), 0.3, frame_id="world"), 1.0)
        memory.update(ObjectSphere("cone", (0, 0, 0), 0.3, frame_id="world"), 1.0)
        self.assertEqual(len(memory.snapshot()), 2)
        self.assertEqual(len(memory.prune(3.1)), 2)
        self.assertEqual(memory.snapshot(), [])

    def test_large_radii_do_not_merge_distinct_same_class_objects(self):
        memory = SemanticSceneMemory(association_distance_m=0.35, ttl_sec=10.0)
        first = memory.update(
            ObjectSphere("chair", (1.0, 0.0, 0.0), 0.8, frame_id="world"),
            1.0,
        )
        second = memory.update(
            ObjectSphere("chair", (1.6, 0.0, 0.0), 0.8, frame_id="world"),
            2.0,
        )
        self.assertNotEqual(first.object_id, second.object_id)
        self.assertEqual(len(memory.snapshot_entries()), 2)

    def test_match_and_snapshot_entries_preserve_stable_object_id(self):
        memory = SemanticSceneMemory(association_distance_m=0.35, ttl_sec=10.0)
        created = memory.update(
            ObjectSphere("chair", (1.0, 0.0, 0.0), 0.4, frame_id="world"),
            1.0,
        )
        matched = memory.match(
            ObjectSphere("chair", (1.1, 0.0, 0.0), 0.4, frame_id="world"),
            2.0,
        )
        self.assertEqual(matched.object_id, created.object_id)
        entries = memory.snapshot_entries()
        self.assertEqual(entries[0].object_id, created.object_id)
        entries[0].center = (99.0, 0.0, 0.0)
        self.assertNotEqual(memory.snapshot_entries()[0].center, entries[0].center)

    def test_short_label_variants_share_semantic_head(self):
        memory = SemanticSceneMemory(association_distance_m=0.35, ttl_sec=10.0)
        first = memory.update(
            ObjectSphere("wooden chairs near wall", (1.0, 0.0, 0.0), 0.4, frame_id="world"),
            1.0,
        )
        second = memory.update(
            ObjectSphere("chair", (1.1, 0.0, 0.0), 0.4, frame_id="world"),
            2.0,
        )
        self.assertEqual(first.object_id, second.object_id)
        different = memory.update(
            ObjectSphere("table", (1.1, 0.0, 0.0), 0.4, frame_id="world"),
            3.0,
        )
        self.assertNotEqual(first.object_id, different.object_id)


if __name__ == "__main__":
    unittest.main()

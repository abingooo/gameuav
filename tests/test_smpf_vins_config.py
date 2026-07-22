from pathlib import Path
import unittest

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "ros_nodes/state_estimation/VINS-Fusion/config/fast_drone_250.yaml"
CALIBRATION_CONFIG = ROOT / "ros_nodes/state_estimation/VINS-Fusion/config/fast_drone_250_online_calibration.yaml"
CALIBRATION_LEFT = ROOT / "ros_nodes/state_estimation/VINS-Fusion/config/left_online_calibration.yaml"
CALIBRATION_RIGHT = ROOT / "ros_nodes/state_estimation/VINS-Fusion/config/right_online_calibration.yaml"
CALIBRATION_LAUNCH = ROOT / "launch/bringup_vins_online_calibration.launch"
CANDIDATE_CONFIG = ROOT / "ros_nodes/state_estimation/VINS-Fusion/config/fast_drone_250_calibrated_candidate.yaml"
CANDIDATE_LAUNCH = ROOT / "launch/bringup_vins_calibrated_candidate.launch"


class SmpfVinsConfigTest(unittest.TestCase):
    def test_realflight_uses_fixed_calibrated_camera_extrinsics(self):
        storage = cv2.FileStorage(str(CONFIG), cv2.FILE_STORAGE_READ)
        try:
            self.assertEqual(int(storage.getNode("estimate_extrinsic").real()), 0)
            self.assertEqual(int(storage.getNode("estimate_td").real()), 0)
            self.assertAlmostEqual(storage.getNode("td").real(), 0.0017191652713265113)
            body_from_cam0 = storage.getNode("body_T_cam0").mat()
            body_from_cam1 = storage.getNode("body_T_cam1").mat()
        finally:
            storage.release()
        for transform in (body_from_cam0, body_from_cam1):
            self.assertEqual(transform.shape, (4, 4))
            self.assertLess(float(np.linalg.norm(transform[:3, 3])), 0.75)
            np.testing.assert_allclose(
                transform[:3, :3].T.dot(transform[:3, :3]),
                np.eye(3),
                atol=1e-5,
            )

        relative = np.linalg.inv(body_from_cam0).dot(body_from_cam1)
        self.assertAlmostEqual(float(np.linalg.norm(relative[:3, 3])), 0.050226251, places=6)

    def test_online_calibration_profile_is_isolated_and_observable(self):
        storage = cv2.FileStorage(str(CALIBRATION_CONFIG), cv2.FILE_STORAGE_READ)
        try:
            self.assertEqual(int(storage.getNode("estimate_extrinsic").real()), 1)
            self.assertEqual(int(storage.getNode("estimate_td").real()), 1)
            self.assertEqual(int(storage.getNode("show_track").real()), 1)
            self.assertAlmostEqual(storage.getNode("td").real(), 0.0017191652713265113)
            output_path = storage.getNode("output_path").string()
            transforms = [
                storage.getNode("body_T_cam0").mat(),
                storage.getNode("body_T_cam1").mat(),
            ]
        finally:
            storage.release()
        self.assertTrue(output_path.endswith("/runtime/vins_online_calibration"))
        for transform in transforms:
            self.assertEqual(transform.shape, (4, 4))
            np.testing.assert_allclose(
                transform[:3, :3].T.dot(transform[:3, :3]),
                np.eye(3),
                atol=1e-5,
            )

        for camera_config in (CALIBRATION_LEFT, CALIBRATION_RIGHT):
            camera_storage = cv2.FileStorage(str(camera_config), cv2.FILE_STORAGE_READ)
            try:
                distortion = camera_storage.getNode("distortion_parameters")
                projection = camera_storage.getNode("projection_parameters")
                for coefficient in ("k1", "k2", "p1", "p2"):
                    self.assertEqual(distortion.getNode(coefficient).real(), 0.0)
                self.assertAlmostEqual(projection.getNode("fx").real(), 386.6515197753906)
                self.assertAlmostEqual(projection.getNode("fy").real(), 386.6515197753906)
                self.assertAlmostEqual(projection.getNode("cx").real(), 322.40374755859375)
                self.assertAlmostEqual(projection.getNode("cy").real(), 244.408203125)
            finally:
                camera_storage.release()

        launch_text = CALIBRATION_LAUNCH.read_text(encoding="utf-8")
        self.assertIn("fast_drone_250_online_calibration.yaml", launch_text)
        self.assertNotIn("px4ctrl", launch_text)
        self.assertNotIn("bringup_ego", launch_text)

    def test_calibrated_candidate_is_fixed_and_sensor_only(self):
        storage = cv2.FileStorage(str(CANDIDATE_CONFIG), cv2.FILE_STORAGE_READ)
        try:
            self.assertEqual(int(storage.getNode("estimate_extrinsic").real()), 0)
            self.assertEqual(int(storage.getNode("estimate_td").real()), 0)
            self.assertAlmostEqual(storage.getNode("td").real(), 0.0017191652713265113)
            body_from_cam0 = storage.getNode("body_T_cam0").mat()
            body_from_cam1 = storage.getNode("body_T_cam1").mat()
        finally:
            storage.release()

        for transform in (body_from_cam0, body_from_cam1):
            np.testing.assert_allclose(
                transform[:3, :3].T.dot(transform[:3, :3]),
                np.eye(3),
                atol=1e-9,
            )
            self.assertAlmostEqual(float(np.linalg.det(transform[:3, :3])), 1.0, places=9)

        relative = np.linalg.inv(body_from_cam0).dot(body_from_cam1)
        self.assertAlmostEqual(float(np.linalg.norm(relative[:3, 3])), 0.050226251, places=6)

        launch_text = CANDIDATE_LAUNCH.read_text(encoding="utf-8")
        self.assertIn("fast_drone_250_calibrated_candidate.yaml", launch_text)
        self.assertNotIn("px4ctrl", launch_text)
        self.assertNotIn("bringup_ego", launch_text)


if __name__ == "__main__":
    unittest.main()

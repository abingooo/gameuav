from pathlib import Path
import unittest

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "ros_nodes/state_estimation/VINS-Fusion/config/fast_drone_250.yaml"


class SmpfVinsConfigTest(unittest.TestCase):
    def test_realflight_uses_fixed_calibrated_camera_extrinsics(self):
        storage = cv2.FileStorage(str(CONFIG), cv2.FILE_STORAGE_READ)
        try:
            self.assertEqual(int(storage.getNode("estimate_extrinsic").real()), 0)
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


if __name__ == "__main__":
    unittest.main()

import time
import unittest

from gateway.camera_stream_gateway.cameras import CameraManager


class CameraStreamGatewayTest(unittest.TestCase):
    def test_subscriber_stale_detection(self):
        manager = CameraManager(workspace_root=".")

        self.assertFalse(manager._subscriber_stale("rgb1"))

        with manager._frame_condition:
            manager._frames["rgb1"] = {"received_at": time.time(), "seq": 1, "message": None}
        self.assertFalse(manager._subscriber_stale("rgb1"))

        with manager._frame_condition:
            manager._frames["rgb1"] = {"received_at": time.time() - 10.0, "seq": 1, "message": None}
        self.assertTrue(manager._subscriber_stale("rgb1"))


if __name__ == "__main__":
    unittest.main()

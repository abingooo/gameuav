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

    def test_stream_generations_are_isolated_by_camera(self):
        manager = CameraManager(workspace_root=".")

        manager.stream_mjpeg("rgb", stream_owner="20.0.0.123")
        manager.stream_mjpeg("rgb1", stream_owner="20.0.0.123")

        self.assertEqual(manager._stream_generations[("20.0.0.123", "rgb")], 1)
        self.assertEqual(manager._stream_generations[("20.0.0.123", "rgb1")], 1)

    def test_new_stream_replaces_only_the_same_client_camera_pair(self):
        manager = CameraManager(workspace_root=".")

        manager.stream_mjpeg("rgb", stream_owner="20.0.0.123")
        manager.stream_mjpeg("rgb1", stream_owner="20.0.0.123")
        manager.stream_mjpeg("rgb", stream_owner="20.0.0.123")

        self.assertEqual(manager._stream_generations[("20.0.0.123", "rgb")], 2)
        self.assertEqual(manager._stream_generations[("20.0.0.123", "rgb1")], 1)


if __name__ == "__main__":
    unittest.main()

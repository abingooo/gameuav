import importlib.util
import math
from pathlib import Path
import sys
import threading
import types
import unittest
from unittest import mock

import numpy as np

from strategy.smpf.runtime.sam_client import SamMask, SamPrediction


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "ros_nodes/mission/smpf_bridge/scripts/smpf_bridge.py"

try:
    import realsense2_camera.msg  # noqa: F401
except ImportError:
    realsense_package = types.ModuleType("realsense2_camera")
    realsense_messages = types.ModuleType("realsense2_camera.msg")
    realsense_messages.Extrinsics = type("Extrinsics", (), {})
    realsense_package.msg = realsense_messages
    sys.modules["realsense2_camera"] = realsense_package
    sys.modules["realsense2_camera.msg"] = realsense_messages

SPEC = importlib.util.spec_from_file_location("smpf_bridge_follow_test", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _observation(grounding=9.0, metric=10.0, odom=10.01, color=None, depth=None):
    color = metric if color is None else color
    depth = metric if depth is None else depth
    return {
        "grounding_frame_stamp": grounding,
        "metric_frame_stamp": metric,
        "metric_color_stamp": color,
        "metric_depth_stamp": depth,
        "metric_odom_stamp": odom,
        "metric_rgbd_odom_skew_sec": max(abs(color - odom), abs(depth - odom)),
        "relocalized": True,
    }


class SmpfBridgeFollowFreshnessTest(unittest.TestCase):
    def test_metric_timing_accepts_declared_boundaries(self):
        timing = MODULE.validate_follow_metric_timing(
            8.0,
            9.08,
            9.0,
            10.0,
            max_frame_age_sec=1.0,
            max_rgbd_odom_skew_sec=0.08,
            color_stamp=9.0,
            depth_stamp=9.08,
        )
        self.assertAlmostEqual(timing["frame_age_sec"], 1.0)
        self.assertAlmostEqual(timing["rgbd_odom_skew_sec"], 0.08)
        self.assertAlmostEqual(timing["color_frame_age_sec"], 1.0)
        self.assertAlmostEqual(timing["depth_odom_skew_sec"], 0.08)

    def test_metric_frame_must_be_strictly_newer_than_grounding(self):
        for metric_stamp in (8.0, 7.99):
            with self.subTest(metric_stamp=metric_stamp), self.assertRaisesRegex(
                RuntimeError, "not newer"
            ):
                MODULE.validate_follow_metric_timing(8.0, metric_stamp, metric_stamp, 8.1)

    def test_metric_frame_age_and_odom_skew_fail_closed(self):
        with self.assertRaisesRegex(RuntimeError, "stale"):
            MODULE.validate_follow_metric_timing(8.0, 9.0, 9.0, 10.001)
        with self.assertRaisesRegex(RuntimeError, "skew"):
            MODULE.validate_follow_metric_timing(8.0, 9.0, 9.081, 9.1)

    def test_each_rgbd_member_must_meet_age_and_odom_skew_limits(self):
        with self.assertRaisesRegex(RuntimeError, "depth frame is stale"):
            MODULE.validate_follow_metric_timing(
                8.0,
                10.0,
                10.0,
                10.95,
                color_stamp=10.0,
                depth_stamp=9.90,
            )
        with self.assertRaisesRegex(RuntimeError, "depth/VINS.*skew"):
            MODULE.validate_follow_metric_timing(
                8.0,
                10.0,
                10.08,
                10.1,
                color_stamp=10.0,
                depth_stamp=9.92,
            )

    def test_frame_cache_retains_color_and_depth_stamps(self):
        bridge = MODULE.SmpfBridgeNode.__new__(MODULE.SmpfBridgeNode)
        bridge.bridge = mock.Mock()
        bridge.lock = threading.RLock()
        bridge.latest_frame = None
        bridge.last_error = None
        color = np.zeros((2, 3, 3), dtype=np.uint8)
        depth = np.ones((2, 3), dtype=np.float32)
        bridge.bridge.imgmsg_to_cv2.side_effect = (color, depth)
        color_msg = types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=MODULE.rospy.Time.from_sec(10.0)),
            encoding="bgr8",
        )
        depth_msg = types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=MODULE.rospy.Time.from_sec(10.08)),
            encoding="32FC1",
        )
        info_msg = types.SimpleNamespace(
            K=(1.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0),
            width=3,
            height=2,
        )

        bridge._frame_cb(color_msg, depth_msg, info_msg)

        self.assertIsInstance(bridge.latest_frame, MODULE.FrameSnapshot)
        self.assertAlmostEqual(bridge.latest_frame.color_stamp, 10.0)
        self.assertAlmostEqual(bridge.latest_frame.depth_stamp, 10.08)
        self.assertAlmostEqual(bridge.latest_frame.stamp, 10.08)
        self.assertIsNone(bridge.last_error)

    def test_follow_sam_requires_at_least_one_mask(self):
        with self.assertRaisesRegex(RuntimeError, "did not return a mask"):
            MODULE.select_follow_sam_mask(SamPrediction("person", ()))

    def test_follow_sam_selects_largest_mask_and_uses_first_for_ties(self):
        first = SamMask((0.0, 0.0, 2.0, 2.0), (1.0, 1.0), (), 4.0)
        largest = SamMask((0.0, 0.0, 3.0, 3.0), (1.5, 1.5), (), 9.0)
        tied = SamMask((1.0, 1.0, 4.0, 4.0), (2.5, 2.5), (), 9.0)
        self.assertIs(
            MODULE.select_follow_sam_mask(SamPrediction("person", (first, largest, tied))),
            largest,
        )

    @staticmethod
    def _bridge_for_gate():
        bridge = MODULE.SmpfBridgeNode.__new__(MODULE.SmpfBridgeNode)
        bridge.lock = threading.RLock()
        bridge.allow_execution = True
        bridge.runtime_execution_enabled = False
        bridge.inference_busy = False
        bridge.task = None
        bridge.pending_replan_at = None
        bridge.search_target = None
        bridge.executor = mock.Mock()
        bridge.executor.state = "DISABLED"
        bridge.stop_pub = mock.Mock()
        bridge.vlm_model_id = "test-vlm"
        bridge.llm_model_id = "test-llm"
        bridge.llm_reasoning_effort = "low"
        bridge._execution_preconditions = mock.Mock(return_value=(True, "ready"))
        bridge._publish_status = mock.Mock()
        bridge._log_event = mock.Mock()
        bridge._start_cycle = mock.Mock()
        return bridge

    def test_execution_cannot_be_enabled_after_task_submission(self):
        bridge = self._bridge_for_gate()
        bridge.inference_busy = True

        with mock.patch.object(
            MODULE.rospy.Time,
            "now",
            return_value=MODULE.rospy.Time.from_sec(10.0),
        ):
            bridge._enable_cb(types.SimpleNamespace(data=True))

        self.assertFalse(bridge.runtime_execution_enabled)
        bridge.executor.set_enabled.assert_called_once_with(False, mock.ANY)
        bridge._publish_status.assert_called_once_with(
            "EXECUTION_REJECTED",
            "execution must be enabled before submitting a task",
        )

    def test_task_freezes_explicit_dry_run_intent_at_submission(self):
        bridge = self._bridge_for_gate()
        bridge._execution_open = mock.Mock(return_value=True)
        message = types.SimpleNamespace(
            data='{"instruction":"Follow the chair","mode":"follow","execute":false}'
        )

        bridge._command_cb(message)

        self.assertFalse(bridge.task["execution_requested_at_submit"])
        bridge._start_cycle.assert_called_once_with()

    def test_abort_atomically_closes_execution_and_publishes_stop(self):
        bridge = self._bridge_for_gate()
        bridge.runtime_execution_enabled = True
        bridge.task = {"task_id": "active-task"}
        bridge.executor.state = "WAITING_ARRIVAL"

        with mock.patch.object(
            MODULE.rospy.Time,
            "now",
            return_value=MODULE.rospy.Time.from_sec(10.0),
        ):
            bridge._control_cb(types.SimpleNamespace(data="abort"))

        self.assertFalse(bridge.runtime_execution_enabled)
        bridge.executor.abort.assert_called_once()
        bridge.executor.set_enabled.assert_called_once_with(False, mock.ANY)
        bridge.stop_pub.publish.assert_called_once()
        self.assertIsNone(bridge.task)

    def test_explicit_dry_run_cannot_publish_missing_target_search_motion(self):
        bridge = self._bridge_for_gate()
        bridge._execution_open = mock.Mock(return_value=True)
        bridge._publish_pose_goal = mock.Mock()
        task = {
            "task_id": "dry-run-task",
            "mode": "follow",
            "execution_requested_at_submit": False,
        }

        bridge._target_missing(task, {"x": 0.0, "y": 0.0, "z": 1.0, "yaw": 0.0})

        bridge._publish_pose_goal.assert_not_called()
        bridge._publish_status.assert_called_once_with(
            "SEARCH_REQUIRED",
            "target not visible; execution was not requested when the task was submitted",
        )

    def test_execute_command_field_must_be_boolean(self):
        bridge = self._bridge_for_gate()
        with self.assertRaisesRegex(ValueError, "execute must be a boolean"):
            bridge._parse_command('{"instruction":"Follow the chair","execute":1}')

    @staticmethod
    def _bridge_for_publish(observation):
        bridge = MODULE.SmpfBridgeNode.__new__(MODULE.SmpfBridgeNode)
        bridge.lock = threading.RLock()
        bridge.follow_metric_frame_max_age_sec = 1.0
        bridge.follow_metric_odom_skew_sec = 0.08
        bridge.goal_pub = mock.Mock()
        bridge.stop_pub = mock.Mock()
        bridge.last_yaw_refresh_at = None
        bridge.latest_odom = {"yaw": 0.0}
        bridge.pending_replan_at = None
        bridge.search_target = None
        bridge.task = {
            "task_id": "follow-task",
            "mode": "follow",
            "follow_metric_observation": observation,
        }
        bridge.executor = mock.Mock()
        bridge._execution_open = mock.Mock(return_value=True)
        bridge._log_event = mock.Mock()
        return bridge

    def test_stale_metric_frame_cannot_publish_and_clears_follow_execution(self):
        observation = _observation()
        bridge = self._bridge_for_publish(observation)
        event = {"type": "publish_goal", "goal": (1.0, 2.0, 3.0), "yaw": 0.2, "index": 0}
        with mock.patch.object(
            MODULE.rospy.Time,
            "now",
            return_value=MODULE.rospy.Time.from_sec(11.001),
        ), self.assertRaisesRegex(RuntimeError, "stale"):
            bridge._handle_execution_events((event,))

        bridge.goal_pub.publish.assert_not_called()
        bridge.stop_pub.publish.assert_called_once()
        bridge.executor.abort.assert_called_once()
        self.assertIsNone(bridge.task)

    def test_missing_metric_metadata_cannot_bypass_follow_publish_gate(self):
        bridge = self._bridge_for_publish(None)
        event = {"type": "publish_goal", "goal": (1.0, 2.0, 3.0), "yaw": 0.2, "index": 0}
        with mock.patch.object(
            MODULE.rospy.Time,
            "now",
            return_value=MODULE.rospy.Time.from_sec(11.0),
        ), self.assertRaisesRegex(RuntimeError, "metadata is unavailable"):
            bridge._handle_execution_events((event,))

        bridge.goal_pub.publish.assert_not_called()
        bridge.stop_pub.publish.assert_called_once()
        bridge.executor.abort.assert_called_once()
        self.assertIsNone(bridge.task)

    def test_partial_metric_metadata_cannot_bypass_follow_publish_gate(self):
        observation = _observation()
        del observation["metric_depth_stamp"]
        bridge = self._bridge_for_publish(observation)
        event = {"type": "publish_goal", "goal": (1.0, 2.0, 3.0), "yaw": 0.2, "index": 0}
        with mock.patch.object(
            MODULE.rospy.Time,
            "now",
            return_value=MODULE.rospy.Time.from_sec(10.5),
        ), self.assertRaisesRegex(RuntimeError, "metadata is unavailable"):
            bridge._handle_execution_events((event,))

        bridge.goal_pub.publish.assert_not_called()
        bridge.stop_pub.publish.assert_called_once()
        self.assertIsNone(bridge.task)

    def test_executor_timeout_and_error_stop_follow_once(self):
        for state in ("TIMEOUT", "ERROR"):
            with self.subTest(state=state):
                bridge = self._bridge_for_publish(_observation())
                bridge.task.update({"cycles": 1, "max_cycles": 10})
                bridge.executor.yaw_error = math.nan
                bridge._publish_status = mock.Mock()
                event = {"type": "terminal", "state": state, "reason": "failed"}

                bridge._handle_execution_events((event, event))

                bridge.stop_pub.publish.assert_called_once()
                self.assertIsNone(bridge.task)

    def test_follow_timeout_and_unsafe_stop_while_success_does_not(self):
        for decision in (MODULE.FOLLOW_TIMEOUT, MODULE.FOLLOW_UNSAFE):
            with self.subTest(decision=decision):
                bridge = self._bridge_for_publish(_observation())

                self.assertTrue(bridge._complete_follow_decision("follow-task", decision))
                self.assertFalse(bridge._complete_follow_decision("follow-task", decision))

                bridge.stop_pub.publish.assert_called_once()
                self.assertIsNone(bridge.task)

        bridge = self._bridge_for_publish(_observation())
        self.assertTrue(bridge._complete_follow_decision("follow-task", MODULE.FOLLOW_SUCCESS))
        bridge.stop_pub.publish.assert_not_called()
        self.assertIsNone(bridge.task)

    def test_static_goal_publish_has_no_follow_completion_age_gate(self):
        bridge = self._bridge_for_publish(None)
        with mock.patch.object(
            MODULE.rospy.Time,
            "now",
            return_value=MODULE.rospy.Time.from_sec(1000.0),
        ):
            timing = bridge._publish_pose_goal((1.0, 2.0, 3.0), 0.2)

        self.assertIsNone(timing)
        bridge.goal_pub.publish.assert_called_once()


if __name__ == "__main__":
    unittest.main()

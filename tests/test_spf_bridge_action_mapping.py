import importlib.util
import json
import math
import unittest
from pathlib import Path
from unittest import mock

import tf
from nav_msgs.msg import Odometry


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "ros_nodes/mission/see_point_fly_bridge/scripts/see_point_fly_bridge.py"
SPEC = importlib.util.spec_from_file_location("see_point_fly_bridge", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def yaw_from_pose(pose):
    q = pose.orientation
    return tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


class SpfBridgeActionMappingTest(unittest.TestCase):
    def make_bridge(self, yaw=0.0):
        bridge = MODULE.SeePointFlyBridge.__new__(MODULE.SeePointFlyBridge)
        bridge.execution_lock = MODULE.threading.RLock()
        bridge.last_odom_msg = Odometry()
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        orientation = bridge.last_odom_msg.pose.pose.orientation
        orientation.x, orientation.y, orientation.z, orientation.w = quaternion
        bridge.frame_id = "world"
        bridge.max_step_xy = 10.0
        bridge.max_step_z = 10.0
        bridge.min_goal_distance_xy = 0.0
        bridge.min_z = -10.0
        bridge.max_z = 10.0
        bridge.stop_pub = mock.Mock()
        return bridge

    def make_execution_bridge(self):
        bridge = self.make_bridge()
        bridge.manual_enable_required = True
        bridge.enabled = True
        bridge.require_armed_for_execution = True
        bridge.mavros_state_timeout_sec = 1.0
        bridge.last_mavros_state = MODULE.State(connected=True, armed=True)
        bridge.last_mavros_state_time = 10.0
        bridge.last_image_msg = object()
        bridge.last_image_time = 10.0
        bridge.last_odom_time = 10.0
        bridge.image_timeout_sec = 1.0
        bridge.odom_timeout_sec = 1.0
        bridge.max_abs_odom_position = 100.0
        bridge.rate_limit_sec = 0.0
        bridge.last_goal_time = 0.0
        bridge.last_projection = None
        bridge.goal_pub = mock.Mock()
        bridge.publish_status = mock.Mock()
        bridge.publish_last_goal = mock.Mock()
        return bridge

    def test_normal_action_faces_author_selected_direction(self):
        bridge = self.make_bridge()
        with mock.patch.object(MODULE.rospy.Time, "now", return_value=MODULE.rospy.Time(1)):
            goal, yaw_right_deg = bridge.action_to_goal(
                {"dx": 1.0, "dy": 1.0, "dz": 0.0}
            )

        self.assertAlmostEqual(goal.pose.position.x, 1.0)
        self.assertAlmostEqual(goal.pose.position.y, -1.0)
        self.assertAlmostEqual(yaw_right_deg, 45.0)
        self.assertAlmostEqual(yaw_from_pose(goal.pose), -math.pi / 4.0)

    def test_yaw_only_converts_tello_right_positive_to_enu(self):
        bridge = self.make_bridge(yaw=math.pi / 6.0)
        with mock.patch.object(MODULE.rospy.Time, "now", return_value=MODULE.rospy.Time(1)):
            goal, yaw_right_deg = bridge.action_to_goal(
                {"yaw_only": True, "yaw_deg": 45.0}
            )

        self.assertAlmostEqual(goal.pose.position.x, 0.0)
        self.assertAlmostEqual(goal.pose.position.y, 0.0)
        self.assertAlmostEqual(yaw_right_deg, 45.0)
        self.assertAlmostEqual(yaw_from_pose(goal.pose), -math.pi / 12.0)

    def test_worker_request_preserves_natural_language_command(self):
        bridge = self.make_bridge()
        bridge.worker_url = "http://127.0.0.1:9310/infer"
        bridge.worker_timeout_sec = 30.0
        captured = {}

        class FakeResponse:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            def read(self):
                return b'{"ok": true, "action": {"dx": 0, "dy": 1, "dz": 0}}'

        def fake_urlopen(request, timeout):
            captured["payload"] = json.loads(request.data.decode("utf-8"))
            captured["timeout"] = timeout
            return FakeResponse()

        command = "It's raining, head to the comfiest chair that looks like it'll keep you dry!"
        with mock.patch.object(MODULE.urllib.request, "urlopen", side_effect=fake_urlopen):
            response = bridge.call_worker(command, "encoded-image")

        self.assertTrue(response["ok"])
        self.assertEqual(captured["payload"]["command"], command)
        self.assertEqual(captured["timeout"], 30.0)

    def test_vehicle_gate_requires_fresh_connected_and_armed_mavros_state(self):
        bridge = self.make_bridge()
        bridge.require_armed_for_execution = True
        bridge.mavros_state_timeout_sec = 1.0
        bridge.last_mavros_state = None
        bridge.last_mavros_state_time = None
        self.assertEqual(bridge.vehicle_state_error(10.0), "no MAVROS state")

        bridge.last_mavros_state = MODULE.State(connected=True, armed=True)
        bridge.last_mavros_state_time = 8.0
        self.assertEqual(bridge.vehicle_state_error(10.0), "stale MAVROS state")

        bridge.last_mavros_state_time = 10.0
        bridge.last_mavros_state.connected = False
        self.assertEqual(bridge.vehicle_state_error(10.0), "MAVROS is disconnected")

        bridge.last_mavros_state.connected = True
        bridge.last_mavros_state.armed = False
        self.assertEqual(bridge.vehicle_state_error(10.0), "PX4 is not armed")

        bridge.last_mavros_state.armed = True
        self.assertIsNone(bridge.vehicle_state_error(10.0))

    def test_disarm_transition_closes_gate_and_stops_ego(self):
        bridge = self.make_bridge()
        bridge.enabled = True
        bridge.last_mavros_state = MODULE.State(connected=True, armed=True)
        bridge.last_mavros_state_time = 9.0
        bridge.publish_status = mock.Mock()

        with mock.patch.object(MODULE.time, "time", return_value=10.0):
            bridge.mavros_state_callback(MODULE.State(connected=True, armed=False))

        self.assertFalse(bridge.enabled)
        bridge.publish_status.assert_called_once_with(
            "execution gate closed: MAVROS disconnected or PX4 disarmed"
        )
        bridge.stop_pub.publish.assert_called_once()

    def test_explicit_gate_cannot_open_while_disarmed(self):
        bridge = self.make_bridge()
        bridge.enabled = False
        bridge.require_armed_for_execution = True
        bridge.mavros_state_timeout_sec = 1.0
        bridge.last_mavros_state = MODULE.State(connected=True, armed=False)
        bridge.last_mavros_state_time = 10.0
        bridge.publish_status = mock.Mock()

        with mock.patch.object(MODULE.time, "time", return_value=10.0):
            bridge.enable_callback(MODULE.Bool(data=True))

        self.assertFalse(bridge.enabled)
        bridge.publish_status.assert_called_once_with("enable rejected: PX4 is not armed")

    def test_disabling_explicit_gate_stops_ego_once(self):
        bridge = self.make_bridge()
        bridge.enabled = True
        bridge.publish_status = mock.Mock()

        bridge.enable_callback(MODULE.Bool(data=False))

        self.assertFalse(bridge.enabled)
        bridge.publish_status.assert_called_once_with("enabled=False")
        bridge.stop_pub.publish.assert_called_once()

        bridge.enable_callback(MODULE.Bool(data=False))
        bridge.stop_pub.publish.assert_called_once()

    def test_rejected_reenable_stops_previously_authorized_session(self):
        bridge = self.make_execution_bridge()
        bridge.last_mavros_state_time = 8.0

        with mock.patch.object(MODULE.time, "time", return_value=10.0):
            bridge.enable_callback(MODULE.Bool(data=True))

        self.assertFalse(bridge.enabled)
        bridge.publish_status.assert_called_once_with(
            "enable rejected: stale MAVROS state"
        )
        bridge.stop_pub.publish.assert_called_once()

    def test_watchdog_closes_gate_and_stops_on_stale_mavros_state(self):
        bridge = self.make_execution_bridge()
        bridge.last_mavros_state_time = 8.0

        with mock.patch.object(MODULE.time, "time", return_value=10.0):
            bridge.watchdog_callback(None)

        self.assertFalse(bridge.enabled)
        bridge.publish_status.assert_called_once_with(
            "execution gate closed: stale MAVROS state"
        )
        bridge.stop_pub.publish.assert_called_once()

    def test_gate_is_rechecked_after_goal_computation_before_publish(self):
        bridge = self.make_execution_bridge()

        def close_gate(goal):
            bridge.enable_callback(MODULE.Bool(data=False))
            return goal

        bridge.project_goal_if_needed = mock.Mock(side_effect=close_gate)
        with mock.patch.object(MODULE.time, "time", return_value=10.0), mock.patch.object(
            MODULE.rospy.Time,
            "now",
            return_value=MODULE.rospy.Time(10),
        ):
            published = bridge.publish_action(
                "fly to the chair",
                {"dx": 0.0, "dy": 1.0, "dz": 0.0},
            )

        self.assertFalse(published)
        bridge.goal_pub.publish.assert_not_called()
        bridge.publish_last_goal.assert_not_called()


if __name__ == "__main__":
    unittest.main()

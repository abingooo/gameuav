import importlib.util
import threading
import unittest
from pathlib import Path
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = (
    ROOT
    / "ros_nodes/control/gameuav_control_interface/scripts/control_interface_node.py"
)
SPEC = importlib.util.spec_from_file_location("control_interface_spf_gate", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class ControlInterfaceSpfGateTest(unittest.TestCase):
    def make_node(self):
        node = MODULE.ControlInterfaceNode.__new__(MODULE.ControlInterfaceNode)
        node.spf_lock = threading.RLock()
        node.spf_execution_enabled = False
        node.spf_mavros_state = None
        node.spf_mavros_state_stamp = MODULE.rospy.Time(0)
        node.spf_mavros_state_timeout_sec = 1.0
        node.attitude_guard_enabled = False
        node.attitude_timeout = 0.5
        node.max_roll_pitch_error_deg = 5.0
        node.attitude_reference = None
        node.attitude_reference_stamp = MODULE.rospy.Time(0)
        node.odom = MODULE.Odometry()
        node.odom.pose.pose.orientation.w = 1.0
        node.odom.pose.pose.position.z = 1.0
        node.odom_stamp = MODULE.rospy.Time(10)
        node.motion_rearm_required = False
        node.latest_ego_cmd = None
        node.latest_ego_stamp = MODULE.rospy.Time(0)
        node.mode = "ego_passthrough"
        node.direct_target = None
        node.direct_yaw = 0.0
        node.direct_stamp = MODULE.rospy.Time(0)
        node.speed_target = None
        node.speed_yaw_rate = 0.0
        node.speed_reference = None
        node.speed_stamp = MODULE.rospy.Time(0)
        node.last_timer_stamp = MODULE.rospy.Time(9.9)
        node.last_status_stamp = MODULE.rospy.Time(0)
        node.direct_position_timeout = 5.0
        node.spf_position_timeout = 0.0
        node.speed_timeout = 0.6
        node.ego_cmd_timeout = 0.6
        node.max_speed = 1.0
        node.max_vertical_speed = 0.5
        node.max_position_step = 3.0
        node.min_z = 0.05
        node.max_z = 3.0
        node.output_pub = mock.Mock()
        node.goal_pub = mock.Mock()
        node.yaw_pub = mock.Mock()
        node.status_pub = mock.Mock()
        return node

    @staticmethod
    def goal(x=1.0, y=0.0, z=1.0):
        msg = MODULE.PoseStamped()
        msg.header.frame_id = "world"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        return msg

    def enable_spf(self, node, now=10.0):
        with mock.patch.object(
            MODULE.rospy.Time,
            "now",
            return_value=MODULE.rospy.Time.from_sec(now),
        ):
            node.mavros_state_cb(MODULE.State(connected=True, armed=True))
            node.spf_enable_cb(MODULE.Bool(data=True))

    def test_spf_target_requires_gate_and_fresh_armed_state(self):
        node = self.make_node()
        with mock.patch.object(
            MODULE.rospy.Time,
            "now",
            return_value=MODULE.rospy.Time(10),
        ):
            node.spf_position_cb(self.goal())
        self.assertEqual(node.mode, "ego_passthrough")
        self.assertIsNone(node.direct_target)

        self.enable_spf(node)
        with mock.patch.object(
            MODULE.rospy.Time,
            "now",
            return_value=MODULE.rospy.Time(10),
        ):
            node.spf_position_cb(self.goal())

        self.assertTrue(node.spf_execution_enabled)
        self.assertEqual(node.mode, "spf_position")
        self.assertEqual(node.direct_target, (1.0, 0.0, 1.0))

    def test_disarm_and_stale_state_clear_spf_target_and_stop_output(self):
        node = self.make_node()
        self.enable_spf(node)
        with mock.patch.object(
            MODULE.rospy.Time,
            "now",
            return_value=MODULE.rospy.Time(10),
        ):
            node.spf_position_cb(self.goal())
            node.mavros_state_cb(MODULE.State(connected=True, armed=False))

        self.assertFalse(node.spf_execution_enabled)
        self.assertEqual(node.mode, "ego_passthrough")
        self.assertIsNone(node.direct_target)

        self.enable_spf(node)
        with mock.patch.object(
            MODULE.rospy.Time,
            "now",
            return_value=MODULE.rospy.Time(10),
        ):
            node.spf_position_cb(self.goal())
        node.output_pub.reset_mock()
        with mock.patch.object(
            MODULE.rospy.Time,
            "now",
            return_value=MODULE.rospy.Time.from_sec(11.1),
        ):
            node.timer_cb(None)

        self.assertFalse(node.spf_execution_enabled)
        self.assertEqual(node.mode, "ego_passthrough")
        self.assertIsNone(node.direct_target)
        node.output_pub.publish.assert_not_called()

    def test_spf_gate_changes_do_not_clear_or_block_direct_position(self):
        node = self.make_node()
        with mock.patch.object(
            MODULE.rospy.Time,
            "now",
            return_value=MODULE.rospy.Time(10),
        ):
            node.position_cb(self.goal(x=2.0))
            node.spf_enable_cb(MODULE.Bool(data=False))
            node.mavros_state_cb(MODULE.State(connected=True, armed=False))

        self.assertEqual(node.mode, "direct_position")
        self.assertEqual(node.direct_target, (2.0, 0.0, 1.0))
        with mock.patch.object(
            MODULE.rospy.Time,
            "now",
            return_value=MODULE.rospy.Time(10),
        ):
            node.timer_cb(None)
        node.output_pub.publish.assert_called_once()

    def test_concurrent_spf_close_cannot_leave_a_target_active(self):
        node = self.make_node()
        self.enable_spf(node)
        entered = threading.Event()
        release = threading.Event()
        original_resolve = node._resolve_pose

        def blocking_resolve(msg, use_current_as_default):
            entered.set()
            self.assertTrue(release.wait(2.0))
            return original_resolve(msg, use_current_as_default)

        node._resolve_pose = blocking_resolve
        goal_thread = threading.Thread(target=node.spf_position_cb, args=(self.goal(),))
        close_thread = threading.Thread(
            target=node.spf_enable_cb,
            args=(MODULE.Bool(data=False),),
        )
        with mock.patch.object(
            MODULE.rospy.Time,
            "now",
            return_value=MODULE.rospy.Time(10),
        ):
            goal_thread.start()
            self.assertTrue(entered.wait(2.0))
            close_thread.start()
            release.set()
            goal_thread.join(2.0)
            close_thread.join(2.0)

        self.assertFalse(goal_thread.is_alive())
        self.assertFalse(close_thread.is_alive())
        self.assertFalse(node.spf_execution_enabled)
        self.assertEqual(node.mode, "ego_passthrough")
        self.assertIsNone(node.direct_target)

    def test_concurrent_spf_close_preserves_new_direct_target(self):
        node = self.make_node()
        self.enable_spf(node)
        entered = threading.Event()
        release = threading.Event()
        original_resolve = node._resolve_pose

        def blocking_resolve(msg, use_current_as_default):
            entered.set()
            self.assertTrue(release.wait(2.0))
            return original_resolve(msg, use_current_as_default)

        node._resolve_pose = blocking_resolve
        direct_thread = threading.Thread(target=node.position_cb, args=(self.goal(x=2.0),))
        close_thread = threading.Thread(
            target=node.spf_enable_cb,
            args=(MODULE.Bool(data=False),),
        )
        with mock.patch.object(
            MODULE.rospy.Time,
            "now",
            return_value=MODULE.rospy.Time(10),
        ):
            direct_thread.start()
            self.assertTrue(entered.wait(2.0))
            close_thread.start()
            release.set()
            direct_thread.join(2.0)
            close_thread.join(2.0)

        self.assertFalse(direct_thread.is_alive())
        self.assertFalse(close_thread.is_alive())
        self.assertFalse(node.spf_execution_enabled)
        self.assertEqual(node.mode, "direct_position")
        self.assertEqual(node.direct_target, (2.0, 0.0, 1.0))


if __name__ == "__main__":
    unittest.main()

import importlib.util
import time
import unittest
from pathlib import Path
from unittest import mock


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "ros_nodes/mission/see_point_fly_bridge/scripts/spf_task_executor.py"
)
SPEC = importlib.util.spec_from_file_location("spf_task_executor", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def odom(now, x=0.0, y=0.0, z=1.0, speed=0.0):
    return {"stamp": now, "x": x, "y": y, "z": z, "speed": speed}


def vehicle_state(now, armed=True, connected=True):
    return {
        "stamp": now,
        "connected": connected,
        "armed": armed,
        "mode": "AUTO.LOITER",
    }


def goal(command, stamp, x=1.0, y=0.0, z=1.0):
    return {
        "stamp": stamp,
        "command": command,
        "goal": {"x": x, "y": y, "z": z},
    }


class TaskLoopTest(unittest.TestCase):
    def make_loop(self, **overrides):
        config = {
            "goal_ack_timeout_sec": 5.0,
            "goal_timeout_sec": 10.0,
            "task_timeout_sec": 60.0,
            "cycle_delay_sec": 0.5,
            "arrival_settle_sec": 1.0,
            "goal_tolerance_xy": 0.25,
            "goal_tolerance_z": 0.2,
            "arrival_max_speed": 0.25,
            "odom_timeout_sec": 1.0,
            "min_start_z": 0.4,
            "max_start_z": 1.5,
            "start_max_speed": 0.5,
            "require_armed_for_start": True,
            "vehicle_state_timeout_sec": 1.0,
            "allow_tabletop_start_disarmed": False,
            "tabletop_min_start_z": -0.2,
            "max_cycles": 3,
        }
        config.update(overrides)
        return MODULE.TaskLoop(**config)

    def test_disabled_by_default_and_requires_hovering_odom(self):
        loop = self.make_loop()
        with self.assertRaises(MODULE.TaskLoopError):
            loop.start("fly to the chair", 10.0, odom(10.0))

        loop.set_enabled(True, 10.0)
        with self.assertRaises(MODULE.TaskLoopError):
            loop.start(
                "fly to the chair",
                10.0,
                odom(10.0, z=0.1),
                vehicle_state=vehicle_state(10.0),
            )
        loop.record_rejection("vehicle must already be hovering inside the configured altitude range", 10.0)
        self.assertEqual(
            loop.status()["last_rejection"],
            "vehicle must already be hovering inside the configured altitude range",
        )
        with self.assertRaises(MODULE.TaskLoopError):
            loop.start(
                "fly to the chair",
                10.0,
                odom(10.0, speed=0.8),
                vehicle_state=vehicle_state(10.0),
            )
        loop.set_enabled(False, 10.1)
        self.assertIsNone(loop.status()["last_rejection"])

    def test_tabletop_start_is_allowed_only_while_disarmed(self):
        loop = self.make_loop(
            require_armed_for_start=False,
            allow_tabletop_start_disarmed=True,
        )
        loop.set_enabled(True, 10.0)
        events = loop.start(
            "fly to the chair",
            10.0,
            odom(10.0, z=0.05),
            vehicle_state=vehicle_state(10.0, armed=False),
        )
        self.assertEqual(events, [("publish_command", "fly to the chair")])
        self.assertTrue(loop.status()["tabletop_start_allowed"])

        loop.control("abort", 10.1)
        loop.control("reset", 10.2)
        with self.assertRaises(MODULE.TaskLoopError):
            loop.start(
                "fly to the chair",
                10.3,
                odom(10.3, z=0.05),
                vehicle_state=vehicle_state(10.3, armed=True),
            )

    def test_start_requires_fresh_connected_and_armed_vehicle_state(self):
        loop = self.make_loop()
        loop.set_enabled(True, 10.0)
        for state in (
            None,
            vehicle_state(8.0),
            vehicle_state(10.0, connected=False),
            vehicle_state(10.0, armed=False),
        ):
            with self.subTest(state=state):
                with self.assertRaises(MODULE.TaskLoopError):
                    loop.start(
                        "fly to the chair",
                        10.0,
                        odom(10.0),
                        vehicle_state=state,
                    )

    def test_arrival_settle_requests_next_spf_cycle(self):
        loop = self.make_loop()
        loop.set_enabled(True, 10.0)
        loop.record_rejection("old rejection", 9.5)
        events = loop.start(
            "fly to the chair",
            10.0,
            odom(10.0),
            vehicle_state=vehicle_state(10.0),
            task_id="task-1",
        )
        self.assertEqual(events, [("publish_command", "fly to the chair")])
        self.assertEqual(loop.state, "WAITING_GOAL")
        self.assertIsNone(loop.status()["last_rejection"])

        self.assertTrue(loop.receive_goal(goal("fly to the chair", 10.1), 10.1))
        self.assertEqual(loop.state, "WAITING_ARRIVAL")
        self.assertEqual(loop.tick(10.2, odom(10.2, x=0.9)), [])
        self.assertEqual(loop.reason, "local goal reached; waiting to settle")
        self.assertEqual(loop.tick(11.3, odom(11.3, x=0.9)), [])
        self.assertEqual(loop.state, "WAITING_NEXT")

        events = loop.tick(11.8, odom(11.8, x=0.9))
        self.assertEqual(events, [("publish_command", "fly to the chair")])
        self.assertEqual(loop.cycle_count, 2)
        self.assertEqual(loop.state, "WAITING_GOAL")

    def test_ignores_stale_or_different_goal(self):
        loop = self.make_loop()
        loop.set_enabled(True, 10.0)
        loop.start(
            "fly to the chair",
            10.0,
            odom(10.0),
            vehicle_state=vehicle_state(10.0),
        )
        self.assertFalse(loop.receive_goal(goal("other task", 10.1), 10.1))
        self.assertFalse(loop.receive_goal(goal("fly to the chair", 9.0), 10.1))
        self.assertEqual(loop.state, "WAITING_GOAL")

    def test_disable_aborts_and_prevents_future_cycles(self):
        loop = self.make_loop()
        loop.set_enabled(True, 10.0)
        loop.start(
            "fly to the chair",
            10.0,
            odom(10.0),
            vehicle_state=vehicle_state(10.0),
        )
        loop.set_enabled(False, 10.2)
        self.assertEqual(loop.state, "DISABLED")
        self.assertFalse(loop.status()["active"])
        self.assertEqual(loop.tick(20.0, odom(20.0)), [])

    def test_goal_ack_and_local_goal_timeout(self):
        loop = self.make_loop()
        loop.set_enabled(True, 10.0)
        loop.start(
            "fly to the chair",
            10.0,
            odom(10.0),
            vehicle_state=vehicle_state(10.0),
        )
        loop.tick(15.1, odom(15.1))
        self.assertEqual(loop.state, "ERROR")

        loop.control("reset", 16.0)
        loop.start(
            "fly to the chair",
            16.0,
            odom(16.0),
            vehicle_state=vehicle_state(16.0),
        )
        loop.receive_goal(goal("fly to the chair", 16.1), 16.1)
        loop.tick(26.2, odom(26.2, x=0.5))
        self.assertEqual(loop.state, "TIMEOUT")

    def test_operator_completion_is_explicit(self):
        loop = self.make_loop()
        loop.set_enabled(True, 10.0)
        loop.start(
            "fly to the chair",
            10.0,
            odom(10.0),
            vehicle_state=vehicle_state(10.0),
        )
        loop.control("complete", 10.5)
        self.assertEqual(loop.state, "SUCCESS")
        self.assertEqual(loop.tick(20.0, odom(20.0)), [])

    def test_disarm_aborts_active_node_loop_and_commands_hold(self):
        node = MODULE.SpfTaskExecutorNode.__new__(MODULE.SpfTaskExecutorNode)
        node.loop = self.make_loop()
        node.loop.set_enabled(True, 10.0)
        node.loop.start(
            "fly to the chair",
            10.0,
            odom(10.0),
            vehicle_state=vehicle_state(10.0),
        )
        node.lock = MODULE.threading.RLock()
        node.stop_pub = mock.Mock()
        node.publish_status = mock.Mock()

        with mock.patch.object(MODULE.time, "time", return_value=10.1):
            node.mavros_state_callback(MODULE.State(connected=True, armed=False))

        self.assertEqual(node.loop.state, "DISABLED")
        self.assertFalse(node.loop.enabled)
        node.stop_pub.publish.assert_called_once()
        node.publish_status.assert_called_once_with(force=True)

    def test_disarm_closes_idle_session_gate(self):
        node = MODULE.SpfTaskExecutorNode.__new__(MODULE.SpfTaskExecutorNode)
        node.loop = self.make_loop()
        node.loop.set_enabled(True, 10.0)
        node.lock = MODULE.threading.RLock()
        node.stop_pub = mock.Mock()
        node.publish_status = mock.Mock()

        with mock.patch.object(MODULE.time, "time", return_value=10.1):
            node.mavros_state_callback(MODULE.State(connected=True, armed=False))

        self.assertEqual(node.loop.state, "DISABLED")
        self.assertFalse(node.loop.enabled)
        node.stop_pub.publish.assert_called_once()
        node.publish_status.assert_called_once_with(force=True)

    def test_enable_rejects_disarmed_vehicle(self):
        node = MODULE.SpfTaskExecutorNode.__new__(MODULE.SpfTaskExecutorNode)
        node.loop = self.make_loop()
        node.lock = MODULE.threading.RLock()
        node.vehicle_state = vehicle_state(10.0, armed=False)
        node.stop_pub = mock.Mock()
        node.publish_status = mock.Mock()

        with mock.patch.object(MODULE.time, "time", return_value=10.0):
            node.enable_callback(MODULE.Bool(data=True))

        self.assertFalse(node.loop.enabled)
        self.assertEqual(node.loop.state, "DISABLED")
        self.assertIn("armed MAVROS state", node.loop.last_rejection)
        node.stop_pub.publish.assert_not_called()

    def test_rejected_reenable_stops_previously_enabled_session(self):
        node = MODULE.SpfTaskExecutorNode.__new__(MODULE.SpfTaskExecutorNode)
        node.loop = self.make_loop()
        node.loop.set_enabled(True, 8.0)
        node.lock = MODULE.threading.RLock()
        node.vehicle_state = vehicle_state(8.0)
        node.stop_pub = mock.Mock()
        node.publish_status = mock.Mock()

        with mock.patch.object(MODULE.time, "time", return_value=10.0):
            node.enable_callback(MODULE.Bool(data=True))

        self.assertFalse(node.loop.enabled)
        self.assertEqual(node.loop.state, "DISABLED")
        node.stop_pub.publish.assert_called_once()

    def test_timer_closes_active_task_and_stops_when_mavros_state_is_stale(self):
        node = MODULE.SpfTaskExecutorNode.__new__(MODULE.SpfTaskExecutorNode)
        node.loop = self.make_loop()
        node.loop.set_enabled(True, 10.0)
        node.loop.start(
            "fly to the chair",
            10.0,
            odom(10.0),
            vehicle_state=vehicle_state(10.0),
        )
        node.lock = MODULE.threading.RLock()
        node.vehicle_state = vehicle_state(8.0)
        node.odom = odom(10.0)
        node.stop_pub = mock.Mock()
        node.publish_status = mock.Mock()
        node.handle_events = mock.Mock()

        with mock.patch.object(MODULE.time, "time", return_value=10.0):
            node.timer_callback(None)

        self.assertFalse(node.loop.enabled)
        self.assertEqual(node.loop.state, "DISABLED")
        node.stop_pub.publish.assert_called_once()
        node.handle_events.assert_not_called()


if __name__ == "__main__":
    unittest.main()

import unittest

from strategy.smpf.runtime.execution import ExecutionStateError, WaypointExecutionLoop


def _odom(x, y, z, speed, stamp, yaw=None):
    result = {"x": x, "y": y, "z": z, "speed": speed, "stamp": stamp}
    if yaw is not None:
        result["yaw"] = yaw
    return result


class SmpfExecutionTest(unittest.TestCase):
    def setUp(self):
        self.loop = WaypointExecutionLoop(arrival_settle_sec=0.5, odom_timeout_sec=1.0)

    def test_disabled_loop_rejects_path(self):
        with self.assertRaises(ExecutionStateError):
            self.loop.start("task", [(1, 0, 1)], 0.0)

    def test_path_advances_only_after_arrival_and_settle(self):
        self.loop.set_enabled(True, 0.0)
        events = self.loop.start("task", [(1, 0, 1), (2, 0, 1)], 0.0)
        self.assertEqual(events[0]["goal"], (1.0, 0.0, 1.0))
        self.assertEqual(self.loop.tick(1.0, _odom(1, 0, 1, 0.1, 1.0)), [])
        events = self.loop.tick(1.6, _odom(1, 0, 1, 0.1, 1.6))
        self.assertEqual(events[0]["goal"], (2.0, 0.0, 1.0))
        self.loop.tick(2.0, _odom(2, 0, 1, 0.1, 2.0))
        events = self.loop.tick(2.6, _odom(2, 0, 1, 0.1, 2.6))
        self.assertEqual(events[0]["state"], "SUCCESS")

    def test_zero_settle_advances_on_first_in_range_sample(self):
        loop = WaypointExecutionLoop(arrival_settle_sec=0.0, odom_timeout_sec=1.0)
        loop.set_enabled(True, 0.0)
        loop.start("task", [(1, 0, 1), (2, 0, 1)], 0.0)

        events = loop.tick(0.2, _odom(1, 0, 1, 0.1, 0.2))
        self.assertEqual(events[0]["goal"], (2.0, 0.0, 1.0))

    def test_intermediate_waypoint_does_not_use_goal_timeout(self):
        loop = WaypointExecutionLoop(
            goal_timeout_sec=1.0,
            task_timeout_sec=10.0,
            arrival_settle_sec=0.5,
            odom_timeout_sec=1.0,
        )
        loop.set_enabled(True, 0.0)
        loop.start("task", [(1, 0, 1), (2, 0, 1)], 0.0)

        self.assertEqual(loop.tick(2.0, _odom(0, 0, 1, 0.0, 2.0)), [])
        self.assertEqual(loop.state, "WAITING_ARRIVAL")

    def test_final_waypoint_still_uses_goal_timeout(self):
        loop = WaypointExecutionLoop(
            goal_timeout_sec=1.0,
            task_timeout_sec=10.0,
            arrival_settle_sec=0.5,
            odom_timeout_sec=1.0,
        )
        loop.set_enabled(True, 0.0)
        loop.start("task", [(1, 0, 1)], 0.0)

        events = loop.tick(2.0, _odom(0, 0, 1, 0.0, 2.0))
        self.assertEqual(events[0]["state"], "TIMEOUT")
        self.assertEqual(events[0]["reason"], "waypoint timeout")

    def test_high_speed_prevents_arrival(self):
        self.loop.set_enabled(True, 0.0)
        self.loop.start("task", [(1, 0, 1)], 0.0)
        self.loop.tick(1.0, _odom(1, 0, 1, 0.5, 1.0))
        self.assertIsNone(self.loop.arrival_since)

    def test_stale_odometry_fails_closed(self):
        self.loop.set_enabled(True, 0.0)
        self.loop.start("task", [(1, 0, 1)], 0.0)
        events = self.loop.tick(2.0, _odom(0, 0, 1, 0.0, 0.0))
        self.assertEqual(events[0]["state"], "ERROR")

    def test_disable_aborts_active_path_and_clears_gate(self):
        self.loop.set_enabled(True, 0.0)
        self.loop.start("task", [(1, 0, 1)], 0.0)
        self.loop.set_enabled(False, 1.0)
        self.assertFalse(self.loop.enabled)
        self.assertEqual(self.loop.state, "DISABLED")

    def test_yaw_must_settle_before_waypoint_success(self):
        self.loop.set_enabled(True, 0.0)
        events = self.loop.start("task", [(1, 0, 1)], 0.0, waypoint_yaws=[1.0])
        self.assertEqual(events[0]["yaw"], 1.0)
        self.assertEqual(self.loop.tick(0.2, _odom(1, 0, 1, 0.1, 0.2, yaw=0.0)), [])
        self.assertIsNone(self.loop.arrival_since)
        self.loop.tick(0.4, _odom(1, 0, 1, 0.1, 0.4, yaw=1.0))
        events = self.loop.tick(1.0, _odom(1, 0, 1, 0.1, 1.0, yaw=1.0))
        self.assertEqual(events[0]["state"], "SUCCESS")

    def test_missing_yaw_fails_closed_when_goal_requires_yaw(self):
        self.loop.set_enabled(True, 0.0)
        self.loop.start("task", [(1, 0, 1)], 0.0, waypoint_yaws=[1.0])
        events = self.loop.tick(0.2, _odom(1, 0, 1, 0.1, 0.2))
        self.assertEqual(events[0]["state"], "ERROR")


if __name__ == "__main__":
    unittest.main()

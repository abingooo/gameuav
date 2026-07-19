import math
import unittest

from strategy.smpf.runtime.yaw_policy import target_facing_yaws


class SmpfYawPolicyTest(unittest.TestCase):
    def test_each_waypoint_faces_world_target(self):
        yaws = target_facing_yaws(
            [(0.0, 1.0, 0.0), (1.0, -1.0, 0.0)],
            (2.0, 0.0, 0.0),
        )
        self.assertAlmostEqual(yaws[0], math.atan2(-1.0, 2.0))
        self.assertAlmostEqual(yaws[1], math.atan2(1.0, 1.0))

    def test_vertical_target_uses_fallback_yaw(self):
        yaws = target_facing_yaws([(1.0, 2.0, 0.0)], (1.0, 2.0, 3.0), fallback_yaw=0.7)
        self.assertEqual(yaws, (0.7,))


if __name__ == "__main__":
    unittest.main()

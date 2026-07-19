import unittest

from strategy.smpf.runtime.contracts import ObjectSphere
from strategy.smpf.runtime.obstacle_relevance import assess_corridor_obstacles


class SmpfObstacleRelevanceTest(unittest.TestCase):
    def test_intersecting_obstacle_is_relevant(self):
        obstacle = ObjectSphere("cone", (2.0, 0.1, 0.0), 0.4)
        assessment = assess_corridor_obstacles(
            (0, 0, 0),
            (3, 0, 0),
            (obstacle,),
            corridor_margin_m=0.25,
        )[0]
        self.assertTrue(assessment.relevant)
        self.assertLess(assessment.centerline_clearance_m, 0.0)

    def test_off_corridor_obstacle_is_rejected(self):
        obstacle = ObjectSphere("chair", (2.0, 2.0, 0.0), 0.4)
        assessment = assess_corridor_obstacles(
            (0, 0, 0),
            (3, 0, 0),
            (obstacle,),
            corridor_margin_m=0.25,
        )[0]
        self.assertFalse(assessment.relevant)
        self.assertGreater(assessment.centerline_clearance_m, 0.25)

    def test_obstacle_behind_approach_goal_is_rejected(self):
        obstacle = ObjectSphere("laptop", (4.3, 0.0, 0.0), 0.5)
        assessment = assess_corridor_obstacles(
            (0, 0, 0),
            (3.0, 0, 0),
            (obstacle,),
            corridor_margin_m=0.25,
        )[0]
        self.assertFalse(assessment.relevant)
        self.assertAlmostEqual(assessment.centerline_clearance_m, 0.8)


if __name__ == "__main__":
    unittest.main()

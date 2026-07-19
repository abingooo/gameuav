import unittest

from strategy.smpf.runtime.contracts import ObjectSphere
from strategy.smpf.runtime.deterministic_planner import (
    VisibilityGraphError,
    approach_goal_candidates_for_sphere,
    approach_goal_for_sphere,
    plan_visibility_graph,
)


BOUNDS = {
    "x_min": -1.0,
    "x_max": 6.0,
    "y_min": -2.0,
    "y_max": 2.0,
    "z_min": -1.0,
    "z_max": 2.0,
}


class SmpfDeterministicPlannerTest(unittest.TestCase):
    def test_direct_path_is_normalized_to_three_guidepoints(self):
        plan = plan_visibility_graph((0, 0, 0), (3, 0, 0), bounds=BOUNDS)
        self.assertEqual(len(plan.guidepoints_m), 3)
        self.assertTrue(plan.validation.valid)

    def test_single_blocking_sphere_gets_verified_detour(self):
        obstacle = ObjectSphere("stool", (2, 0, 0), 0.6)
        plan = plan_visibility_graph(
            (0, 0, 0),
            (4, 0, 0),
            (obstacle,),
            bounds=BOUNDS,
            clearance_margin_m=0.1,
        )
        self.assertTrue(plan.validation.valid)
        self.assertGreater(plan.path_length_m, 4.0)
        self.assertGreater(plan.validation.minimum_clearance_m, 0.0)

    def test_multiple_spheres_remain_collision_free(self):
        obstacles = (
            ObjectSphere("chair", (1.5, 0.0, 0.0), 0.45),
            ObjectSphere("table", (3.0, 0.2, 0.0), 0.55),
        )
        plan = plan_visibility_graph(
            (0, 0, 0),
            (4, 0, 0),
            obstacles,
            bounds=BOUNDS,
            clearance_margin_m=0.1,
        )
        self.assertTrue(plan.validation.valid)
        self.assertGreaterEqual(len(plan.guidepoints_m), 3)

    def test_blocked_bounded_corridor_fails_closed(self):
        corridor = {
            "x_min": 0.0,
            "x_max": 4.0,
            "y_min": -0.1,
            "y_max": 0.1,
            "z_min": -0.1,
            "z_max": 0.1,
        }
        with self.assertRaises(VisibilityGraphError):
            plan_visibility_graph(
                (0, 0, 0),
                (4, 0, 0),
                (ObjectSphere("wall", (2, 0, 0), 0.6),),
                bounds=corridor,
                clearance_margin_m=0.1,
            )

    def test_target_approach_goal_stays_outside_safety_sphere(self):
        target = ObjectSphere("chair", (2.0, 0.0, 0.0), 0.5)
        goal = approach_goal_for_sphere(target, clearance_margin_m=0.1, standoff_m=0.2)
        self.assertAlmostEqual(goal[0], 1.2)

    def test_target_approach_candidates_respect_low_flight_bound(self):
        target = ObjectSphere("stool", (2.0, 0.0, 0.15), 0.6)
        candidates = approach_goal_candidates_for_sphere(
            target,
            clearance_margin_m=0.05,
            standoff_m=0.15,
            bounds={"z_min": 0.1, "z_max": 2.0},
        )
        self.assertGreater(len(candidates), 1)
        self.assertTrue(all(candidate[2] >= 0.1 for candidate in candidates))

    def test_follow_candidate_generation_can_move_away_when_too_close(self):
        target = ObjectSphere("person", (0.5, 0.0, 0.0), 0.2)
        candidates = approach_goal_candidates_for_sphere(
            target,
            standoff_m=0.8,
            allow_inside=True,
        )
        self.assertAlmostEqual(candidates[0][0], -0.5)


if __name__ == "__main__":
    unittest.main()

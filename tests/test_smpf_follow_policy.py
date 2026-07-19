import unittest

from strategy.smpf.runtime.contracts import ObjectSphere
from strategy.smpf.runtime.follow_policy import (
    FOLLOW_CONTINUE,
    FOLLOW_SUCCESS,
    FOLLOW_TIMEOUT,
    FOLLOW_UNSAFE,
    evaluate_follow_surface_standoff,
    next_follow_observation_is_final,
    select_follow_goal,
    validate_follow_goal_point,
)


class SmpfFollowPolicyTest(unittest.TestCase):
    def test_default_goal_is_point_fifteen_centimeters_outside_target_sphere(self):
        target = ObjectSphere("person", (3.0, 0.0, 0.0), 0.85)
        goal = select_follow_goal(target, (target,))
        self.assertAlmostEqual(goal.goal[0], 2.0)
        self.assertEqual(goal.goal, goal.desired_standoff_goal)
        self.assertAlmostEqual(goal.distance_m, 2.0)
        self.assertAlmostEqual(goal.target_center_distance_m, 1.0)
        self.assertAlmostEqual(goal.target_surface_distance_m, 0.15)
        self.assertAlmostEqual(goal.requested_surface_standoff_m, 0.15)
        self.assertFalse(goal.safety_limited)
        self.assertTrue(goal.target_visible)
        self.assertFalse(goal.clipped)

    def test_point_safety_margin_overrides_an_unsafe_surface_request(self):
        target = ObjectSphere("person", (3.0, 0.0, 0.0), 0.85)
        goal = select_follow_goal(
            target,
            (target,),
            clearance_margin_m=0.05,
            safety_padding_m=0.01,
            surface_standoff_m=0.03,
        )
        self.assertAlmostEqual(goal.target_surface_distance_m, 0.06)
        self.assertTrue(goal.safety_limited)

    def test_goal_selector_can_move_away_when_target_is_too_close(self):
        target = ObjectSphere("person", (0.2, 0.0, 0.0), 0.2)
        goal = select_follow_goal(target, (target,))
        self.assertAlmostEqual(goal.goal[0], -0.15)
        self.assertAlmostEqual(goal.target_surface_distance_m, 0.15)

    def test_goal_validation_checks_point_not_straight_line(self):
        obstacle = ObjectSphere("chair", (0.25, 0.0, 0.0), 0.10)
        validation = validate_follow_goal_point(
            (0.5, 0.0, 0.0),
            (obstacle,),
            bounds={"x_min": -1.0, "x_max": 1.0},
        )
        self.assertTrue(validation.valid)

    def test_goal_validation_rejects_occupied_goal(self):
        obstacle = ObjectSphere("chair", (0.5, 0.0, 0.0), 0.10)
        validation = validate_follow_goal_point((0.5, 0.0, 0.0), (obstacle,))
        self.assertFalse(validation.valid)

    def test_selector_uses_an_alternate_free_standoff_point(self):
        target = ObjectSphere("person", (3.0, 0.0, 0.0), 0.85)
        blocker = ObjectSphere("table", (2.0, 0.0, 0.0), 0.25)
        goal = select_follow_goal(
            target,
            (target, blocker),
            bounds={
                "x_min": -1.0,
                "x_max": 5.0,
                "y_min": -3.0,
                "y_max": 3.0,
                "z_min": -1.0,
                "z_max": 3.0,
            },
            clearance_margin_m=0.05,
        )
        self.assertNotEqual(goal.candidate_index, 0)
        self.assertEqual(goal.goal, goal.desired_standoff_goal)
        self.assertAlmostEqual(goal.target_surface_distance_m, 0.15)
        self.assertTrue(
            validate_follow_goal_point(
                goal.goal,
                (target, blocker),
                clearance_margin_m=0.05,
            ).valid
        )

    def test_selector_rejects_a_point_whose_target_view_is_blocked(self):
        target = ObjectSphere("person", (3.0, 0.0, 0.0), 0.85)
        blocker = ObjectSphere("panel", (2.08, 0.0, 0.0), 0.01)
        goal = select_follow_goal(target, (target, blocker))
        self.assertNotEqual(goal.candidate_index, 0)
        self.assertTrue(goal.target_visible)

    def test_bounded_goal_remains_an_optional_ablation(self):
        target = ObjectSphere("person", (3.0, 0.0, 0.0), 0.85)
        goal = select_follow_goal(target, (target,), max_step_m=0.5)
        self.assertAlmostEqual(goal.distance_m, 0.5)
        self.assertTrue(goal.clipped)

    def test_surface_standoff_inside_tolerance_succeeds(self):
        self.assertEqual(
            evaluate_follow_surface_standoff(0.18, desired_standoff_m=0.15, tolerance_m=0.10),
            FOLLOW_SUCCESS,
        )

    def test_final_observation_outside_tolerance_times_out(self):
        self.assertTrue(next_follow_observation_is_final(10, 10))
        self.assertEqual(
            evaluate_follow_surface_standoff(
                0.5,
                desired_standoff_m=0.15,
                tolerance_m=0.10,
                final_observation=True,
            ),
            FOLLOW_TIMEOUT,
        )

    def test_nonfinal_distant_observation_continues(self):
        self.assertEqual(evaluate_follow_surface_standoff(0.5), FOLLOW_CONTINUE)

    def test_observation_inside_safety_distance_is_unsafe(self):
        self.assertEqual(
            evaluate_follow_surface_standoff(0.03, minimum_safe_surface_distance_m=0.05),
            FOLLOW_UNSAFE,
        )


if __name__ == "__main__":
    unittest.main()

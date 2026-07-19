import unittest

from strategy.smpf.runtime.contracts import ObjectSphere
from strategy.smpf.runtime.goal_validation import validate_goal_conditioned_polyline


class SmpfGoalValidationTest(unittest.TestCase):
    def setUp(self):
        self.target = ObjectSphere("chair", (4.0, 0.0, 0.0), 0.5)

    def validate(self, points, obstacles=()):
        return validate_goal_conditioned_polyline(
            points,
            (self.target,) + tuple(obstacles),
            target_sphere=self.target,
            clearance_margin_m=0.1,
            min_target_standoff_m=0.15,
            max_target_standoff_m=1.0,
            min_target_progress_m=0.1,
            require_target_visibility=True,
            require_origin_start=True,
        )

    def test_safe_path_that_stops_far_away_is_rejected(self):
        result = self.validate([(0, 0, 0), (0.5, 1.0, 0), (1.0, 1.0, 0)])
        self.assertFalse(result.validation.valid)
        self.assertTrue(any(issue.kind == "target_standoff" for issue in result.validation.issues))

    def test_safe_path_inside_terminal_standoff_is_accepted(self):
        result = self.validate([(0, 0, 0), (2.0, 1.0, 0), (3.2, 0.8, 0)])
        self.assertTrue(result.validation.valid, result.validation.issues)
        self.assertLess(result.target_surface_distance_m, 1.0)
        self.assertGreater(result.target_progress_m, 0.1)
        self.assertTrue(result.target_visible)

    def test_blocked_final_view_is_rejected(self):
        blocker = ObjectSphere("cone", (3.5, 0.5, 0.0), 0.25)
        result = self.validate(
            [(0, 0, 0), (2.0, 1.0, 0), (3.0, 1.0, 0)],
            obstacles=(blocker,),
        )
        self.assertFalse(result.validation.valid)
        self.assertTrue(any(issue.kind == "target_occluded" for issue in result.validation.issues))

    def test_endpoint_tangent_to_clearance_margin_is_rejected_as_too_close(self):
        result = self.validate([(0, 0, 0), (2.0, 1.0, 0), (3.4, 0.0, 0)])
        self.assertFalse(result.validation.valid)
        self.assertTrue(any(issue.kind == "target_too_close" for issue in result.validation.issues))


if __name__ == "__main__":
    unittest.main()

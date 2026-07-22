import json
import unittest
from unittest import mock

from strategy.smpf.runtime.contracts import ObjectSphere
from strategy.smpf.runtime.model_planner import (
    ModelPlannerClient,
    PlanningRequest,
    PlanSchemaError,
    PlanValidationError,
    build_planning_prompt,
    parse_and_validate_plan,
)
from strategy.smpf.runtime.deterministic_planner import (
    approach_goal_candidates_for_sphere,
    approach_goal_for_sphere,
)


def _content(points):
    return json.dumps(
        {
            "schema": "smpf.guidepoint_plan.v1",
            "frame": "body_flu",
            "guidepoints_m": points,
            "reasoning": "保持安全距离接近目标。",
        }
    )


class SmpfModelPlannerTest(unittest.TestCase):
    def setUp(self):
        self.obstacle = ObjectSphere("chair", (2.0, 0.0, 0.0), 0.5, frame_id="body_flu")
        self.request = PlanningRequest(
            "飞到椅子旁边",
            (self.obstacle,),
            bounds_flu_m={"x_min": -1.0, "x_max": 5.0, "y_min": -3.0, "y_max": 3.0, "z_min": -1.0, "z_max": 3.0},
            clearance_margin_m=0.1,
        )

    def test_runtime_default_uses_recommended_planning_model(self):
        with mock.patch.dict("os.environ", {}, clear=True):
            planner = ModelPlannerClient(api_key="test", base_url="http://localhost")
        self.assertEqual(planner.model_id, "gpt-5.2")
        self.assertEqual(planner.reasoning_effort, "low")

    def test_reasoning_effort_is_normalized_and_validated(self):
        planner = ModelPlannerClient(
            api_key="test",
            base_url="http://localhost",
            reasoning_effort=" MINIMAL ",
        )
        self.assertEqual(planner.reasoning_effort, "minimal")
        with self.assertRaises(ValueError):
            ModelPlannerClient(
                api_key="test",
                base_url="http://localhost",
                reasoning_effort="fast",
            )

    def test_completion_payload_declares_reasoning_effort(self):
        response = mock.Mock(status_code=200)
        response.json.return_value = {
            "choices": [{"message": {"content": '{"ok":true}'}}]
        }
        session = mock.Mock()
        session.post.return_value = response
        planner = ModelPlannerClient(
            api_key="test",
            base_url="http://localhost",
            reasoning_effort="low",
            session=session,
        )
        self.assertEqual(planner._complete("test prompt"), '{"ok":true}')
        payload = session.post.call_args.kwargs["json"]
        self.assertEqual(payload["model"], "gpt-5.2")
        self.assertEqual(payload["reasoning_effort"], "low")
        self.assertNotIn("temperature", payload)
        self.assertEqual(planner.raw_responses[0]["body"], response.json.return_value)

    def test_non_gpt5_completion_payload_keeps_temperature(self):
        response = mock.Mock(status_code=200)
        response.json.return_value = {
            "choices": [{"message": {"content": '{"ok":true}'}}]
        }
        session = mock.Mock()
        session.post.return_value = response
        planner = ModelPlannerClient(
            api_key="test",
            base_url="http://localhost",
            model_id="compatible-model",
            temperature=0.1,
            session=session,
        )
        planner._complete("test prompt")
        payload = session.post.call_args.kwargs["json"]
        self.assertEqual(payload["temperature"], 0.1)

    def test_kimi_k3_completion_uses_provider_default_temperature(self):
        response = mock.Mock(status_code=200)
        response.json.return_value = {
            "choices": [{"message": {"content": '{"ok":true}'}}]
        }
        session = mock.Mock()
        session.post.return_value = response
        planner = ModelPlannerClient(
            api_key="test",
            base_url="http://localhost",
            model_id="kimi-k3",
            temperature=0.1,
            session=session,
        )

        planner._complete("test prompt")

        payload = session.post.call_args.kwargs["json"]
        self.assertNotIn("temperature", payload)

    def test_valid_flu_detour_is_accepted(self):
        plan = parse_and_validate_plan(
            _content([[0.0, 0.0, 0.0], [1.0, 1.0, 0.2], [2.0, 1.0, 0.2]]),
            self.request,
        )
        self.assertEqual(plan.guidepoints_m[-1], (2.0, 1.0, 0.2))
        self.assertTrue(plan.validation.valid)

    def test_segment_collision_is_rejected(self):
        with self.assertRaises(PlanValidationError):
            parse_and_validate_plan(
                _content([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [3.0, 0.0, 0.0]]),
                self.request,
            )

    def test_wrong_frame_is_rejected(self):
        data = json.loads(_content([[0, 0, 0], [0, 1, 0], [1, 1, 0]]))
        data["frame"] = "camera_optical"
        with self.assertRaises(PlanSchemaError):
            parse_and_validate_plan(json.dumps(data), self.request)

    def test_markdown_wrapping_is_rejected(self):
        with self.assertRaises(PlanSchemaError):
            parse_and_validate_plan("```json\n%s\n```" % _content([[0, 0, 0], [0, 1, 0], [1, 1, 0]]), self.request)

    def test_prompt_defines_only_body_flu_axes(self):
        prompt = build_planning_prompt(self.request)
        self.assertIn("x forward, y left, z up", prompt)
        self.assertIn('"center_flu_m"', prompt)
        self.assertNotIn("y positive = downward", prompt)

    def test_target_contract_rejects_safe_but_incomplete_path(self):
        target = ObjectSphere("chair", (4.0, 0.0, 0.0), 0.5, frame_id="body_flu")
        request = PlanningRequest(
            "fly to the chair",
            (target,),
            target_sphere=target,
            max_target_standoff_m=1.0,
        )
        with self.assertRaises(PlanValidationError) as context:
            parse_and_validate_plan(
                _content([[0.0, 0.0, 0.0], [0.5, 1.0, 0.0], [1.0, 1.0, 0.0]]),
                request,
            )
        self.assertTrue(
            any(issue.kind == "target_standoff" for issue in context.exception.validation_result.issues)
        )

    def test_prompt_marks_target_role_and_terminal_contract(self):
        target = ObjectSphere("chair", (3.0, 0.0, 0.0), 0.5, frame_id="body_flu")
        request = PlanningRequest("fly to chair", (target,), target_sphere=target)
        prompt = build_planning_prompt(request)
        self.assertIn('"role":"target"', prompt)
        self.assertIn('"min_surface_standoff_m":0.15', prompt)
        self.assertIn('"max_surface_standoff_m":1.0', prompt)

    def test_repeated_colliding_model_paths_use_verified_graph_fallback(self):
        request = PlanningRequest(
            "fly beyond the obstacle",
            (ObjectSphere("stool", (2.0, 0.0, 0.0), 0.6, frame_id="body_flu"),),
            bounds_flu_m={
                "x_min": -1.0,
                "x_max": 5.0,
                "y_min": -2.0,
                "y_max": 2.0,
                "z_min": -1.0,
                "z_max": 2.0,
            },
            clearance_margin_m=0.1,
            fallback_goal_flu_m=(4.0, 0.0, 0.0),
        )
        invalid = _content([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [4.0, 0.0, 0.0]])
        planner = ModelPlannerClient(api_key="test", base_url="http://localhost")
        planner._complete = lambda _prompt: invalid
        plan = planner.plan(request, max_attempts=2)
        self.assertEqual(plan.planner_source, "visibility_graph_fallback")
        self.assertEqual(plan.attempts, 2)
        self.assertGreater(plan.graph_candidate_count, 0)
        self.assertTrue(plan.validation.valid)

    def test_llm_only_ablation_does_not_use_graph_fallback(self):
        request = PlanningRequest(
            "fly beyond the obstacle",
            (ObjectSphere("stool", (2.0, 0.0, 0.0), 0.6, frame_id="body_flu"),),
            fallback_goal_flu_m=(4.0, 0.0, 0.0),
        )
        invalid = _content([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [4.0, 0.0, 0.0]])
        planner = ModelPlannerClient(api_key="test", base_url="http://localhost")
        planner._complete = lambda _prompt: invalid
        with self.assertRaises(PlanValidationError):
            planner.plan(request, max_attempts=2, enable_deterministic_fallback=False)

    def test_graph_fallback_repairs_safe_but_incomplete_model_goal(self):
        target = ObjectSphere("chair", (4.0, 0.0, 0.0), 0.5, frame_id="body_flu")
        request = PlanningRequest(
            "fly to the chair",
            (target,),
            target_sphere=target,
            clearance_margin_m=0.1,
            max_target_standoff_m=1.0,
            fallback_goal_flu_m=approach_goal_for_sphere(
                target,
                clearance_margin_m=0.1,
                standoff_m=0.15,
            ),
        )
        incomplete = _content([[0.0, 0.0, 0.0], [0.5, 1.0, 0.0], [1.0, 1.0, 0.0]])
        planner = ModelPlannerClient(api_key="test", base_url="http://localhost")
        planner._complete = lambda _prompt: incomplete
        plan = planner.plan(request, max_attempts=2)
        self.assertEqual(plan.planner_source, "visibility_graph_fallback")
        self.assertLessEqual(plan.target_surface_distance_m, 1.0)
        self.assertTrue(plan.target_visible)

    def test_graph_fallback_tries_alternate_target_approach_candidates(self):
        target = ObjectSphere("stool", (3.0, 0.0, 0.3), 0.5, frame_id="body_flu")
        blocker = ObjectSphere("person", (2.3, 0.0, 0.2), 0.25, frame_id="body_flu")
        bounds = {
            "x_min": -1.0,
            "x_max": 5.0,
            "y_min": -2.0,
            "y_max": 2.0,
            "z_min": 0.1,
            "z_max": 2.0,
        }
        candidates = approach_goal_candidates_for_sphere(
            target,
            clearance_margin_m=0.1,
            standoff_m=0.15,
            bounds=bounds,
        )
        request = PlanningRequest(
            "fly to the stool",
            (target, blocker),
            bounds_flu_m=bounds,
            clearance_margin_m=0.1,
            target_sphere=target,
            fallback_goals_flu_m=candidates,
        )
        invalid = _content([[0.0, 0.0, 0.0], [2.3, 0.0, 0.2], [2.3, 0.0, 0.2]])
        planner = ModelPlannerClient(api_key="test", base_url="http://localhost")
        planner._complete = lambda _prompt: invalid
        plan = planner.plan(request, max_attempts=2)
        self.assertEqual(plan.planner_source, "visibility_graph_fallback")
        self.assertTrue(plan.fallback_trigger.startswith("target_approach_candidate_"))
        self.assertTrue(plan.validation.valid)


if __name__ == "__main__":
    unittest.main()

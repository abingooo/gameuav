import json
import unittest
from unittest import mock

from strategy.smpf.runtime.task_stages import (
    TaskStageClient,
    TaskStageSchemaError,
    parse_task_stages,
    task_stage_prompt,
)


class SmpfTaskStagesTest(unittest.TestCase):
    def test_runtime_default_uses_recommended_stage_model(self):
        with mock.patch.dict("os.environ", {}, clear=True):
            client = TaskStageClient(api_key="test", base_url="http://localhost")
        self.assertEqual(client.model_id, "gpt-5.2")
        self.assertEqual(client.reasoning_effort, "low")

    def test_stage_payload_declares_reasoning_effort(self):
        response = mock.Mock(status_code=200)
        response.json.return_value = {
            "choices": [
                {
                    "message": {
                        "content": json.dumps(
                            {
                                "schema": "smpf.task_stages.v1",
                                "stages": [
                                    {
                                        "instruction": "reach the chair",
                                        "completion": "reach_target",
                                    }
                                ],
                            }
                        )
                    }
                }
            ]
        }
        session = mock.Mock()
        session.post.return_value = response
        client = TaskStageClient(
            api_key="test",
            base_url="http://localhost",
            reasoning_effort="minimal",
            session=session,
        )
        stages = client.decompose("reach the chair")
        self.assertEqual(len(stages.stages), 1)
        payload = session.post.call_args.kwargs["json"]
        self.assertEqual(payload["reasoning_effort"], "minimal")

    def test_ordered_long_horizon_stages_are_parsed(self):
        result = parse_task_stages(
            json.dumps(
                {
                    "schema": "smpf.task_stages.v1",
                    "stages": [
                        {"instruction": "reach the first chair", "completion": "reach_target"},
                        {
                            "instruction": "reach a different chair from the previous stage",
                            "completion": "reach_target",
                        },
                    ],
                }
            )
        )
        self.assertEqual(len(result.stages), 2)
        self.assertIn("different chair", result.stages[1].instruction)

    def test_unknown_completion_rule_is_rejected(self):
        with self.assertRaises(TaskStageSchemaError):
            parse_task_stages(
                '{"schema":"smpf.task_stages.v1","stages":[{"instruction":"chair","completion":"model_decides"}]}'
            )

    def test_prompt_forbids_inventing_flight_actions(self):
        prompt = task_stage_prompt("fly to chairs and the next")
        self.assertIn("Do not add takeoff, landing, arming", prompt)
        self.assertIn("a different chair", prompt)


if __name__ == "__main__":
    unittest.main()

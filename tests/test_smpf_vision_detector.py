import json
import unittest
from unittest import mock

from strategy.smpf.runtime.vision_detector import (
    DetectionSchemaError,
    VisionDetectorClient,
    detection_prompt,
    parse_detection,
    parse_scene_detection,
    scene_detection_prompt,
)


class SmpfVisionDetectorTest(unittest.TestCase):
    def test_runtime_default_uses_recommended_visual_model(self):
        with mock.patch.dict("os.environ", {}, clear=True):
            client = VisionDetectorClient(api_key="test", base_url="http://localhost")
        self.assertEqual(client.model_id, "gemini-3.5-flash")

    def test_valid_detection_converts_to_pixels(self):
        detection = parse_detection(
            json.dumps(
                {
                    "schema": "smpf.detection.v1",
                    "label": "red chair",
                    "bbox_yxyx_1000": [100, 200, 800, 900],
                    "confidence": 0.9,
                }
            )
        )
        self.assertEqual(detection.pixel_bbox((480, 640, 3)), (48, 128, 383, 575))

    def test_explicit_empty_detection_is_supported(self):
        detection = parse_detection(
            '{"schema":"smpf.detection.v1","label":"","bbox_yxyx_1000":[],"confidence":0.0}'
        )
        self.assertIsNone(detection)

    def test_invalid_empty_detection_is_rejected(self):
        with self.assertRaises(DetectionSchemaError):
            parse_detection(
                '{"schema":"smpf.detection.v1","label":"chair","bbox_yxyx_1000":[],"confidence":0.8}'
            )

    def test_normalized_bbox_requires_integers(self):
        with self.assertRaises(DetectionSchemaError):
            parse_detection(
                '{"schema":"smpf.detection.v1","label":"chair","bbox_yxyx_1000":[1.2,2,3,4],"confidence":0.8}'
            )

    def test_prompt_forbids_offscreen_guessing(self):
        prompt = detection_prompt("飞到红色椅子旁边")
        self.assertIn("Do not infer an off-screen location", prompt)
        self.assertIn("[ymin,xmin,ymax,xmax]", prompt)

    def test_scene_detection_contains_target_and_explicit_obstacle(self):
        scene = parse_scene_detection(
            json.dumps(
                {
                    "schema": "smpf.scene_detection.v1",
                    "target": {
                        "label": "person",
                        "bbox_yxyx_1000": [100, 200, 900, 600],
                        "confidence": 0.9,
                    },
                    "obstacles": [
                        {
                            "label": "cone",
                            "bbox_yxyx_1000": [500, 600, 900, 800],
                            "confidence": 0.8,
                        }
                    ],
                }
            )
        )
        self.assertEqual(scene.target.label, "person")
        self.assertEqual(len(scene.obstacles), 1)
        self.assertEqual(scene.obstacles[0].label, "cone")

    def test_scene_detection_supports_missing_target_during_search(self):
        scene = parse_scene_detection(
            '{"schema":"smpf.scene_detection.v1","target":{"label":"","bbox_yxyx_1000":[],"confidence":0.0},"obstacles":[]}'
        )
        self.assertIsNone(scene.target)

    def test_scene_prompt_requests_named_or_corridor_obstacles(self):
        prompt = scene_detection_prompt("fly to person without hitting cone")
        self.assertIn("explicitly named", prompt)
        self.assertIn("direct approach corridor", prompt)
        self.assertIn("clearly outside", prompt)

    def test_schema_failure_retries_without_relaxing_parser(self):
        client = VisionDetectorClient(
            api_key="test",
            base_url="http://localhost",
            max_attempts=2,
        )
        outputs = iter(
            (
                "not json",
                '{"schema":"smpf.scene_detection.v1","target":{"label":"chair","bbox_yxyx_1000":[100,200,800,700],"confidence":0.8},"obstacles":[]}',
            )
        )
        prompts = []

        def complete(_image, prompt):
            prompts.append(prompt)
            return next(outputs)

        client._complete = complete
        scene = client.detect_scene([[[0, 0, 0]]], "fly to chair")
        self.assertEqual(scene.target.label, "chair")
        self.assertEqual(client.last_attempts, 2)
        self.assertIn("rejected by the strict JSON parser", prompts[1])

    def test_schema_retry_exhaustion_still_fails_closed(self):
        client = VisionDetectorClient(
            api_key="test",
            base_url="http://localhost",
            max_attempts=2,
        )
        client._complete = lambda _image, _prompt: "not json"
        with self.assertRaises(DetectionSchemaError):
            client.detect_scene([[[0, 0, 0]]], "fly to chair")
        self.assertEqual(client.last_attempts, 2)


if __name__ == "__main__":
    unittest.main()

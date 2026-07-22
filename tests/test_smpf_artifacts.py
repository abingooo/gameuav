import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from strategy.smpf.runtime.artifacts import SmpfArtifactWriter, colorize_depth_image


class SmpfArtifactsTest(unittest.TestCase):
    def test_summary_tracks_plan_artifacts_and_terminal_outcome(self):
        with tempfile.TemporaryDirectory() as directory:
            writer = SmpfArtifactWriter(directory)
            writer.record_event(
                {
                    "schema": "gameuav.experiment.event.v1",
                    "timestamp": 10.0,
                    "method": "smpf",
                    "event": "task_received",
                    "task_id": "task-1",
                    "mode": "navigate",
                    "instruction": "reach the chair",
                    "models": {"llm": "gpt-5.2", "vlm": "gemini-3.5-flash"},
                    "execution_gate_open": True,
                }
            )
            writer.write_response(
                "task-1",
                "planning_llm",
                {
                    "status_code": 200,
                    "body": {"id": "response-1", "api_key": "must-not-leak"},
                },
            )
            image = np.zeros((40, 60, 3), dtype=np.uint8)
            annotated, image_path = writer.write_annotated_image(
                "task-1",
                "vlm_sam_geometry",
                image,
                [
                    {
                        "label": "target",
                        "bbox_yxyx": (5, 10, 30, 45),
                        "centroid_uv": (25, 18),
                    }
                ],
            )
            depth = np.arange(40 * 60, dtype=np.uint16).reshape(40, 60) + 1
            depth_visualization, _depth_image_path, depth_raw_path = writer.write_depth_image(
                "task-1",
                "aligned",
                depth,
                [],
            )
            geometry_path = writer.write_geometry(
                "task-1",
                "sphere_models",
                {
                    "schema": "gameuav.smpf.sphere_models.v1",
                    "objects": [{"label": "chair", "safety_radius_m": 0.5}],
                },
            )
            writer.record_event(
                {
                    "schema": "gameuav.experiment.event.v1",
                    "timestamp": 12.0,
                    "method": "smpf",
                    "event": "plan_verified",
                    "task_id": "task-1",
                    "latency_sec": {"cycle_total": 2.0},
                    "path_length_m": 1.5,
                }
            )
            writer.record_event(
                {
                    "schema": "gameuav.experiment.event.v1",
                    "timestamp": 15.0,
                    "method": "smpf",
                    "event": "terminal",
                    "task_id": "task-1",
                    "state": "SUCCESS",
                    "success": True,
                    "reason": "arrived",
                }
            )
            task_dir = Path(directory) / "task-1"
            summary = json.loads((task_dir / "summary.json").read_text(encoding="utf-8"))
            response = json.loads(
                next((task_dir / "responses").glob("*.json")).read_text(encoding="utf-8")
            )
            image_exists = image_path.is_file()
            saved_image_ok = cv2.imread(str(image_path)) is not None
            saved_depth = np.load(str(depth_raw_path), allow_pickle=False)
            geometry = json.loads(geometry_path.read_text(encoding="utf-8"))
        self.assertEqual(summary["elapsed_sec"], 5.0)
        self.assertEqual(summary["plan"]["latency_sec"]["cycle_total"], 2.0)
        self.assertEqual(summary["outcome"]["state"], "SUCCESS")
        self.assertEqual(len(summary["artifacts"]["images"]), 2)
        self.assertEqual(len(summary["artifacts"]["depth"]), 1)
        self.assertEqual(len(summary["artifacts"]["geometry"]), 1)
        self.assertEqual(len(summary["artifacts"]["responses"]), 1)
        self.assertEqual(response["response"]["body"]["api_key"], "<redacted>")
        self.assertTrue(image_exists)
        self.assertGreater(int(np.count_nonzero(annotated)), 0)
        self.assertTrue(saved_image_ok)
        self.assertEqual(depth_visualization.shape, (40, 60, 3))
        self.assertTrue(np.array_equal(saved_depth, depth))
        self.assertEqual(geometry["objects"][0]["label"], "chair")

    def test_depth_colorization_marks_invalid_pixels_black(self):
        depth = np.asarray([[0.0, 1.0], [2.0, np.nan]], dtype=np.float32)
        colorized = colorize_depth_image(depth)
        self.assertEqual(colorized.shape, (2, 2, 3))
        self.assertTrue(np.array_equal(colorized[0, 0], (0, 0, 0)))
        self.assertTrue(np.array_equal(colorized[1, 1], (0, 0, 0)))
        self.assertGreater(int(np.count_nonzero(colorized[0, 1])), 0)


if __name__ == "__main__":
    unittest.main()

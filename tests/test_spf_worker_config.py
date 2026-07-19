import importlib.util
import json
import os
import tempfile
from types import SimpleNamespace
import unittest
from pathlib import Path
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "strategy/see_point_fly/worker/spf_worker.py"
SPEC = importlib.util.spec_from_file_location("spf_worker", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class SpfWorkerConfigTest(unittest.TestCase):
    def write_config(self, text):
        handle = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
        self.addCleanup(lambda: os.unlink(handle.name))
        with handle:
            handle.write(text)
        return handle.name

    def test_reads_author_operational_mode_from_config(self):
        path = self.write_config("operational_mode: obstacle_mode\n")
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assertEqual(MODULE._effective_operational_mode(path), "obstacle_mode")

    def test_environment_override_takes_precedence(self):
        path = self.write_config("operational_mode: adaptive_mode\n")
        with mock.patch.dict(os.environ, {"SPF_OPERATIONAL_MODE": "obstacle_mode"}, clear=True):
            self.assertEqual(MODULE._effective_operational_mode(path), "obstacle_mode")

    def test_rejects_unknown_operational_mode(self):
        path = self.write_config("operational_mode: invented_mode\n")
        with mock.patch.dict(os.environ, {}, clear=True):
            with self.assertRaises(MODULE.WorkerError):
                MODULE._effective_operational_mode(path)

    def test_health_reports_effective_mode_and_config(self):
        path = self.write_config(
            "api_provider: openai\n"
            "model_name: gemini-test-flash\n"
            "operational_mode: obstacle_mode\n"
        )
        with mock.patch.dict(os.environ, {"SPF_CONFIG_PATH": path}, clear=True):
            health = MODULE._health_payload()
        self.assertEqual(health["spf_config_path"], path)
        self.assertEqual(health["operational_mode"], "obstacle_mode")
        self.assertEqual(health["api_provider"], "openai")
        self.assertEqual(health["model"], "gemini-test-flash")
        self.assertEqual(health["model_source"], "configured_override")

    def test_all_published_task_prompts_reach_author_projector_unchanged(self):
        prompts = []
        for filename in ("spf_realworld_tasks.json", "spf_simulation_tasks.json"):
            manifest = json.loads(
                (
                    ROOT
                    / "strategy/smpf/experiments"
                    / filename
                ).read_text(encoding="utf-8")
            )
            prompts.extend(task["prompt"] for task in manifest["tasks"])

        seen = []

        class FakeImage:
            shape = (480, 640, 3)

        class FakeProjector:
            api_provider = "openai"
            model_name = "gemini-test-flash"
            operational_mode = "adaptive_mode"

            def get_vlm_points(self, image, instruction):
                self.image = image
                seen.append(instruction)
                return [
                    SimpleNamespace(
                        dx=0.0,
                        dy=1.0,
                        dz=0.0,
                        yaw_only=False,
                        screen_x=320.0,
                        screen_y=240.0,
                    )
                ]

        projector = FakeProjector()
        with mock.patch.object(MODULE, "_decode_image", return_value=FakeImage()), mock.patch.object(
            MODULE, "_load_projector", return_value=projector
        ), mock.patch.object(MODULE, "_spf_logging_enabled", return_value=False):
            for prompt in prompts:
                result = MODULE.infer_action(
                    {"command": prompt, "image_jpeg_b64": "test-image"}
                )
                self.assertEqual(result["dy"], 1.0)
                self.assertEqual(result["model"], "gemini-test-flash")
                self.assertEqual(result["operational_mode"], "adaptive_mode")

        self.assertEqual(seen, prompts)


if __name__ == "__main__":
    unittest.main()

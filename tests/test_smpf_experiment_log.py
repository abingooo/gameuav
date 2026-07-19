import json
from pathlib import Path
import tempfile
import unittest

from strategy.smpf.runtime.experiment_log import JsonlExperimentLogger


class SmpfExperimentLogTest(unittest.TestCase):
    def test_logger_appends_parseable_events_and_redacts_credentials(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "events.jsonl"
            logger = JsonlExperimentLogger(path)
            logger.log(
                "plan_verified",
                "task-1",
                latency_sec=1.2,
                nested={"api_key": "do-not-write", "value": 3},
            )
            logger.log("terminal", "task-1", success=True)
            records = [json.loads(line) for line in path.read_text().splitlines()]
        self.assertEqual(len(records), 2)
        self.assertEqual(records[0]["schema"], "gameuav.experiment.event.v1")
        self.assertEqual(records[0]["nested"]["api_key"], "<redacted>")
        self.assertEqual(records[0]["nested"]["value"], 3)
        self.assertTrue(records[1]["success"])

    def test_non_finite_float_is_serialized_as_null(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "events.jsonl"
            JsonlExperimentLogger(path).log("metric", minimum_clearance=float("inf"))
            record = json.loads(path.read_text())
        self.assertIsNone(record["minimum_clearance"])


if __name__ == "__main__":
    unittest.main()

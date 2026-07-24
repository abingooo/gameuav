import importlib.util
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = (
    ROOT
    / "ros_nodes/state_estimation/VINS-Fusion/vins_estimator/scripts/vins_health_monitor.py"
)
LAUNCH = (
    ROOT
    / "ros_nodes/state_estimation/VINS-Fusion/vins_estimator/launch/fast_drone_250.launch"
)


def load_monitor_module():
    spec = importlib.util.spec_from_file_location("vins_health_monitor", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class VinsHealthMonitorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.monitor = load_monitor_module()

    def test_numeric_helpers_reject_invalid_vectors(self):
        self.assertEqual(self.monitor.norm3([3.0, 4.0, 0.0]), 5.0)
        self.assertTrue(math.isnan(self.monitor.norm3([math.nan, 0.0, 0.0])))
        self.assertIsNone(self.monitor.vector_difference([1.0, 2.0], [1.0, 2.0]))

    def test_vector_difference(self):
        self.assertEqual(
            self.monitor.vector_difference([4.0, 5.0, 6.0], [1.0, 2.0, 3.0]),
            [3.0, 3.0, 3.0],
        )

    def test_csv_fields_are_unique(self):
        self.assertEqual(len(self.monitor.CSV_FIELDS), len(set(self.monitor.CSV_FIELDS)))
        self.assertIn("aligned_error_m", self.monitor.CSV_FIELDS)
        self.assertIn("feature_count", self.monitor.CSV_FIELDS)

    def test_launch_starts_monitor_by_default(self):
        root = ET.parse(LAUNCH).getroot()
        args = {node.attrib["name"]: node.attrib for node in root.findall("arg")}
        nodes = {node.attrib["name"]: node.attrib for node in root.findall("node")}
        self.assertEqual(args["enable_health_monitor"]["default"], "true")
        self.assertEqual(nodes["vins_health_monitor"]["type"], "vins_health_monitor.py")


if __name__ == "__main__":
    unittest.main()

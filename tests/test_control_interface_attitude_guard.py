import importlib.util
import math
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = (
    ROOT
    / "ros_nodes/control/gameuav_control_interface/scripts/control_interface_node.py"
)
SPEC = importlib.util.spec_from_file_location("control_interface_node", MODULE_PATH)
CONTROL_INTERFACE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CONTROL_INTERFACE)


class Quaternion:
    def __init__(self, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        self.w = cr * cp * cy + sr * sp * sy
        self.x = sr * cp * cy - cr * sp * sy
        self.y = cr * sp * cy + sr * cp * sy
        self.z = cr * cp * sy - sr * sp * cy


class ControlInterfaceAttitudeGuardTest(unittest.TestCase):
    def test_ignores_world_yaw_offset(self):
        estimate = Quaternion(roll_deg=2.0, pitch_deg=-1.0, yaw_deg=0.0)
        reference = Quaternion(roll_deg=2.0, pitch_deg=-1.0, yaw_deg=75.0)
        roll_error, pitch_error = CONTROL_INTERFACE._roll_pitch_error_deg(
            estimate, reference
        )
        self.assertAlmostEqual(roll_error, 0.0, places=6)
        self.assertAlmostEqual(pitch_error, 0.0, places=6)

    def test_rejectable_error_matches_live_failure_shape(self):
        estimate = Quaternion(roll_deg=10.8, pitch_deg=-2.4)
        reference = Quaternion(roll_deg=-1.5, pitch_deg=0.0, yaw_deg=70.0)
        roll_error, pitch_error = CONTROL_INTERFACE._roll_pitch_error_deg(
            estimate, reference
        )
        self.assertGreater(roll_error, 5.0)
        self.assertLess(pitch_error, 5.0)

    def test_zero_norm_quaternion_is_invalid(self):
        invalid = Quaternion()
        invalid.x = invalid.y = invalid.z = invalid.w = 0.0
        self.assertIsNone(CONTROL_INTERFACE._roll_pitch_from_quaternion(invalid))


if __name__ == "__main__":
    unittest.main()

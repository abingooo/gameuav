import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def launch_root(relative_path):
    return ET.parse(str(ROOT / relative_path)).getroot()


def arg_default(root, name):
    element = root.find("./arg[@name='%s']" % name)
    if element is None:
        raise AssertionError("missing launch arg: %s" % name)
    return element.get("default")


class SpfDirectControlRoutingTest(unittest.TestCase):
    def test_spf_bridge_defaults_to_dedicated_direct_position_topic(self):
        bridge = launch_root(
            "ros_nodes/mission/see_point_fly_bridge/launch/see_point_fly_bridge.launch"
        )
        bringup = launch_root("launch/bringup_see_point_fly.launch")

        self.assertEqual(arg_default(bridge, "goal_topic"), "/control/spf_position")
        self.assertEqual(arg_default(bringup, "goal_topic"), "/control/spf_position")

    def test_control_interface_converts_spf_target_to_px4ctrl_command_topic(self):
        control = launch_root(
            "ros_nodes/control/gameuav_control_interface/launch/control_interface.launch"
        )
        flight = launch_root("launch/bringup_flight_control.launch")

        self.assertEqual(arg_default(control, "spf_position_topic"), "/control/spf_position")
        self.assertEqual(arg_default(control, "output_position_cmd_topic"), "/control/position_cmd")
        self.assertEqual(arg_default(control, "spf_position_timeout"), "0.0")

        px4ctrl_include = flight.find("./include[@file='$(dirname)/bringup_px4ctrl.launch']")
        self.assertIsNotNone(px4ctrl_include)
        cmd_arg = px4ctrl_include.find("./arg[@name='cmd_topic']")
        self.assertIsNotNone(cmd_arg)
        self.assertEqual(cmd_arg.get("value"), "$(arg control_output_position_cmd_topic)")
        self.assertEqual(
            arg_default(flight, "control_output_position_cmd_topic"),
            "/control/position_cmd",
        )


if __name__ == "__main__":
    unittest.main()

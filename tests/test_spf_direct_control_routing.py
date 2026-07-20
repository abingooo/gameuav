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


def node_param(root, node_name, param_name):
    node = root.find("./node[@name='%s']" % node_name)
    if node is None:
        raise AssertionError("missing launch node: %s" % node_name)
    param = node.find("./param[@name='%s']" % param_name)
    if param is None:
        raise AssertionError("missing node parameter: %s" % param_name)
    return param.get("value")


class SpfDirectControlRoutingTest(unittest.TestCase):
    def test_spf_bridge_defaults_to_dedicated_direct_position_topic(self):
        bridge = launch_root(
            "ros_nodes/mission/see_point_fly_bridge/launch/see_point_fly_bridge.launch"
        )
        bringup = launch_root("launch/bringup_see_point_fly.launch")

        self.assertEqual(arg_default(bridge, "goal_topic"), "/control/spf_position")
        self.assertEqual(arg_default(bringup, "goal_topic"), "/control/spf_position")
        self.assertEqual(arg_default(bridge, "goal_projection_enabled"), "false")
        self.assertEqual(arg_default(bringup, "goal_projection_enabled"), "false")
        self.assertEqual(arg_default(bridge, "enable_topic"), "/spf/enable")
        self.assertEqual(arg_default(bringup, "enable_topic"), "/spf/enable")

        bridge_source = (
            ROOT
            / "ros_nodes/mission/see_point_fly_bridge/scripts/see_point_fly_bridge.py"
        ).read_text(encoding="utf-8")
        self.assertIn(
            'rospy.get_param("~goal_projection_enabled", False)',
            bridge_source,
        )
        self.assertEqual(
            node_param(bridge, "see_point_fly_bridge", "require_armed_for_execution"),
            "true",
        )
        self.assertEqual(
            node_param(bridge, "see_point_fly_bridge", "manual_enable_required"),
            "true",
        )
        self.assertEqual(
            node_param(bridge, "see_point_fly_bridge", "enable_topic"),
            "$(arg enable_topic)",
        )
        self.assertEqual(
            node_param(bridge, "spf_task_executor", "enable_topic"),
            "$(arg enable_topic)",
        )
        self.assertEqual(
            node_param(bridge, "spf_task_executor", "require_armed_for_start"),
            "true",
        )
        self.assertEqual(
            node_param(bridge, "spf_task_executor", "allow_tabletop_start_disarmed"),
            "false",
        )
        self.assertEqual(
            node_param(bridge, "spf_task_executor", "arrival_settle_sec"),
            "0.5",
        )
        self.assertEqual(
            node_param(bridge, "spf_task_executor", "goal_tolerance_yaw_deg"),
            "10.0",
        )
    def test_control_interface_converts_spf_target_to_px4ctrl_command_topic(self):
        control = launch_root(
            "ros_nodes/control/gameuav_control_interface/launch/control_interface.launch"
        )
        flight = launch_root("launch/bringup_flight_control.launch")

        self.assertEqual(arg_default(control, "spf_position_topic"), "/control/spf_position")
        self.assertEqual(arg_default(control, "spf_enable_topic"), "/spf/enable")
        self.assertEqual(arg_default(control, "mavros_state_topic"), "/mavros/state")
        self.assertEqual(arg_default(control, "spf_mavros_state_timeout_sec"), "2.5")
        self.assertEqual(arg_default(control, "output_position_cmd_topic"), "/control/position_cmd")
        self.assertEqual(arg_default(control, "spf_position_timeout"), "0.0")
        arrival_defaults = {
            "spf_release_on_arrival": "true",
            "spf_arrival_tolerance_xy": "0.25",
            "spf_arrival_tolerance_z": "0.20",
            "spf_arrival_tolerance_yaw_deg": "10.0",
            "spf_arrival_max_speed": "0.25",
            "spf_arrival_settle_sec": "0.5",
        }
        for name, expected in arrival_defaults.items():
            self.assertEqual(arg_default(control, name), expected)
            self.assertEqual(
                node_param(control, "gameuav_control_interface", name),
                "$(arg %s)" % name,
            )
        self.assertEqual(
            node_param(control, "gameuav_control_interface", "spf_enable_topic"),
            "$(arg spf_enable_topic)",
        )
        self.assertEqual(
            node_param(control, "gameuav_control_interface", "mavros_state_topic"),
            "$(arg mavros_state_topic)",
        )

        px4ctrl_include = flight.find("./include[@file='$(dirname)/bringup_px4ctrl.launch']")
        self.assertIsNotNone(px4ctrl_include)
        cmd_arg = px4ctrl_include.find("./arg[@name='cmd_topic']")
        self.assertIsNotNone(cmd_arg)
        self.assertEqual(cmd_arg.get("value"), "$(arg control_output_position_cmd_topic)")
        self.assertEqual(
            arg_default(flight, "control_output_position_cmd_topic"),
            "/control/position_cmd",
        )
        realflight = launch_root("launch/bringup_realflight.launch")
        for name, expected in arrival_defaults.items():
            self.assertEqual(arg_default(flight, name), expected)
            self.assertEqual(arg_default(realflight, name), expected)

        control_include = flight.find("./group/include[@file='$(dirname)/bringup_control_interface.launch']")
        self.assertIsNotNone(control_include)
        flight_control_include = realflight.find("./include[@file='$(dirname)/bringup_flight_control.launch']")
        self.assertIsNotNone(flight_control_include)
        for name in arrival_defaults:
            self.assertEqual(
                control_include.find("./arg[@name='%s']" % name).get("value"),
                "$(arg %s)" % name,
            )
            self.assertEqual(
                flight_control_include.find("./arg[@name='%s']" % name).get("value"),
                "$(arg %s)" % name,
            )


if __name__ == "__main__":
    unittest.main()

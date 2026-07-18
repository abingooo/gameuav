import tempfile
import unittest
from pathlib import Path

import yaml

from gateway.message_adapter.adapter import (
    adapt_ros_dict_to_state,
    network_command_to_ros_command,
)
from gateway.topic_mapping.loader import get_inbound_mapping, get_outbound_mapping, load_topic_mapping


class GatewayMappingTest(unittest.TestCase):
    def test_load_mapping(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "topics.yaml"
            path.write_text(
                yaml.safe_dump(
                    {
                        "gateway_mapping": {
                            "inbound": {"set_goal": "/planning/goal"},
                            "outbound": {"state": ["/mavros/state"]},
                        }
                    }
                ),
                encoding="utf-8",
            )

            data = load_topic_mapping(str(path))
            self.assertEqual(get_inbound_mapping(data)["set_goal"], "/planning/goal")
            self.assertEqual(get_outbound_mapping(data)["state"], ["/mavros/state"])


class GatewayAdapterTest(unittest.TestCase):
    def test_odometry_to_state(self):
        payload = {
            "pose": {"pose": {"position": {"x": 1.0, "y": 2.0, "z": 3.0}}},
            "twist": {"twist": {"linear": {"x": 0.1, "y": 0.2, "z": 0.3}}},
        }

        state = adapt_ros_dict_to_state("uav1", "/vins_fusion/imu_propagate", "nav_msgs/Odometry", payload)

        self.assertEqual(state["uav_id"], "uav1")
        self.assertEqual(state["position"], [1.0, 2.0, 3.0])
        self.assertEqual(state["velocity"], [0.1, 0.2, 0.3])

    def test_battery_to_state(self):
        state = adapt_ros_dict_to_state(
            "uav1",
            "/mavros/battery",
            "sensor_msgs/BatteryState",
            {"percentage": 0.7, "voltage": 15.8, "current": 1.2},
        )

        self.assertEqual(state["battery"]["percentage"], 0.7)
        self.assertEqual(state["battery"]["voltage"], 15.8)

    def test_mavros_state_to_state(self):
        state = adapt_ros_dict_to_state(
            "uav1",
            "/mavros/state",
            "mavros_msgs/State",
            {"connected": True, "armed": False, "guided": True, "mode": "OFFBOARD"},
        )

        self.assertTrue(state["mavros"]["connected"])
        self.assertFalse(state["mavros"]["armed"])
        self.assertEqual(state["mavros"]["mode"], "OFFBOARD")

    def test_network_set_goal_to_ros_command(self):
        command = network_command_to_ros_command(
            "set_goal",
            {"x": 1.0, "y": 2.0, "z": 1.5, "frame_id": "world", "yaw": 1.57079632679},
        )

        self.assertEqual(command["type"], "topic")
        self.assertEqual(command["topic"], "/planning/goal")
        self.assertEqual(command["payload"]["pose"]["position"]["z"], 1.5)
        self.assertAlmostEqual(command["payload"]["pose"]["orientation"]["z"], 0.70710678118, places=6)

    def test_network_ego_position_to_control_interface(self):
        command = network_command_to_ros_command(
            "ego-position",
            {"x": 0.5, "y": 0.0, "z": 0.0},
        )

        self.assertEqual(command["type"], "topic")
        self.assertEqual(command["topic"], "/control/ego_position")
        self.assertEqual(command["msg_type"], "geometry_msgs/PoseStamped")
        self.assertEqual(command["payload"]["header"]["frame_id"], "body")

    def test_network_position_to_control_interface(self):
        command = network_command_to_ros_command(
            "position",
            {"x": 1.0, "y": 2.0, "z": 0.8, "frame_id": "world"},
        )

        self.assertEqual(command["topic"], "/control/position")
        self.assertEqual(command["payload"]["pose"]["position"]["z"], 0.8)

    def test_network_speed_to_control_interface(self):
        command = network_command_to_ros_command(
            "speed",
            {"vx": 0.2, "vy": 0.0, "vz": -0.1, "yaw_rate": 0.3},
        )

        self.assertEqual(command["topic"], "/control/speed")
        self.assertEqual(command["msg_type"], "geometry_msgs/TwistStamped")
        self.assertEqual(command["payload"]["header"]["frame_id"], "body")
        self.assertEqual(command["payload"]["twist"]["linear"]["x"], 0.2)
        self.assertEqual(command["payload"]["twist"]["angular"]["z"], 0.3)

    def test_network_stop_to_control_interface(self):
        command = network_command_to_ros_command("stop", {})

        self.assertEqual(command["topic"], "/control/stop")
        self.assertEqual(command["msg_type"], "std_msgs/Empty")
        self.assertEqual(command["payload"], {})

    def test_network_tiplight_to_ros_command(self):
        command = network_command_to_ros_command("tiplight", {"data": "ready"})

        self.assertEqual(command["type"], "topic")
        self.assertEqual(command["topic"], "/actuation/tiplight_cmd")
        self.assertEqual(command["payload"], {"data": "ready"})

    def test_network_takeoff_to_agent_safe_command(self):
        command = network_command_to_ros_command("takeoff", {"dry_run": True})

        self.assertEqual(command["type"], "agent_ros_command")
        self.assertEqual(command["command"], "safe_takeoff")
        self.assertEqual(command["args"], {"dry_run": True})


if __name__ == "__main__":
    unittest.main()

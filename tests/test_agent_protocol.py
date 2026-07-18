import unittest

from comm.protocol.agent_protocol import (
    ProtocolError,
    decode_message,
    encode_message,
    make_module_command,
    make_ros_command,
)


class AgentProtocolTest(unittest.TestCase):
    def test_module_command_round_trip(self):
        message = make_module_command("start", "vins", source_id="gcs", target_id="uav1")

        decoded = decode_message(encode_message(message))

        self.assertEqual(decoded["message_type"], "module_command")
        self.assertEqual(decoded["payload"]["action"], "start")
        self.assertEqual(decoded["payload"]["module"], "vins")

    def test_checksum_mismatch_is_rejected(self):
        message = make_module_command("status", "mavros")
        message["payload"]["module"] = "ego"

        with self.assertRaises(ProtocolError):
            encode_message(message)

    def test_action_validation(self):
        with self.assertRaises(ProtocolError):
            make_module_command("shell", "vins")

    def test_module_required_except_list(self):
        with self.assertRaises(ProtocolError):
            make_module_command("start")

        message = make_module_command("list")
        self.assertNotIn("module", message["payload"])

    def test_health_does_not_require_module(self):
        message = make_module_command("health", auth_token="token")

        self.assertEqual(message["payload"]["action"], "health")
        self.assertEqual(message["payload"]["auth_token"], "token")
        self.assertNotIn("module", message["payload"])

    def test_ros_command_round_trip(self):
        message = make_ros_command(
            "set_goal",
            args={"x": "1", "y": "2", "z": "3"},
            auth_token="token",
        )

        decoded = decode_message(encode_message(message))

        self.assertEqual(decoded["message_type"], "ros_command")
        self.assertEqual(decoded["payload"]["command"], "set_goal")
        self.assertEqual(decoded["payload"]["auth_token"], "token")


if __name__ == "__main__":
    unittest.main()

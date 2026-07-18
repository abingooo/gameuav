import unittest

from agent.uav_agent.server import AgentState, ThreadedAgentServer
from comm.protocol.agent_protocol import decode_message, encode_message, make_module_command, make_ros_command


class FakeManager:
    ros_master_uri = "http://localhost:11311"

    def list_modules(self):
        return {"vins": {"type": "launch"}}

    def start(self, module, args=None):
        return {"module": module, "status": "running", "pid": 123, "args": args or {}}

    def stop(self, module):
        return {"module": module, "status": "exited", "pid": None}

    def restart(self, module, args=None):
        return self.start(module, args)

    def status(self, module=None):
        if module is None:
            return {"vins": {"module": "vins", "status": "running", "pid": 123}}
        return {"module": module, "status": "running", "pid": 123}

    def is_ros_master_reachable(self):
        return False


class FakeRosExecutor:
    ros_master_uri = "http://localhost:11311"

    def list_commands(self):
        return {"health": {"enabled": True, "type": "builtin"}}

    def execute(self, command, args=None):
        return {"ok": True, "command": command, "args": args or {}}


class DummyServer:
    handle_raw_message = ThreadedAgentServer.handle_raw_message
    _handle_module_command = ThreadedAgentServer._handle_module_command
    _handle_ros_command = ThreadedAgentServer._handle_ros_command
    _check_auth = ThreadedAgentServer._check_auth
    _health = ThreadedAgentServer._health

    def __init__(self):
        self.state = AgentState(
            manager=FakeManager(),
            ros_executor=FakeRosExecutor(),
            uav_id="uav1",
            auth_token="secret",
        )


def make_server():
    return DummyServer()


class UavAgentServerTest(unittest.TestCase):
    def test_handle_start_command(self):
        server = make_server()
        request = make_module_command(
            "start",
            "vins",
            args={"mode": "test"},
            source_id="gcs",
            target_id="uav1",
            auth_token="secret",
        )

        response = decode_message(encode_message(server.handle_raw_message(encode_message(request))))

        self.assertEqual(response["message_type"], "module_status")
        self.assertIs(response["payload"]["ok"], True)
        self.assertEqual(response["payload"]["status"]["status"], "running")

    def test_reject_wrong_target(self):
        server = make_server()
        request = make_module_command(
            "status", "vins", source_id="gcs", target_id="uav2", auth_token="secret"
        )

        response = decode_message(encode_message(server.handle_raw_message(encode_message(request))))

        self.assertEqual(response["message_type"], "error")
        self.assertIs(response["payload"]["ok"], False)

    def test_reject_invalid_auth_token(self):
        server = make_server()
        request = make_module_command(
            "status", "vins", source_id="gcs", target_id="uav1", auth_token="wrong"
        )

        response = decode_message(encode_message(server.handle_raw_message(encode_message(request))))

        self.assertEqual(response["message_type"], "error")
        self.assertEqual(response["payload"]["detail"], "invalid auth token")

    def test_handle_health_command(self):
        server = make_server()
        request = make_module_command("health", source_id="gcs", target_id="uav1", auth_token="secret")

        response = decode_message(encode_message(server.handle_raw_message(encode_message(request))))

        self.assertEqual(response["message_type"], "module_status")
        self.assertIs(response["payload"]["ok"], True)
        self.assertEqual(response["payload"]["action"], "health")
        self.assertIn("agent", response["payload"])
        self.assertIn("ros", response["payload"])
        self.assertIn("modules", response["payload"])

    def test_handle_ros_command(self):
        server = make_server()
        request = make_ros_command(
            "set_goal",
            args={"x": "1", "y": "2", "z": "3"},
            source_id="gcs",
            target_id="uav1",
            auth_token="secret",
        )

        response = decode_message(encode_message(server.handle_raw_message(encode_message(request))))

        self.assertEqual(response["message_type"], "ros_command_result")
        self.assertIs(response["payload"]["ok"], True)
        self.assertEqual(response["payload"]["command"], "set_goal")
        self.assertEqual(response["payload"]["args"]["x"], "1")


if __name__ == "__main__":
    unittest.main()

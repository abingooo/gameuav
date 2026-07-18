import socket
import threading
import time
import unittest

from comm.heartbeat.monitor import HeartbeatMonitor
from comm.protocol.network_protocol import (
    MESSAGE_TYPE_HEARTBEAT,
    MESSAGE_TYPE_STATE,
    NetworkProtocolError,
    decode_message,
    encode_message,
    make_command,
    make_heartbeat,
    make_state,
)
from comm.tcp_link.client import TcpCommandClient
from comm.tcp_link.server import TcpCommandServer
from comm.udp_link.link import UdpLink


class NetworkProtocolTest(unittest.TestCase):
    def test_encode_decode_state(self):
        message = make_state("uav1", state={"battery": 0.8})
        decoded = decode_message(encode_message(message))

        self.assertEqual(decoded["message_type"], MESSAGE_TYPE_STATE)
        self.assertEqual(decoded["payload"]["battery"], 0.8)

    def test_checksum_rejects_tampering(self):
        message = make_heartbeat("uav1")
        message["payload"]["status"] = "tampered"

        with self.assertRaises(NetworkProtocolError):
            encode_message(message)


class HeartbeatMonitorTest(unittest.TestCase):
    def test_online_offline(self):
        now = [100.0]
        monitor = HeartbeatMonitor(timeout_sec=3.0, now_func=lambda: now[0])

        monitor.update("uav1")
        self.assertTrue(monitor.is_online("uav1"))

        now[0] = 104.0
        self.assertFalse(monitor.is_online("uav1"))
        self.assertFalse(monitor.peers()["uav1"]["online"])


class UdpLinkTest(unittest.TestCase):
    def test_send_receive_heartbeat_and_state(self):
        receiver = UdpLink("127.0.0.1", 0)
        sender = UdpLink("127.0.0.1", 0)
        host, port = receiver.address

        try:
            sender.send_heartbeat("uav1", host, port, payload={"battery": 0.9})
            heartbeat, _address = receiver.recv()
            self.assertEqual(heartbeat["message_type"], MESSAGE_TYPE_HEARTBEAT)
            self.assertEqual(heartbeat["payload"]["battery"], 0.9)

            sender.send_state("uav1", host, port, state={"position": [1, 2, 3]})
            state, _address = receiver.recv()
            self.assertEqual(state["message_type"], MESSAGE_TYPE_STATE)
            self.assertEqual(state["payload"]["position"], [1, 2, 3])
        finally:
            sender.close()
            receiver.close()


class TcpCommandLinkTest(unittest.TestCase):
    def test_command_ack_result(self):
        def handler(command, args, message):
            return {"command": command, "args": args, "source_id": message["source_id"]}

        server = TcpCommandServer(("127.0.0.1", 0), "uav1", handler)
        host, port = server.server_address
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()

        try:
            client = TcpCommandClient(host, port, source_id="gcs", target_id="uav1", timeout=2.0)
            response = client.send_command("set_goal", {"x": 1})

            self.assertEqual(response["ack"]["message_type"], "ack")
            self.assertTrue(response["ack"]["payload"]["accepted"])
            self.assertEqual(response["result"]["message_type"], "result")
            self.assertEqual(response["result"]["payload"]["result"]["command"], "set_goal")
            self.assertEqual(response["result"]["payload"]["result"]["args"], {"x": 1})
        finally:
            server.shutdown()
            server.server_close()

    def test_probe_connection(self):
        server = TcpCommandServer(("127.0.0.1", 0), "uav1", lambda command, args, message: {})
        host, port = server.server_address
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()

        try:
            client = TcpCommandClient(host, port, source_id="gcs", target_id="uav1", timeout=2.0)
            result = client.probe_connection()

            self.assertEqual(result["host"], host)
            self.assertEqual(result["port"], port)
        finally:
            server.shutdown()
            server.server_close()


if __name__ == "__main__":
    unittest.main()

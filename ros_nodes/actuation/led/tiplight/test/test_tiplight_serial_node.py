#!/usr/bin/env python3
import importlib.util
import os
import sys
import types
import unittest
from unittest import mock


SCRIPT_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts",
    "tiplight_serial_node.py",
)


class FakeRospy(types.SimpleNamespace):
    def __init__(self):
        super().__init__()
        self.params = {}
        self.subscribers = []
        self.publishers = []
        self.warnings = []
        self.infos = []
        self.errors = []

    def get_param(self, name, default=None):
        return self.params.get(name, default)

    def Publisher(self, topic, msg_type, queue_size=10, latch=False):
        publisher = FakePublisher(topic, msg_type, queue_size, latch)
        self.publishers.append(publisher)
        return publisher

    def Subscriber(self, topic, msg_type, callback, queue_size=10):
        subscriber = FakeSubscriber(topic, msg_type, callback, queue_size)
        self.subscribers.append(subscriber)
        return subscriber

    def loginfo(self, *args):
        self.infos.append(args)

    def logwarn(self, *args):
        self.warnings.append(args)

    def logerr(self, *args):
        self.errors.append(args)

    def is_shutdown(self):
        return False


class FakePublisher:
    def __init__(self, topic, msg_type, queue_size, latch):
        self.topic = topic
        self.msg_type = msg_type
        self.queue_size = queue_size
        self.latch = latch
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeSubscriber:
    def __init__(self, topic, msg_type, callback, queue_size):
        self.topic = topic
        self.msg_type = msg_type
        self.callback = callback
        self.queue_size = queue_size


class FakeString:
    def __init__(self, data=""):
        self.data = data


class FakeSerialException(Exception):
    pass


class FakeSerialPort:
    instances = []
    attempts = []
    available_ports = set()

    def __init__(self, port, baudrate, timeout=0, write_timeout=1):
        FakeSerialPort.attempts.append(port)
        if port not in FakeSerialPort.available_ports:
            raise FakeSerialException(f"could not open port {port}")

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.write_timeout = write_timeout
        self.writes = []
        self.is_open = True
        self.in_waiting = 0
        FakeSerialPort.instances.append(self)

    def reset_input_buffer(self):
        pass

    def write(self, data):
        self.writes.append(data)

    def flush(self):
        pass

    def read(self, _size):
        return b""

    def close(self):
        self.is_open = False


def load_module(fake_rospy):
    std_msgs_module = types.ModuleType("std_msgs")
    std_msgs_msg_module = types.ModuleType("std_msgs.msg")
    std_msgs_msg_module.String = FakeString

    serial_module = types.ModuleType("serial")
    serial_module.Serial = FakeSerialPort
    serial_module.SerialException = FakeSerialException

    with mock.patch.dict(
        sys.modules,
        {
            "rospy": fake_rospy,
            "std_msgs": std_msgs_module,
            "std_msgs.msg": std_msgs_msg_module,
            "serial": serial_module,
        },
    ):
        spec = importlib.util.spec_from_file_location(
            "tiplight_serial_node_under_test",
            SCRIPT_PATH,
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module


class TiplightSerialNodeTest(unittest.TestCase):
    def setUp(self):
        FakeSerialPort.instances = []
        FakeSerialPort.attempts = []
        FakeSerialPort.available_ports = {"/dev/gameuav_tiplight"}
        self.fake_rospy = FakeRospy()
        self.module = load_module(self.fake_rospy)

    def test_default_topics_and_auto_serial_port(self):
        with mock.patch.object(self.module.os.path, "exists", return_value=True):
            node = self.module.TiplightSerialNode()

        self.assertEqual(node.command_topic, "/actuation/tiplight_cmd")
        self.assertEqual(node.status_topic, "/status/tiplight")
        self.assertEqual(node.port, "auto")
        self.assertEqual(FakeSerialPort.instances[0].port, "/dev/gameuav_tiplight")
        self.assertEqual(
            self.fake_rospy.subscribers[0].topic,
            "/actuation/tiplight_cmd",
        )
        self.assertEqual(self.fake_rospy.publishers[0].topic, "/status/tiplight")
        self.assertTrue(self.fake_rospy.publishers[0].latch)

    def test_sends_all_esp32_state_commands(self):
        with mock.patch.object(self.module.os.path, "exists", return_value=True):
            node = self.module.TiplightSerialNode()
        commands = {
            "default": b"1",
            "ready": b"2",
            "takeoff": b"3",
            "hover": b"4",
            "game": b"5",
            "defense": b"6",
            "enemy": b"7",
            "abort": b"8",
            "next": b"n",
            "prev": b"p",
            "status": b"?",
        }

        with mock.patch.object(node, "read_response", return_value=""):
            for text in commands:
                node.handle_message(FakeString(text))

        self.assertEqual(FakeSerialPort.instances[0].writes, list(commands.values()))

    def test_publishes_esp32_response(self):
        with mock.patch.object(self.module.os.path, "exists", return_value=True):
            node = self.module.TiplightSerialNode()

        with mock.patch.object(node, "read_response", return_value="state=4 name=hover"):
            node.handle_message(FakeString("hover"))

        self.assertEqual(
            self.fake_rospy.publishers[0].messages,
            ["state=4 name=hover"],
        )

    def test_rejects_unknown_command(self):
        with mock.patch.object(self.module.os.path, "exists", return_value=True):
            node = self.module.TiplightSerialNode()
        node.handle_message(FakeString("launch"))

        self.assertEqual(FakeSerialPort.instances[0].writes, [])
        self.assertEqual(len(self.fake_rospy.warnings), 1)

    def test_stays_alive_without_serial_device(self):
        FakeSerialPort.available_ports = set()

        with mock.patch.object(self.module.os.path, "exists", return_value=False), \
             mock.patch.object(self.module.glob, "glob", return_value=[]):
            node = self.module.TiplightSerialNode()
            node.handle_message(FakeString("hover"))

        self.assertIsNone(node.serial)
        self.assertEqual(FakeSerialPort.instances, [])
        self.assertEqual(FakeSerialPort.attempts, [])
        self.assertEqual(len(self.fake_rospy.subscribers), 1)

    def test_can_connect_after_device_appears(self):
        FakeSerialPort.available_ports = set()

        def exists(path):
            return path == "/dev/gameuav_tiplight" and FakeSerialPort.available_ports

        with mock.patch.object(self.module.os.path, "exists", side_effect=exists):
            node = self.module.TiplightSerialNode()
            FakeSerialPort.available_ports = {"/dev/gameuav_tiplight"}
            node.last_open_attempt_at = 0.0
            with mock.patch.object(node, "read_response", return_value=""):
                node.handle_message(FakeString("ready"))

        self.assertEqual(FakeSerialPort.instances[0].writes, [b"2"])


if __name__ == "__main__":
    unittest.main()

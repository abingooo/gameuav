#!/usr/bin/env python3
import glob
import os
import sys
import threading
import time

import rospy
from std_msgs.msg import String


def add_local_venv_site_packages():
    package_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    version = f"python{sys.version_info.major}.{sys.version_info.minor}"
    candidates = [
        os.path.join(package_root, ".venv", "lib", version, "site-packages"),
        os.path.join(
            package_root,
            "..",
            "..",
            "..",
            ".venv",
            "lib",
            version,
            "site-packages",
        ),
    ]

    for site_packages in candidates:
        site_packages = os.path.abspath(site_packages)
        if os.path.isdir(site_packages) and site_packages not in sys.path:
            sys.path.insert(0, site_packages)


add_local_venv_site_packages()
import serial


DEFAULT_PORT = "auto"
DEFAULT_COMMAND_TOPIC = "/actuation/tiplight_cmd"
DEFAULT_STATUS_TOPIC = "/status/tiplight"
PREFERRED_PORTS = [
    "/dev/gameuav_tiplight",
]
AUTO_PORT_PATTERNS = [
    "/dev/serial/by-id/*Espressif*",
    "/dev/serial/by-id/*esp32*",
    "/dev/serial/by-id/*USB_JTAG*",
]


COMMANDS = {
    "1": "1",
    "default": "1",
    "idle": "1",
    "2": "2",
    "ready": "2",
    "3": "3",
    "takeoff": "3",
    "take_off": "3",
    "4": "4",
    "hover": "4",
    "5": "5",
    "game": "5",
    "6": "6",
    "defense": "6",
    "defense_success": "6",
    "defence": "6",
    "defence_success": "6",
    "7": "7",
    "enemy": "7",
    "enemy_success": "7",
    "8": "8",
    "abort": "8",
    "aborted": "8",
    "error": "8",
    "n": "n",
    "next": "n",
    "p": "p",
    "prev": "p",
    "previous": "p",
    "?": "?",
    "status": "?",
    "help": "?",
}
VALID_COMMAND_TEXT = (
    "1/default, 2/ready, 3/takeoff, 4/hover, 5/game, "
    "6/defense, 7/enemy, 8/abort, next, prev, status"
)


class TiplightSerialNode:
    def __init__(self):
        self.port = rospy.get_param("~port", DEFAULT_PORT)
        self.baud = int(rospy.get_param("~baud", 115200))
        legacy_topic = rospy.get_param("~topic", None)
        self.command_topic = rospy.get_param(
            "~command_topic", legacy_topic or DEFAULT_COMMAND_TOPIC
        )
        self.status_topic = rospy.get_param("~status_topic", DEFAULT_STATUS_TOPIC)
        self.read_seconds = float(rospy.get_param("~read_seconds", 0.4))
        self.reconnect_seconds = float(rospy.get_param("~reconnect_seconds", 1.0))
        self.lock = threading.Lock()
        self.serial = None
        self.active_port = None
        self.last_open_attempt_at = 0.0

        self.status_publisher = rospy.Publisher(
            self.status_topic,
            String,
            queue_size=10,
            latch=True,
        )
        self.subscriber = rospy.Subscriber(
            self.command_topic,
            String,
            self.handle_message,
            queue_size=10,
        )
        self.open_serial(force=True)

        rospy.loginfo(
            "tiplight serial node ready: command_topic=%s status_topic=%s port=%s baud=%d",
            self.command_topic,
            self.status_topic,
            self.port,
            self.baud,
        )

    def auto_detect_port(self):
        for port in PREFERRED_PORTS:
            if os.path.exists(port):
                return port

        for pattern in AUTO_PORT_PATTERNS:
            matches = sorted(glob.glob(pattern))
            if matches:
                return matches[0]

        return None

    def resolve_port(self):
        if self.port.lower() == "auto":
            return self.auto_detect_port()
        return self.port

    def open_serial(self, force=False):
        if self.serial and self.serial.is_open:
            return True

        now = time.monotonic()
        if not force and now - self.last_open_attempt_at < self.reconnect_seconds:
            return False
        self.last_open_attempt_at = now

        port = self.resolve_port()
        if not port:
            rospy.logwarn(
                "tiplight serial device not found; waiting for %s or %s",
                ", ".join(PREFERRED_PORTS),
                ", ".join(AUTO_PORT_PATTERNS),
            )
            return False

        try:
            self.serial = serial.Serial(
                port,
                self.baud,
                timeout=0,
                write_timeout=1,
            )
            self.active_port = port
            self.serial.reset_input_buffer()
            rospy.loginfo("tiplight serial connected: port=%s baud=%d", port, self.baud)
            return True
        except serial.SerialException as exc:
            rospy.logwarn("failed to open tiplight serial port %s: %s", port, exc)
            self.serial = None
            self.active_port = None
            return False

    def handle_message(self, msg):
        value = msg.data.strip().lower()
        command = COMMANDS.get(value)

        if command is None:
            rospy.logwarn(
                "unsupported tiplight command '%s'; valid values: %s",
                msg.data,
                VALID_COMMAND_TEXT,
            )
            return

        with self.lock:
            if not self.open_serial():
                rospy.logwarn(
                    "tiplight command '%s' ignored because no serial device is connected",
                    msg.data,
                )
                return

            try:
                self.serial.write(command.encode("ascii"))
                self.serial.flush()
                response = self.read_response(self.read_seconds)
            except serial.SerialException as exc:
                rospy.logerr("serial write failed: %s", exc)
                self.close()
                return

        rospy.loginfo(
            "tiplight command '%s' sent as '%s' on %s",
            msg.data,
            command,
            self.active_port,
        )
        if response:
            rospy.loginfo("tiplight response: %s", response)
            self.status_publisher.publish(response)

    def read_response(self, seconds):
        deadline = time.monotonic() + seconds
        chunks = []

        while time.monotonic() < deadline and not rospy.is_shutdown():
            data = self.serial.read(self.serial.in_waiting or 1)
            if data:
                chunks.append(data)
            else:
                time.sleep(0.01)

        return b"".join(chunks).decode("utf-8", errors="replace").strip()

    def close(self):
        if self.serial and self.serial.is_open:
            self.serial.close()


def main():
    rospy.init_node("tiplight_serial_node")
    node = TiplightSerialNode()
    rospy.on_shutdown(node.close)
    rospy.spin()


if __name__ == "__main__":
    main()

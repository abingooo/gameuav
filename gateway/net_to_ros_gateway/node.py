#!/usr/bin/env python3

import argparse
import threading

from comm.agent_client import AgentClient
from comm.tcp_link.server import serve_forever
from gateway.message_adapter.adapter import network_command_to_ros_command


class NetToRosGateway:
    def __init__(self, agent_client=None):
        self.agent_client = agent_client
        self._ros_init_lock = threading.Lock()

    def handle_network_command(self, command, args, message=None):
        ros_command = network_command_to_ros_command(command, args or {})
        if ros_command["type"] == "agent_ros_command":
            if not self.agent_client:
                raise RuntimeError("agent_client is required for %s" % command)
            return self.agent_client.send_ros_command(ros_command["command"], args=ros_command["args"])
        if ros_command["type"] == "topic":
            return self.publish_ros_topic(ros_command)
        raise RuntimeError("unsupported adapted command type: %s" % ros_command["type"])

    def publish_ros_topic(self, ros_command):
        self._ensure_ros_initialized()

        import rospy
        import roslib.message

        msg_class = roslib.message.get_message_class(ros_command["msg_type"])
        if msg_class is None:
            raise RuntimeError("unknown ROS message type: %s" % ros_command["msg_type"])
        publisher = rospy.Publisher(ros_command["topic"], msg_class, queue_size=10, latch=True)
        message = _dict_to_ros_message(msg_class, ros_command["payload"])
        publisher.publish(message)
        return {
            "topic": ros_command["topic"],
            "msg_type": ros_command["msg_type"],
            "published": True,
        }

    def _ensure_ros_initialized(self):
        import rospy

        if rospy.core.is_initialized():
            return
        with self._ros_init_lock:
            if not rospy.core.is_initialized():
                rospy.init_node("net_to_ros_gateway", disable_signals=True)


def _dict_to_ros_message(msg_class, payload):
    message = msg_class()
    _assign_fields(message, payload)
    return message


def _assign_fields(message, payload):
    for key, value in payload.items():
        if isinstance(value, dict):
            _assign_fields(getattr(message, key), value)
        else:
            setattr(message, key, value)


def main(argv=None):
    parser = argparse.ArgumentParser(description="TCP network command to ROS gateway")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=9100)
    parser.add_argument("--uav-id", default="uav1")
    parser.add_argument("--agent-host", default="127.0.0.1")
    parser.add_argument("--agent-port", type=int, default=8765)
    parser.add_argument("--agent-token", default="uavuavuavuav")
    parser.add_argument("--agent-timeout", type=float, default=5.0)
    args = parser.parse_args(argv)
    agent_client = AgentClient(
        host=args.agent_host,
        port=args.agent_port,
        auth_token=args.agent_token,
        source_id="net_to_ros_gateway",
        target_id=args.uav_id,
        timeout=args.agent_timeout,
    )
    gateway = NetToRosGateway(agent_client=agent_client)
    serve_forever(args.host, args.port, args.uav_id, gateway.handle_network_command)


if __name__ == "__main__":
    main()

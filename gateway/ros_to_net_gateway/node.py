#!/usr/bin/env python3

import argparse
import time

from comm.protocol.network_protocol import make_state
from comm.udp_link.link import UdpLink
from gateway.message_adapter.adapter import adapt_ros_dict_to_state


def _ros_message_to_dict(message):
    if hasattr(message, "pose") and hasattr(message.pose, "pose"):
        return {
            "pose": {
                "pose": {
                    "position": {
                        "x": message.pose.pose.position.x,
                        "y": message.pose.pose.position.y,
                        "z": message.pose.pose.position.z,
                    },
                    "orientation": {
                        "x": message.pose.pose.orientation.x,
                        "y": message.pose.pose.orientation.y,
                        "z": message.pose.pose.orientation.z,
                        "w": message.pose.pose.orientation.w,
                    },
                }
            },
            "twist": {
                "twist": {
                    "linear": {
                        "x": message.twist.twist.linear.x,
                        "y": message.twist.twist.linear.y,
                        "z": message.twist.twist.linear.z,
                    }
                }
            },
        }
    if hasattr(message, "percentage") and hasattr(message, "voltage"):
        return {
            "percentage": message.percentage,
            "voltage": message.voltage,
            "current": message.current,
        }
    if hasattr(message, "connected") and hasattr(message, "armed"):
        return {
            "connected": message.connected,
            "armed": message.armed,
            "guided": message.guided,
            "mode": message.mode,
        }
    raise TypeError("unsupported ROS message object: %s" % type(message).__name__)


class RosToNetGateway:
    def __init__(self, uav_id, udp_host, udp_port, mappings, bind_host="0.0.0.0", bind_port=0):
        self.uav_id = uav_id
        self.udp_host = udp_host
        self.udp_port = int(udp_port)
        self.mappings = mappings
        self.link = UdpLink(bind_host, bind_port)

    def handle_ros_message(self, topic, msg_type, ros_message):
        payload = _ros_message_to_dict(ros_message)
        state = adapt_ros_dict_to_state(self.uav_id, topic, msg_type, payload)
        envelope = make_state(self.uav_id, target_id="*", state=state)
        self.link.send(envelope, self.udp_host, self.udp_port)
        return envelope


def main(argv=None):
    parser = argparse.ArgumentParser(description="ROS topic to UDP network gateway")
    parser.add_argument("--uav-id", default="uav1")
    parser.add_argument("--udp-host", default="127.0.0.1")
    parser.add_argument("--udp-port", type=int, default=9001)
    parser.add_argument("--rate-hz", type=float, default=5.0)
    args = parser.parse_args(argv)

    import rospy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import BatteryState
    from mavros_msgs.msg import State

    gateway = RosToNetGateway(
        uav_id=args.uav_id,
        udp_host=args.udp_host,
        udp_port=args.udp_port,
        mappings={},
    )

    rospy.init_node("ros_to_net_gateway")

    min_period = 1.0 / max(args.rate_hz, 0.1)
    last_sent = {}

    def throttled(topic, msg_type):
        def callback(message):
            now = time.time()
            if now - last_sent.get(topic, 0.0) < min_period:
                return
            last_sent[topic] = now
            gateway.handle_ros_message(topic, msg_type, message)

        return callback

    rospy.Subscriber("/vins_fusion/imu_propagate", Odometry, throttled("/vins_fusion/imu_propagate", "nav_msgs/Odometry"), queue_size=10)
    rospy.Subscriber("/mavros/battery", BatteryState, throttled("/mavros/battery", "sensor_msgs/BatteryState"), queue_size=10)
    rospy.Subscriber("/mavros/state", State, throttled("/mavros/state", "mavros_msgs/State"), queue_size=10)
    rospy.spin()


if __name__ == "__main__":
    main()

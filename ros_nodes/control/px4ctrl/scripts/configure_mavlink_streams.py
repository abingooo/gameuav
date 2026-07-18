#!/usr/bin/env python3

import math
import sys
import time

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong


MAV_CMD_SET_MESSAGE_INTERVAL = 511

DEFAULT_STREAMS = [
    {"name": "HIGHRES_IMU", "message_id": 105, "interval_us": 4550},
    {"name": "ATTITUDE_QUATERNION", "message_id": 31, "interval_us": 4550},
]


def resolve_mavros_name(mavros_ns, suffix):
    base = rospy.resolve_name(mavros_ns).rstrip("/")
    return base + suffix


def wait_for_mavros_connection(state_topic, timeout):
    deadline = time.time() + timeout
    while not rospy.is_shutdown() and time.time() < deadline:
        try:
            remaining = max(0.1, deadline - time.time())
            msg = rospy.wait_for_message(state_topic, State, timeout=min(1.0, remaining))
            if msg.connected:
                return True
        except rospy.ROSException:
            pass
    return False


def stream_interval_us(stream):
    if "interval_us" in stream:
        return float(stream["interval_us"])

    rate_hz = float(stream.get("rate_hz", 0.0))
    if rate_hz <= 0.0 or not math.isfinite(rate_hz):
        raise ValueError("stream must define positive interval_us or rate_hz")
    return 1e6 / rate_hz


def configure_stream(command_srv, stream):
    name = str(stream.get("name", "message_%s" % stream.get("message_id", "unknown")))
    message_id = int(stream["message_id"])
    interval_us = stream_interval_us(stream)

    resp = command_srv(
        broadcast=False,
        command=MAV_CMD_SET_MESSAGE_INTERVAL,
        confirmation=0,
        param1=float(message_id),
        param2=float(interval_us),
        param3=0.0,
        param4=0.0,
        param5=0.0,
        param6=0.0,
        param7=0.0,
    )
    return name, message_id, interval_us, resp


def main():
    rospy.init_node("configure_mavlink_streams")

    mavros_ns = rospy.get_param("~mavros_ns", "mavros")
    streams = rospy.get_param("~streams", DEFAULT_STREAMS)
    wait_for_connection = bool(rospy.get_param("~wait_for_connection", True))
    connection_timeout = float(rospy.get_param("~connection_timeout", 30.0))
    service_timeout = float(rospy.get_param("~service_timeout", 30.0))
    apply_attempts = int(rospy.get_param("~apply_attempts", 3))
    retry_interval = float(rospy.get_param("~retry_interval", 1.0))

    command_service = resolve_mavros_name(mavros_ns, "/cmd/command")
    state_topic = resolve_mavros_name(mavros_ns, "/state")

    if wait_for_connection:
        rospy.loginfo("waiting for MAVROS connection on %s", state_topic)
        if not wait_for_mavros_connection(state_topic, connection_timeout):
            rospy.logerr("MAVROS did not connect within %.1f seconds", connection_timeout)
            return 1

    rospy.loginfo("waiting for MAVROS command service %s", command_service)
    try:
        rospy.wait_for_service(command_service, timeout=service_timeout)
    except rospy.ROSException as exc:
        rospy.logerr("MAVROS command service unavailable: %s", exc)
        return 1

    command_srv = rospy.ServiceProxy(command_service, CommandLong)

    for attempt in range(1, max(1, apply_attempts) + 1):
        all_success = True
        for stream in streams:
            try:
                name, message_id, interval_us, resp = configure_stream(command_srv, stream)
            except Exception as exc:
                all_success = False
                rospy.logwarn("failed to request MAVLink stream %s: %s", stream, exc)
                continue

            rate_hz = 1e6 / interval_us if interval_us > 0.0 else 0.0
            if resp.success:
                rospy.loginfo(
                    "requested MAVLink %s(%d) interval %.1f us (%.1f Hz), result=%d",
                    name,
                    message_id,
                    interval_us,
                    rate_hz,
                    resp.result,
                )
            else:
                all_success = False
                rospy.logwarn(
                    "MAVLink %s(%d) interval request rejected, result=%d",
                    name,
                    message_id,
                    resp.result,
                )

        if all_success:
            return 0

        if attempt < apply_attempts and not rospy.is_shutdown():
            rospy.sleep(retry_interval)

    return 1


if __name__ == "__main__":
    sys.exit(main())

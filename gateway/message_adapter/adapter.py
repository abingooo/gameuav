#!/usr/bin/env python3

import math


class MessageAdapterError(ValueError):
    pass


def _finite_or_none(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def pose_stamped_to_position(payload):
    pose = payload.get("pose") or {}
    position = pose.get("position") or {}
    orientation = pose.get("orientation") or {}
    return {
        "position": [
            _finite_or_none(position.get("x")),
            _finite_or_none(position.get("y")),
            _finite_or_none(position.get("z")),
        ],
        "orientation": {
            "x": _finite_or_none(orientation.get("x")),
            "y": _finite_or_none(orientation.get("y")),
            "z": _finite_or_none(orientation.get("z")),
            "w": _finite_or_none(orientation.get("w")),
        },
    }


def odometry_to_state(payload):
    pose = payload.get("pose", {}).get("pose", {})
    twist = payload.get("twist", {}).get("twist", {})
    position = pose.get("position") or {}
    orientation = pose.get("orientation") or {}
    linear = twist.get("linear") or {}
    orientation_state = {
        "x": _finite_or_none(orientation.get("x")),
        "y": _finite_or_none(orientation.get("y")),
        "z": _finite_or_none(orientation.get("z")),
        "w": _finite_or_none(orientation.get("w")),
    }
    return {
        "position": [
            _finite_or_none(position.get("x")),
            _finite_or_none(position.get("y")),
            _finite_or_none(position.get("z")),
        ],
        "orientation": orientation_state,
        "yaw": quaternion_to_yaw(orientation_state),
        "velocity": [
            _finite_or_none(linear.get("x")),
            _finite_or_none(linear.get("y")),
            _finite_or_none(linear.get("z")),
        ],
    }


def battery_to_state(payload):
    return {
        "battery": {
            "percentage": _finite_or_none(payload.get("percentage")),
            "voltage": _finite_or_none(payload.get("voltage")),
            "current": _finite_or_none(payload.get("current")),
        }
    }


def mavros_state_to_state(payload):
    return {
        "mavros": {
            "connected": bool(payload.get("connected", False)),
            "armed": bool(payload.get("armed", False)),
            "guided": bool(payload.get("guided", False)),
            "mode": payload.get("mode", ""),
        }
    }


def quaternion_to_yaw(orientation):
    x = orientation.get("x")
    y = orientation.get("y")
    z = orientation.get("z")
    w = orientation.get("w")
    if None in (x, y, z, w):
        return None
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def yaw_to_quaternion(yaw):
    yaw = _finite_or_none(yaw)
    if yaw is None:
        raise MessageAdapterError("yaw must be finite")
    half_yaw = yaw / 2.0
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(half_yaw),
        "w": math.cos(half_yaw),
    }


def _required_float(args, key):
    value = _finite_or_none(args.get(key))
    if value is None:
        raise MessageAdapterError("argument %s must be finite" % key)
    return value


def _optional_float(args, key, default=0.0):
    if key not in args:
        return float(default)
    value = _finite_or_none(args.get(key))
    if value is None:
        raise MessageAdapterError("argument %s must be finite" % key)
    return value


def _pose_topic_command(topic, args, default_frame_id):
    yaw = _optional_float(args, "yaw", 0.0)
    return {
        "type": "topic",
        "topic": topic,
        "msg_type": "geometry_msgs/PoseStamped",
        "payload": {
            "header": {"frame_id": args.get("frame_id", default_frame_id)},
            "pose": {
                "position": {
                    "x": _required_float(args, "x"),
                    "y": _required_float(args, "y"),
                    "z": _required_float(args, "z"),
                },
                "orientation": yaw_to_quaternion(yaw),
            },
        },
    }


def build_state_payload(uav_id, state=None, source_topic=None):
    payload = {
        "uav_id": uav_id,
    }
    if source_topic:
        payload["source_topic"] = source_topic
    if state:
        payload.update(state)
    return payload


def adapt_ros_dict_to_state(uav_id, topic, msg_type, payload):
    if msg_type == "nav_msgs/Odometry":
        state = odometry_to_state(payload)
    elif msg_type == "sensor_msgs/BatteryState":
        state = battery_to_state(payload)
    elif msg_type == "mavros_msgs/State":
        state = mavros_state_to_state(payload)
    elif msg_type == "geometry_msgs/PoseStamped":
        state = pose_stamped_to_position(payload)
    else:
        state = {"raw": payload}
    return build_state_payload(uav_id, state=state, source_topic=topic)


def network_command_to_ros_command(command, args):
    if command == "set_goal":
        return _pose_topic_command("/planning/goal", args, "world")
    if command in {"ego_position", "ego-position"}:
        return _pose_topic_command("/control/ego_position", args, "body")
    if command == "position":
        return _pose_topic_command("/control/position", args, "world")
    if command == "speed":
        return {
            "type": "topic",
            "topic": "/control/speed",
            "msg_type": "geometry_msgs/TwistStamped",
            "payload": {
                "header": {"frame_id": args.get("frame_id", "body")},
                "twist": {
                    "linear": {
                        "x": _required_float(args, "vx"),
                        "y": _required_float(args, "vy"),
                        "z": _required_float(args, "vz"),
                    },
                    "angular": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": _optional_float(args, "yaw_rate", 0.0),
                    },
                },
            },
        }
    if command == "stop":
        return {
            "type": "topic",
            "topic": "/control/stop",
            "msg_type": "std_msgs/Empty",
            "payload": {},
        }
    if command == "tiplight":
        return {
            "type": "topic",
            "topic": "/actuation/tiplight_cmd",
            "msg_type": "std_msgs/String",
            "payload": {"data": str(args["data"])},
        }
    if command == "takeoff":
        return {
            "type": "agent_ros_command",
            "command": "safe_takeoff",
            "args": {"dry_run": bool(args.get("dry_run", True))},
        }
    if command == "land":
        return {
            "type": "agent_ros_command",
            "command": "safe_land",
            "args": {"dry_run": bool(args.get("dry_run", True))},
        }
    raise MessageAdapterError("unsupported network command: %s" % command)

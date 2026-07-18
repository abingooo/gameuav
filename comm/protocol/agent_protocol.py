#!/usr/bin/env python3

import json
import time
import uuid
import zlib


PROTOCOL_VERSION = "gameuav.agent.v1"

MESSAGE_TYPE_MODULE_COMMAND = "module_command"
MESSAGE_TYPE_MODULE_STATUS = "module_status"
MESSAGE_TYPE_ROS_COMMAND = "ros_command"
MESSAGE_TYPE_ROS_COMMAND_RESULT = "ros_command_result"
MESSAGE_TYPE_ERROR = "error"

ALLOWED_ACTIONS = {"start", "stop", "restart", "status", "list", "health"}


class ProtocolError(ValueError):
    pass


def now_timestamp():
    return time.time()


def checksum_payload(payload):
    data = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return format(zlib.crc32(data) & 0xFFFFFFFF, "08x")


def make_envelope(message_type, source_id, target_id, payload, sequence_id=None):
    envelope = {
        "protocol_version": PROTOCOL_VERSION,
        "message_type": message_type,
        "source_id": source_id,
        "target_id": target_id,
        "sequence_id": sequence_id if sequence_id is not None else uuid.uuid4().hex,
        "timestamp": now_timestamp(),
        "payload": payload,
    }
    envelope["checksum"] = checksum_payload({k: v for k, v in envelope.items() if k != "checksum"})
    return envelope


def validate_envelope(envelope):
    if not isinstance(envelope, dict):
        raise ProtocolError("message must be a JSON object")

    required = [
        "protocol_version",
        "message_type",
        "source_id",
        "target_id",
        "sequence_id",
        "timestamp",
        "payload",
        "checksum",
    ]
    for key in required:
        if key not in envelope:
            raise ProtocolError("missing field: %s" % key)

    if envelope["protocol_version"] != PROTOCOL_VERSION:
        raise ProtocolError("unsupported protocol_version: %s" % envelope["protocol_version"])

    expected = checksum_payload({k: v for k, v in envelope.items() if k != "checksum"})
    if envelope["checksum"] != expected:
        raise ProtocolError("checksum mismatch")

    if not isinstance(envelope["payload"], dict):
        raise ProtocolError("payload must be an object")

    return envelope


def encode_message(envelope):
    validate_envelope(envelope)
    return (json.dumps(envelope, sort_keys=True, separators=(",", ":")) + "\n").encode("utf-8")


def decode_message(line):
    if isinstance(line, bytes):
        line = line.decode("utf-8")
    try:
        envelope = json.loads(line)
    except json.JSONDecodeError as exc:
        raise ProtocolError("invalid JSON: %s" % exc)
    return validate_envelope(envelope)


def make_module_command(
    action,
    module=None,
    args=None,
    source_id="gcs",
    target_id="uav1",
    request_id=None,
    auth_token=None,
):
    if action not in ALLOWED_ACTIONS:
        raise ProtocolError("unsupported action: %s" % action)
    if action not in {"list", "health"} and not module:
        raise ProtocolError("module is required for action %s" % action)

    payload = {
        "request_id": request_id if request_id is not None else uuid.uuid4().hex,
        "action": action,
    }
    if auth_token is not None:
        payload["auth_token"] = auth_token
    if module is not None:
        payload["module"] = module
    if args:
        payload["args"] = args
    return make_envelope(MESSAGE_TYPE_MODULE_COMMAND, source_id, target_id, payload)


def make_module_status(source_id, target_id, request_id, payload, sequence_id=None):
    status_payload = dict(payload)
    status_payload["request_id"] = request_id
    return make_envelope(MESSAGE_TYPE_MODULE_STATUS, source_id, target_id, status_payload, sequence_id)


def make_ros_command(
    command,
    args=None,
    source_id="gcs",
    target_id="uav1",
    request_id=None,
    auth_token=None,
):
    if not command:
        raise ProtocolError("command is required")

    payload = {
        "request_id": request_id if request_id is not None else uuid.uuid4().hex,
        "command": command,
    }
    if auth_token is not None:
        payload["auth_token"] = auth_token
    if args:
        payload["args"] = args
    return make_envelope(MESSAGE_TYPE_ROS_COMMAND, source_id, target_id, payload)


def make_ros_command_result(source_id, target_id, request_id, payload, sequence_id=None):
    result_payload = dict(payload)
    result_payload["request_id"] = request_id
    return make_envelope(MESSAGE_TYPE_ROS_COMMAND_RESULT, source_id, target_id, result_payload, sequence_id)


def make_error(source_id, target_id, request_id, code, detail, sequence_id=None):
    return make_envelope(
        MESSAGE_TYPE_ERROR,
        source_id,
        target_id,
        {
            "request_id": request_id,
            "ok": False,
            "code": code,
            "detail": detail,
        },
        sequence_id,
    )

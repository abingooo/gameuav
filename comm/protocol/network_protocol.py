#!/usr/bin/env python3

import time
import uuid
import zlib

from comm.serializer.json_serializer import dumps_message, loads_message


PROTOCOL_VERSION = "gameuav.net.v1"

MESSAGE_TYPE_HEARTBEAT = "heartbeat"
MESSAGE_TYPE_STATE = "state"
MESSAGE_TYPE_ALERT = "alert"
MESSAGE_TYPE_COMMAND = "command"
MESSAGE_TYPE_ACK = "ack"
MESSAGE_TYPE_RESULT = "result"
MESSAGE_TYPE_ERROR = "error"

RELIABILITY_BEST_EFFORT = "best_effort"
RELIABILITY_RELIABLE = "reliable"

VALID_MESSAGE_TYPES = {
    MESSAGE_TYPE_HEARTBEAT,
    MESSAGE_TYPE_STATE,
    MESSAGE_TYPE_ALERT,
    MESSAGE_TYPE_COMMAND,
    MESSAGE_TYPE_ACK,
    MESSAGE_TYPE_RESULT,
    MESSAGE_TYPE_ERROR,
}

VALID_RELIABILITY = {
    RELIABILITY_BEST_EFFORT,
    RELIABILITY_RELIABLE,
}


class NetworkProtocolError(ValueError):
    pass


def now_timestamp():
    return time.time()


def checksum_payload(payload):
    data = dumps_message(payload)
    return format(zlib.crc32(data) & 0xFFFFFFFF, "08x")


def make_envelope(
    message_type,
    source_id,
    target_id,
    payload=None,
    sequence_id=None,
    reliability=RELIABILITY_BEST_EFFORT,
    timestamp=None,
):
    if message_type not in VALID_MESSAGE_TYPES:
        raise NetworkProtocolError("invalid message_type: %s" % message_type)
    if reliability not in VALID_RELIABILITY:
        raise NetworkProtocolError("invalid reliability: %s" % reliability)

    envelope = {
        "protocol_version": PROTOCOL_VERSION,
        "message_type": message_type,
        "source_id": source_id,
        "target_id": target_id,
        "sequence_id": sequence_id if sequence_id is not None else uuid.uuid4().hex,
        "timestamp": timestamp if timestamp is not None else now_timestamp(),
        "reliability": reliability,
        "payload": payload or {},
    }
    envelope["checksum"] = checksum_payload({k: v for k, v in envelope.items() if k != "checksum"})
    return envelope


def validate_envelope(envelope):
    if not isinstance(envelope, dict):
        raise NetworkProtocolError("message must be object")

    required = [
        "protocol_version",
        "message_type",
        "source_id",
        "target_id",
        "sequence_id",
        "timestamp",
        "reliability",
        "payload",
        "checksum",
    ]
    for key in required:
        if key not in envelope:
            raise NetworkProtocolError("missing field: %s" % key)

    if envelope["protocol_version"] != PROTOCOL_VERSION:
        raise NetworkProtocolError("unsupported protocol_version: %s" % envelope["protocol_version"])
    if envelope["message_type"] not in VALID_MESSAGE_TYPES:
        raise NetworkProtocolError("invalid message_type: %s" % envelope["message_type"])
    if envelope["reliability"] not in VALID_RELIABILITY:
        raise NetworkProtocolError("invalid reliability: %s" % envelope["reliability"])
    if not isinstance(envelope["payload"], dict):
        raise NetworkProtocolError("payload must be object")

    expected = checksum_payload({k: v for k, v in envelope.items() if k != "checksum"})
    if envelope["checksum"] != expected:
        raise NetworkProtocolError("checksum mismatch")

    return envelope


def encode_message(envelope):
    return dumps_message(validate_envelope(envelope))


def encode_line(envelope):
    return encode_message(envelope) + b"\n"


def decode_message(data):
    return validate_envelope(loads_message(data))


def make_heartbeat(source_id, target_id="*", status="online", payload=None):
    heartbeat_payload = {"status": status}
    if payload:
        heartbeat_payload.update(payload)
    return make_envelope(
        MESSAGE_TYPE_HEARTBEAT,
        source_id,
        target_id,
        heartbeat_payload,
        reliability=RELIABILITY_BEST_EFFORT,
    )


def make_state(source_id, target_id="*", state=None):
    return make_envelope(
        MESSAGE_TYPE_STATE,
        source_id,
        target_id,
        state or {},
        reliability=RELIABILITY_BEST_EFFORT,
    )


def make_command(source_id, target_id, command, args=None, request_id=None):
    return make_envelope(
        MESSAGE_TYPE_COMMAND,
        source_id,
        target_id,
        {
            "request_id": request_id if request_id is not None else uuid.uuid4().hex,
            "command": command,
            "args": args or {},
        },
        reliability=RELIABILITY_RELIABLE,
    )


def make_ack(source_id, target_id, request_id, accepted=True, detail=""):
    return make_envelope(
        MESSAGE_TYPE_ACK,
        source_id,
        target_id,
        {
            "request_id": request_id,
            "accepted": bool(accepted),
            "detail": detail,
        },
        reliability=RELIABILITY_RELIABLE,
    )


def make_result(source_id, target_id, request_id, ok, result=None):
    return make_envelope(
        MESSAGE_TYPE_RESULT,
        source_id,
        target_id,
        {
            "request_id": request_id,
            "ok": bool(ok),
            "result": result or {},
        },
        reliability=RELIABILITY_RELIABLE,
    )


def make_error(source_id, target_id, request_id=None, code="error", detail=""):
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
        reliability=RELIABILITY_RELIABLE,
    )

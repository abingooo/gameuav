#!/usr/bin/env python3

import json


class SerializationError(ValueError):
    pass


def dumps_message(message):
    try:
        return json.dumps(message, sort_keys=True, separators=(",", ":")).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise SerializationError("failed to serialize message: %s" % exc)


def dumps_line(message):
    return dumps_message(message) + b"\n"


def loads_message(data):
    if isinstance(data, bytes):
        data = data.decode("utf-8")
    try:
        return json.loads(data)
    except json.JSONDecodeError as exc:
        raise SerializationError("failed to parse JSON message: %s" % exc)

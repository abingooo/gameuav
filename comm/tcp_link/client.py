#!/usr/bin/env python3

import socket

from comm.protocol.network_protocol import (
    MESSAGE_TYPE_ACK,
    MESSAGE_TYPE_ERROR,
    MESSAGE_TYPE_RESULT,
    decode_message,
    encode_line,
    make_command,
)


class TcpCommandClientError(RuntimeError):
    pass


class TcpCommandClient:
    def __init__(self, host, port, source_id="gcs", target_id="uav1", timeout=5.0):
        self.host = host
        self.port = int(port)
        self.source_id = source_id
        self.target_id = target_id
        self.timeout = float(timeout)

    def send_command(self, command, args=None):
        message = make_command(self.source_id, self.target_id, command, args=args or {})
        try:
            with socket.create_connection((self.host, self.port), timeout=self.timeout) as sock:
                sock.settimeout(self.timeout)
                sock.sendall(encode_line(message))
                reader = sock.makefile("rb")
                first = self._read_message(reader)
                if first["message_type"] == MESSAGE_TYPE_ERROR:
                    raise TcpCommandClientError(first["payload"].get("detail", "command rejected"))
                if first["message_type"] != MESSAGE_TYPE_ACK:
                    raise TcpCommandClientError("expected ack, got %s" % first["message_type"])
                if not first["payload"].get("accepted", False):
                    raise TcpCommandClientError(first["payload"].get("detail", "command not accepted"))
                second = self._read_message(reader)
                if second["message_type"] == MESSAGE_TYPE_ERROR:
                    raise TcpCommandClientError(second["payload"].get("detail", "command failed"))
                if second["message_type"] != MESSAGE_TYPE_RESULT:
                    raise TcpCommandClientError("expected result, got %s" % second["message_type"])
                return {
                    "ack": first,
                    "result": second,
                }
        except (OSError, socket.timeout) as exc:
            raise TcpCommandClientError("tcp command failed: %s" % exc)

    def probe_connection(self, timeout=None):
        timeout = self.timeout if timeout is None else float(timeout)
        try:
            with socket.create_connection((self.host, self.port), timeout=timeout):
                return {
                    "host": self.host,
                    "port": self.port,
                }
        except (OSError, socket.timeout) as exc:
            raise TcpCommandClientError("tcp gateway unavailable: %s" % exc)

    @staticmethod
    def _read_message(reader):
        line = reader.readline()
        if not line:
            raise TcpCommandClientError("connection closed while waiting for response")
        return decode_message(line)

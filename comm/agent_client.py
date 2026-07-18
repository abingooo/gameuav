#!/usr/bin/env python3

import socket
from dataclasses import dataclass

from comm.protocol.agent_protocol import (
    MESSAGE_TYPE_ERROR,
    ProtocolError,
    decode_message,
    encode_message,
    make_module_command,
    make_ros_command,
)


class AgentClientError(RuntimeError):
    pass


@dataclass
class AgentClient:
    host: str
    port: int = 8765
    auth_token: str = "uavuavuavuav"
    source_id: str = "gcs_backend"
    target_id: str = "uav1"
    timeout: float = 5.0

    def send_module_command(self, action, module=None, args=None):
        message = make_module_command(
            action=action,
            module=module,
            args=args or {},
            source_id=self.source_id,
            target_id=self.target_id,
            auth_token=self.auth_token,
        )
        response_timeout = max(self.timeout, 60.0) if action in {"start", "restart"} else self.timeout
        return self._send(message, response_timeout=response_timeout)

    def send_ros_command(self, command, args=None):
        message = make_ros_command(
            command=command,
            args=args or {},
            source_id=self.source_id,
            target_id=self.target_id,
            auth_token=self.auth_token,
        )
        return self._send(message)

    def _send(self, message, response_timeout=None):
        response_timeout = self.timeout if response_timeout is None else float(response_timeout)
        try:
            with socket.create_connection((self.host, self.port), timeout=self.timeout) as sock:
                sock.settimeout(response_timeout)
                sock.sendall(encode_message(message))
                line = sock.makefile("rb").readline()
        except (OSError, socket.timeout) as exc:
            raise AgentClientError("failed to reach agent %s:%s: %s" % (self.host, self.port, exc))

        if not line:
            raise AgentClientError("agent closed connection without response")

        try:
            response = decode_message(line)
        except ProtocolError as exc:
            raise AgentClientError("invalid agent response: %s" % exc)

        return response

    @staticmethod
    def is_error_response(response):
        return response.get("message_type") == MESSAGE_TYPE_ERROR

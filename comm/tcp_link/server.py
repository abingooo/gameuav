#!/usr/bin/env python3

import socketserver
import traceback

from comm.protocol.network_protocol import (
    MESSAGE_TYPE_COMMAND,
    NetworkProtocolError,
    decode_message,
    encode_line,
    make_ack,
    make_error,
    make_result,
)


class TcpCommandHandler(socketserver.StreamRequestHandler):
    def handle(self):
        for raw_line in self.rfile:
            response_messages = self.server.handle_raw_message(raw_line)
            for message in response_messages:
                self.wfile.write(encode_line(message))
                self.wfile.flush()


class TcpCommandServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, server_address, node_id, command_handler):
        super().__init__(server_address, TcpCommandHandler)
        self.node_id = node_id
        self.command_handler = command_handler

    def handle_raw_message(self, raw_line):
        source_id = "unknown"
        request_id = None
        try:
            message = decode_message(raw_line)
            source_id = message["source_id"]
            payload = message["payload"]
            request_id = payload.get("request_id")

            if message["target_id"] not in (self.node_id, "*"):
                raise NetworkProtocolError("target mismatch: %s" % message["target_id"])
            if message["message_type"] != MESSAGE_TYPE_COMMAND:
                raise NetworkProtocolError("expected command message")

            ack = make_ack(self.node_id, source_id, request_id, accepted=True)
            result_payload = self.command_handler(payload.get("command"), payload.get("args") or {}, message)
            result = make_result(self.node_id, source_id, request_id, ok=True, result=result_payload)
            return [ack, result]
        except Exception as exc:
            detail = str(exc)
            if not isinstance(exc, NetworkProtocolError):
                detail = "%s: %s" % (exc.__class__.__name__, detail)
            error = make_error(self.node_id, source_id, request_id, exc.__class__.__name__, detail)
            return [error]


def serve_forever(host, port, node_id, command_handler):
    with TcpCommandServer((host, port), node_id, command_handler) as server:
        server.serve_forever()

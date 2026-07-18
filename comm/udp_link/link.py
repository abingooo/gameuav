#!/usr/bin/env python3

import socket
import threading
import time

from comm.protocol.network_protocol import (
    MESSAGE_TYPE_HEARTBEAT,
    MESSAGE_TYPE_STATE,
    NetworkProtocolError,
    decode_message,
    encode_message,
    make_heartbeat,
    make_state,
)


class UdpLink:
    def __init__(self, bind_host="0.0.0.0", bind_port=0, recv_timeout=0.2, allow_broadcast=True):
        self.bind_host = bind_host
        self.bind_port = int(bind_port)
        self.recv_timeout = float(recv_timeout)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if allow_broadcast:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind((self.bind_host, self.bind_port))
        self.sock.settimeout(self.recv_timeout)

    @property
    def address(self):
        return self.sock.getsockname()

    def send(self, message, host, port):
        data = encode_message(message)
        self.sock.sendto(data, (host, int(port)))

    def send_heartbeat(self, source_id, host, port, target_id="*", status="online", payload=None):
        self.send(make_heartbeat(source_id, target_id=target_id, status=status, payload=payload), host, port)

    def send_state(self, source_id, host, port, target_id="*", state=None):
        self.send(make_state(source_id, target_id=target_id, state=state or {}), host, port)

    def recv(self):
        data, address = self.sock.recvfrom(65535)
        return decode_message(data), address

    def close(self):
        self.sock.close()


class UdpReceiver:
    def __init__(self, link, on_message):
        self.link = link
        self.on_message = on_message
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self, timeout=2.0):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout)

    def _run(self):
        while not self._stop.is_set():
            try:
                message, address = self.link.recv()
            except socket.timeout:
                continue
            except OSError:
                break
            except NetworkProtocolError:
                continue
            self.on_message(message, address)


class PeriodicUdpPublisher:
    def __init__(self, link, interval_sec, build_message, destination):
        self.link = link
        self.interval_sec = float(interval_sec)
        self.build_message = build_message
        self.destination = destination
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self, timeout=2.0):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout)

    def _run(self):
        host, port = self.destination
        while not self._stop.is_set():
            self.link.send(self.build_message(), host, port)
            self._stop.wait(self.interval_sec)

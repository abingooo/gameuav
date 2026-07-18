#!/usr/bin/env python3

import time


class HeartbeatMonitor:
    def __init__(self, timeout_sec=3.0, now_func=None):
        self.timeout_sec = float(timeout_sec)
        self.now_func = now_func or time.time
        self._peers = {}

    def update(self, source_id, timestamp=None, payload=None):
        timestamp = self.now_func() if timestamp is None else float(timestamp)
        self._peers[source_id] = {
            "source_id": source_id,
            "last_seen": timestamp,
            "payload": payload or {},
        }

    def is_online(self, source_id, now=None):
        peer = self._peers.get(source_id)
        if not peer:
            return False
        now = self.now_func() if now is None else float(now)
        return now - peer["last_seen"] <= self.timeout_sec

    def peers(self, now=None):
        now = self.now_func() if now is None else float(now)
        result = {}
        for source_id, peer in self._peers.items():
            result[source_id] = dict(peer)
            result[source_id]["online"] = now - peer["last_seen"] <= self.timeout_sec
            result[source_id]["age_sec"] = max(0.0, now - peer["last_seen"])
        return result

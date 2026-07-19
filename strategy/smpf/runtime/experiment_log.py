"""Append-only structured experiment events with credential redaction."""

import json
import math
from pathlib import Path
import threading
import time


_SENSITIVE_KEY_PARTS = ("api_key", "apikey", "authorization", "credential", "secret", "token")


def _redact(value, key=""):
    normalized_key = str(key).lower().replace("-", "_")
    if any(part in normalized_key for part in _SENSITIVE_KEY_PARTS):
        return "<redacted>"
    if isinstance(value, dict):
        return {str(child_key): _redact(child_value, child_key) for child_key, child_value in value.items()}
    if isinstance(value, (list, tuple)):
        return [_redact(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


class JsonlExperimentLogger:
    """Write one independently parseable event per line for SPF/SMPF trials."""

    def __init__(self, path, method="smpf"):
        self.path = Path(path)
        self.method = str(method or "").strip()
        if not self.method:
            raise ValueError("experiment method cannot be empty")
        self._lock = threading.Lock()

    def log(self, event, task_id=None, **fields):
        event = str(event or "").strip()
        if not event:
            raise ValueError("experiment event cannot be empty")
        record = {
            "schema": "gameuav.experiment.event.v1",
            "timestamp": time.time(),
            "method": self.method,
            "event": event,
            "task_id": None if task_id is None else str(task_id),
        }
        record.update(fields)
        sanitized = _redact(record)
        line = json.dumps(sanitized, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        with self._lock:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open("a", encoding="utf-8") as stream:
                stream.write(line + "\n")
        return sanitized

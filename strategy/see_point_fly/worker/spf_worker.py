#!/usr/bin/env python3

"""HTTP boundary for See-Point-Fly inference.

This worker is intentionally isolated from ROS. It accepts a JSON payload from
the ROS bridge and returns a relative action suggestion:

{
  "ok": true,
  "action": {
    "dx": 0.0,
    "dy": 1.0,
    "dz": 0.0,
    "yaw_only": false,
    "label": "..."
  }
}

The worker is isolated from ROS so SPF can use its own Python environment.
"""

from __future__ import annotations

import argparse
import base64
import os
import sys
import json
import time
import urllib.request
import urllib.error
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict


os.environ.setdefault("PYNPUT_BACKEND", "dummy")


class WorkerError(RuntimeError):
    pass


_PROJECTOR = None
_LAST_VLM_RESPONSE_TEXT = ""
_LOG_FILENAMES = {"input.jpg", "annotated.jpg", "request.json", "response.json", "meta.json"}


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _upstream_root() -> Path:
    return _repo_root() / "strategy" / "see_point_fly" / "upstream"


def _ensure_upstream_path() -> Path:
    upstream = _upstream_root()
    src = upstream / "src"
    if not src.exists():
        raise WorkerError("SPF upstream not found at %s" % upstream)
    src_text = str(src)
    if src_text not in sys.path:
        sys.path.insert(0, src_text)
    return upstream


def _load_projector(image_width: int, image_height: int):
    global _PROJECTOR
    if _PROJECTOR is not None:
        if (
            getattr(_PROJECTOR, "image_width", None) == image_width
            and getattr(_PROJECTOR, "image_height", None) == image_height
        ):
            return _PROJECTOR
        _PROJECTOR = None

    upstream = _ensure_upstream_path()
    config_path = os.environ.get(
        "SPF_CONFIG_PATH",
        str(_repo_root() / "strategy" / "see_point_fly" / "adapter" / "config_tello.yaml"),
    )
    if not Path(config_path).exists():
        config_path = str(upstream / "config_tello.yaml")

    try:
        from spf.tello.action_projector import TelloActionProjector
    except Exception as exc:
        raise WorkerError(
            "failed to import SPF upstream dependencies: %s. "
            "Run the worker with the SPF uv environment." % exc
        )

    mode = os.environ.get("SPF_OPERATIONAL_MODE", "adaptive_mode")
    try:
        _PROJECTOR = TelloActionProjector(
            image_width=image_width,
            image_height=image_height,
            mode=mode,
            config_path=config_path,
        )
    except Exception as exc:
        raise WorkerError("failed to initialize SPF projector: %s" % exc)
    _patch_projector_wire_api(_PROJECTOR)
    return _PROJECTOR


def _patch_projector_wire_api(projector) -> None:
    if os.environ.get("SPF_OPENAI_WIRE_API", "").lower() != "responses":
        return
    vlm_client = getattr(projector, "vlm_client", None)
    if vlm_client is None or getattr(vlm_client, "api_provider", "") != "openai":
        return
    vlm_client.generate_response = _responses_generate_response.__get__(vlm_client, type(vlm_client))


def _extract_responses_text(response: Dict[str, Any]) -> str:
    texts = []
    for item in response.get("output", []) or []:
        for content in item.get("content", []) or []:
            if content.get("type") == "output_text" and content.get("text"):
                texts.append(content["text"])
    if texts:
        return "\n".join(texts)
    return str(response.get("output_text") or "")


def _set_last_vlm_response_text(text: str) -> str:
    global _LAST_VLM_RESPONSE_TEXT
    _LAST_VLM_RESPONSE_TEXT = str(text or "")
    return _LAST_VLM_RESPONSE_TEXT


def _masked_env(name: str) -> str:
    value = os.environ.get(name, "")
    if not value:
        return ""
    if len(value) <= 8:
        return "***"
    return value[:4] + "***" + value[-4:]


def _health_payload() -> Dict[str, Any]:
    return {
        "ok": True,
        "service": "spf_worker",
        "wire_api": os.environ.get("SPF_OPENAI_WIRE_API", "chat.completions"),
        "openai_base_url": os.environ.get("OPENAI_BASE_URL", ""),
        "openai_api_key": _masked_env("OPENAI_API_KEY"),
        "spf_config_path": os.environ.get("SPF_CONFIG_PATH", ""),
        "logging_enabled": _spf_logging_enabled(),
        "log_dir": str(_spf_log_root()),
    }


def _responses_generate_response(vlm_client, prompt: str, image) -> str:
    try:
        import cv2
    except Exception as exc:
        raise WorkerError("missing cv2 for Responses API image encoding: %s" % exc)

    ok, buffer = cv2.imencode(".jpg", image)
    if not ok:
        raise WorkerError("failed to encode image for Responses API")

    encoded_image = base64.b64encode(buffer).decode("utf-8")
    api_key = os.environ.get("OPENAI_API_KEY")
    if not api_key:
        raise WorkerError("OPENAI_API_KEY not found in environment variables")
    base_url = os.environ.get("OPENAI_BASE_URL", "https://api.openai.com/v1").rstrip("/")
    if base_url.endswith("/v1"):
        base_url = base_url[:-3]
    url = base_url + "/responses"
    payload = {
        "model": vlm_client.model_name,
        "input": [
            {
                "role": "user",
                "content": [
                    {"type": "input_text", "text": prompt},
                    {
                        "type": "input_image",
                        "image_url": "data:image/jpeg;base64,%s" % encoded_image,
                    },
                ],
            }
        ],
        "temperature": 0.4,
        "max_output_tokens": 2048,
    }
    data = json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(
        url,
        data=data,
        headers={
            "Authorization": "Bearer %s" % api_key,
            "Content-Type": "application/json",
        },
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=float(os.environ.get("SPF_API_TIMEOUT_SEC", "90"))) as response:
            body = json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        body = exc.read().decode("utf-8", "replace")
        raise WorkerError("Responses API http %s: %s" % (exc.code, body[:500]))
    except Exception as exc:
        raise WorkerError("Responses API request failed: %s" % exc)
    return _set_last_vlm_response_text(_extract_responses_text(body))


def _decode_image(image_jpeg_b64: str):
    try:
        image_bytes = base64.b64decode(image_jpeg_b64, validate=True)
    except Exception as exc:
        raise WorkerError("invalid image_jpeg_b64: %s" % exc)
    try:
        import cv2
        import numpy as np
    except Exception as exc:
        raise WorkerError("missing image dependencies: %s" % exc)

    encoded = np.frombuffer(image_bytes, dtype=np.uint8)
    image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if image is None:
        raise WorkerError("image_jpeg_b64 did not decode to an image")
    return image


def _spf_logging_enabled() -> bool:
    return os.environ.get("SPF_LOG_ENABLED", "true").lower() not in {"0", "false", "no"}


def _spf_log_root() -> Path:
    return Path(os.environ.get("SPF_LOG_DIR", str(_repo_root() / "logs" / "spf")))


def _new_log_dir() -> Path:
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    suffix = "%06d" % int((time.time() % 1.0) * 1000000)
    path = _spf_log_root() / ("%s_%s" % (timestamp, suffix))
    path.mkdir(parents=True, exist_ok=True)
    return path


def _safe_log_file(request_path: str) -> Path:
    path = urllib.parse.urlparse(request_path).path
    parts = [urllib.parse.unquote(part) for part in path.split("/") if part]
    if len(parts) != 3 or parts[0] != "logs":
        raise WorkerError("invalid log path")
    run_id, filename = parts[1], parts[2]
    if run_id in {".", ".."} or "/" in run_id or "\\" in run_id:
        raise WorkerError("invalid log run id")
    if filename not in _LOG_FILENAMES:
        raise WorkerError("invalid log filename")
    root = _spf_log_root().resolve()
    file_path = (root / run_id / filename).resolve()
    try:
        file_path.relative_to(root)
    except ValueError as exc:
        raise WorkerError("invalid log path") from exc
    return file_path


def _latest_log_payload() -> Dict[str, Any]:
    root = _spf_log_root().resolve()
    try:
        candidates = [
            path
            for path in root.iterdir()
            if path.is_dir() and (path / "annotated.jpg").is_file()
        ]
    except OSError as exc:
        raise WorkerError("failed to inspect log directory: %s" % exc)
    if not candidates:
        return {"ok": True, "latest": None}
    latest = max(candidates, key=lambda path: (path / "annotated.jpg").stat().st_mtime)
    annotated = latest / "annotated.jpg"
    return {
        "ok": True,
        "latest": {
            "run_id": latest.name,
            "log_dir": str(latest),
            "updated_at": annotated.stat().st_mtime,
            "has_annotated": True,
            "has_input": (latest / "input.jpg").is_file(),
        },
    }


def _write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _safe_payload_for_log(payload: Dict[str, Any], image_jpeg_b64: str) -> Dict[str, Any]:
    safe = dict(payload)
    safe.pop("image_jpeg_b64", None)
    safe["image_jpeg_bytes"] = int(len(image_jpeg_b64) * 3 / 4) if image_jpeg_b64 else 0
    return safe


def _draw_annotation(image, action: Dict[str, Any], command: str):
    import cv2

    annotated = image.copy()
    screen_x = action.get("screen_x")
    screen_y = action.get("screen_y")
    if screen_x is not None and screen_y is not None:
        point = (int(round(float(screen_x))), int(round(float(screen_y))))
        cv2.circle(annotated, point, 10, (0, 255, 0), -1)
        cv2.circle(annotated, point, 18, (255, 255, 255), 2)
        label = "dx %.2f dy %.2f dz %.2f" % (
            float(action.get("dx", 0.0)),
            float(action.get("dy", 0.0)),
            float(action.get("dz", 0.0)),
        )
        _draw_text(annotated, label, (point[0] + 14, max(24, point[1] - 10)), 18)
    _draw_text(annotated, command[:80], (12, 12), 22)
    return annotated


def _draw_text(image, text: str, origin, font_size: int):
    try:
        import cv2
        import numpy as np
        from PIL import Image, ImageDraw, ImageFont
    except Exception:
        import cv2

        cv2.putText(image, str(text), origin, cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        return

    font_path = _find_cjk_font()
    if not font_path:
        cv2.putText(image, str(text), origin, cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        return
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    pil_image = Image.fromarray(rgb)
    draw = ImageDraw.Draw(pil_image)
    font = ImageFont.truetype(font_path, font_size)
    x, y = int(origin[0]), int(origin[1])
    draw.text((x, y), str(text), font=font, fill=(255, 255, 255), stroke_width=2, stroke_fill=(0, 0, 0))
    image[:] = cv2.cvtColor(np.asarray(pil_image), cv2.COLOR_RGB2BGR)


def _find_cjk_font():
    candidates = [
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
        "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
        "/usr/share/fonts/truetype/arphic/uming.ttc",
    ]
    for path in candidates:
        if Path(path).exists():
            return path
    return None


def _write_spf_log(log_dir: Path, image, command: str, request_payload: Dict[str, Any], image_jpeg_b64: str, action: Dict[str, Any], started_at: float) -> None:
    import cv2

    cv2.imwrite(str(log_dir / "input.jpg"), image)
    cv2.imwrite(str(log_dir / "annotated.jpg"), _draw_annotation(image, action, command))
    _write_json(log_dir / "request.json", _safe_payload_for_log(request_payload, image_jpeg_b64))
    _write_json(
        log_dir / "response.json",
        {
            "ok": True,
            "action": action,
            "raw_vlm_response": _LAST_VLM_RESPONSE_TEXT,
        },
    )
    _write_json(
        log_dir / "meta.json",
        {
            "created_at": time.time(),
            "duration_sec": max(0.0, time.time() - started_at),
            "image_shape": list(image.shape),
            "worker": _health_payload(),
        },
    )


def _write_spf_error_log(log_dir: Path, image, command: str, request_payload: Dict[str, Any], image_jpeg_b64: str, error: str, started_at: float) -> None:
    import cv2

    cv2.imwrite(str(log_dir / "input.jpg"), image)
    _write_json(log_dir / "request.json", _safe_payload_for_log(request_payload, image_jpeg_b64))
    _write_json(
        log_dir / "response.json",
        {
            "ok": False,
            "error": error,
            "raw_vlm_response": _LAST_VLM_RESPONSE_TEXT,
        },
    )
    _write_json(
        log_dir / "meta.json",
        {
            "created_at": time.time(),
            "duration_sec": max(0.0, time.time() - started_at),
            "image_shape": list(image.shape),
            "worker": _health_payload(),
        },
    )


def infer_action(payload: Dict[str, Any]) -> Dict[str, Any]:
    started_at = time.time()
    command = str(payload.get("command") or "").strip()
    image_jpeg_b64 = str(payload.get("image_jpeg_b64") or "")
    if not command:
        raise WorkerError("missing command")
    if not image_jpeg_b64:
        raise WorkerError("missing image_jpeg_b64")

    image = _decode_image(image_jpeg_b64)
    log_dir = _new_log_dir() if _spf_logging_enabled() else None
    height, width = image.shape[:2]
    try:
        projector = _load_projector(width, height)
        actions = projector.get_vlm_points(image, command)
    except Exception as exc:
        if log_dir:
            _write_spf_error_log(log_dir, image, command, payload, image_jpeg_b64, "SPF inference failed: %s" % exc, started_at)
        raise WorkerError("SPF inference failed: %s" % exc)
    if not actions:
        if log_dir:
            _write_spf_error_log(log_dir, image, command, payload, image_jpeg_b64, "SPF returned no action", started_at)
        raise WorkerError("SPF returned no action")
    action = actions[0]
    if action is None:
        if log_dir:
            _write_spf_error_log(log_dir, image, command, payload, image_jpeg_b64, "SPF returned empty action", started_at)
        raise WorkerError("SPF returned empty action")

    label = getattr(action, "label", "")
    yaw_only = bool(getattr(action, "yaw_only", False))
    yaw_deg = None
    if yaw_only:
        import math

        yaw_deg = math.degrees(math.atan2(float(action.dx), float(action.dy)))

    result = {
        "dx": float(action.dx),
        "dy": float(action.dy),
        "dz": float(action.dz),
        "yaw_only": yaw_only,
        "yaw_deg": yaw_deg,
        "label": label,
        "screen_x": float(getattr(action, "screen_x", 0.0)),
        "screen_y": float(getattr(action, "screen_y", 0.0)),
    }
    if log_dir:
        _write_spf_log(log_dir, image, command, payload, image_jpeg_b64, result, started_at)
        result["log_dir"] = str(log_dir)
    return result


class Handler(BaseHTTPRequestHandler):
    server_version = "GameUAVSPFWorker/0.1"

    def do_GET(self):
        parsed_path = urllib.parse.urlparse(self.path).path
        if parsed_path == "/health":
            self._send_json(200, _health_payload())
            return
        if parsed_path == "/logs/latest":
            try:
                self._send_json(200, _latest_log_payload())
            except WorkerError as exc:
                self._send_json(503, {"ok": False, "error": str(exc)})
            return
        if parsed_path.startswith("/logs/"):
            try:
                path = _safe_log_file(self.path)
            except WorkerError as exc:
                self._send_json(400, {"ok": False, "error": str(exc)})
                return
            if not path.is_file():
                self._send_json(404, {"ok": False, "error": "log file not found"})
                return
            self._send_file(path)
            return
        self._send_json(404, {"ok": False, "error": "not found"})

    def do_POST(self):
        if self.path != "/infer":
            self._send_json(404, {"ok": False, "error": "not found"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            self._send_json(400, {"ok": False, "error": "invalid content length"})
            return
        try:
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
            action = infer_action(payload)
            self._send_json(200, {"ok": True, "action": action})
        except WorkerError as exc:
            self._send_json(503, {"ok": False, "error": str(exc)})
        except Exception as exc:
            self._send_json(500, {"ok": False, "error": str(exc)})

    def log_message(self, fmt, *args):
        print("%s - %s" % (self.address_string(), fmt % args))

    def _send_json(self, status: int, payload: Dict[str, Any]):
        data = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _send_file(self, path: Path):
        data = path.read_bytes()
        content_type = "image/jpeg" if path.suffix.lower() == ".jpg" else "application/json"
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)


def main():
    parser = argparse.ArgumentParser(description="GameUAV SPF worker")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9310)
    args = parser.parse_args()

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print("spf_worker listening on %s:%d" % (args.host, args.port))
    server.serve_forever()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import json
import os
import shlex
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path


class CameraCaptureError(RuntimeError):
    pass


class CameraStreamError(RuntimeError):
    pass


DEFAULT_CAMERAS = {
    "rgb": {
        "name": "RGB0",
        "topic": "/camera/color/image_raw",
        "enabled": True,
        "mode": "video",
        "quality": 45,
    },
    "rgb1": {
        "name": "RGB1",
        "topic": "/rgb1/image_raw",
        "enabled": True,
        "mode": "video",
        "quality": 45,
    },
    "gray": {
        "name": "灰度",
        "topic": "/camera/infra1/image_rect_raw",
        "enabled": False,
        "mode": "video",
        "quality": 45,
    },
    "depth": {
        "name": "深度",
        "topic": "/camera/depth/image_rect_raw",
        "enabled": False,
        "mode": "video",
        "quality": 45,
    },
}

CAMERA_ALIASES = {
    "usb_rgb": "rgb1",
}

MAX_STREAM_FPS = 12.0
MAX_STREAM_QUALITY = 70
SUBSCRIBER_STALE_SEC = 3.0
MUTABLE_CAMERA_SETTING_KEYS = ("enabled", "mode", "quality")


def canonical_camera_id(camera_id):
    return CAMERA_ALIASES.get(camera_id, camera_id)


def _sanitize_camera_settings(settings):
    sanitized = {}
    if "enabled" in settings:
        sanitized["enabled"] = bool(settings["enabled"])
    if settings.get("mode") in ("photo", "video"):
        sanitized["mode"] = settings["mode"]
    if "quality" in settings:
        try:
            sanitized["quality"] = max(1, min(100, int(settings["quality"])))
        except (TypeError, ValueError):
            pass
    return sanitized


def _serializable_camera_settings(settings):
    return {
        key: settings[key]
        for key in MUTABLE_CAMERA_SETTING_KEYS
        if key in settings
    }


@dataclass
class RemoteCameraManager:
    base_url: str
    cameras: dict = field(default_factory=lambda: {key: dict(value) for key, value in DEFAULT_CAMERAS.items()})
    timeout: float = 5.0

    def __post_init__(self):
        self.base_url = self.base_url.rstrip("/")

    def list_settings(self):
        try:
            payload = self._request_json("/api/cameras/settings")
            cameras = payload.get("cameras") or {}
            if cameras:
                self.cameras = {key: dict(value) for key, value in cameras.items()}
        except CameraCaptureError:
            pass
        return {key: dict(value) for key, value in self.cameras.items()}

    def stream_stats(self):
        payload = self._request_json("/api/cameras/stats")
        return payload.get("streams") or {}

    def update_settings(self, camera_id, update):
        camera_id = canonical_camera_id(camera_id)
        if camera_id not in self.cameras:
            self.list_settings()
        if camera_id not in self.cameras:
            raise KeyError(camera_id)
        payload = self._request_json(
            "/api/cameras/%s/settings" % urllib.parse.quote(camera_id, safe=""),
            method="POST",
            body=json.dumps(update).encode("utf-8"),
            headers={"Content-Type": "application/json"},
        )
        settings = payload.get("settings") or {}
        if settings:
            self.cameras[camera_id] = dict(settings)
        return dict(self.cameras[camera_id])

    def capture_snapshot(self, camera_id, quality=None):
        camera_id = canonical_camera_id(camera_id)
        if camera_id not in self.cameras:
            self.list_settings()
        if camera_id not in self.cameras:
            raise KeyError(camera_id)
        query = {}
        if quality is not None:
            query["quality"] = str(quality)
        path = "/api/cameras/%s/snapshot" % urllib.parse.quote(camera_id, safe="")
        if query:
            path += "?" + urllib.parse.urlencode(query)
        payload = self._request_json(path)
        payload["camera_id"] = camera_id
        return payload

    def stream_mjpeg(self, camera_id, quality=None, fps=5.0, stream_owner=None):
        camera_id = canonical_camera_id(camera_id)
        if camera_id not in self.cameras:
            self.list_settings()
        if camera_id not in self.cameras:
            raise KeyError(camera_id)
        query = {
            "quality": str(quality if quality is not None else self.cameras[camera_id].get("quality", 60)),
            "fps": str(fps),
        }
        path = "/api/cameras/%s/stream?%s" % (
            urllib.parse.quote(camera_id, safe=""),
            urllib.parse.urlencode(query),
        )
        url = self.base_url + path

        def generate():
            request = urllib.request.Request(url, headers={"Accept": "multipart/x-mixed-replace"})
            try:
                with urllib.request.urlopen(request, timeout=self.timeout) as response:
                    while True:
                        chunk = response.read(65536)
                        if not chunk:
                            break
                        yield chunk
            except (urllib.error.URLError, TimeoutError, OSError) as exc:
                raise CameraStreamError("remote camera stream failed: %s" % exc)

        return generate()

    def _request_json(self, path, method="GET", body=None, headers=None):
        url = self.base_url + path
        request = urllib.request.Request(url, data=body, method=method, headers=headers or {})
        try:
            with urllib.request.urlopen(request, timeout=self.timeout) as response:
                data = response.read()
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace") or str(exc)
            if exc.code == 404:
                raise KeyError(detail)
            raise CameraCaptureError("remote camera request failed: %s" % detail)
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            raise CameraCaptureError("remote camera request failed: %s" % exc)
        try:
            return json.loads(data.decode("utf-8"))
        except json.JSONDecodeError as exc:
            raise CameraCaptureError("invalid remote camera response: %s" % exc)


@dataclass
class CameraManager:
    workspace_root: str
    ros_setup: str = "/opt/ros/noetic/setup.bash"
    workspace_setup: str = "devel/setup.bash"
    cameras: dict = field(default_factory=lambda: {key: dict(value) for key, value in DEFAULT_CAMERAS.items()})
    settings_path: str = None
    timeout: float = 3.0
    _settings_lock: threading.Lock = field(default_factory=threading.Lock, init=False, repr=False)
    _ros_lock: threading.Lock = field(default_factory=threading.Lock, init=False, repr=False)
    _frame_condition: threading.Condition = field(default_factory=threading.Condition, init=False, repr=False)
    _ros_initialized: bool = field(default=False, init=False)
    _ros_error: str = field(default="", init=False)
    _subscribers: dict = field(default_factory=dict, init=False, repr=False)
    _frames: dict = field(default_factory=dict, init=False, repr=False)
    _stream_lock: threading.Lock = field(default_factory=threading.Lock, init=False, repr=False)
    _stream_stats: dict = field(default_factory=dict, init=False, repr=False)
    _stream_generations: dict = field(default_factory=dict, init=False, repr=False)

    def __post_init__(self):
        self.workspace_root = str(Path(self.workspace_root).resolve())
        if self.settings_path is None:
            self.settings_path = str(Path(self.workspace_root) / "runtime" / "camera_settings.json")
        self._load_settings()

    def list_settings(self):
        return {key: dict(value) for key, value in self.cameras.items()}

    def stream_stats(self):
        now = time.time()
        with self._stream_lock:
            stats = {key: dict(value) for key, value in self._stream_stats.items()}
        for camera_id, item in stats.items():
            item["camera_id"] = camera_id
            item["age_sec"] = max(0.0, now - item.get("updated_at", now))
        return stats

    def update_settings(self, camera_id, update):
        camera_id = canonical_camera_id(camera_id)
        if camera_id not in self.cameras:
            raise KeyError(camera_id)
        current = dict(self.cameras[camera_id])
        current.update(_sanitize_camera_settings(update))
        self.cameras[camera_id] = current
        self._save_settings()
        return dict(current)

    def _load_settings(self):
        path = Path(self.settings_path)
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return
        except (OSError, json.JSONDecodeError):
            return
        cameras = payload.get("cameras") if isinstance(payload, dict) else None
        if not isinstance(cameras, dict):
            return
        for camera_id, update in cameras.items():
            camera_id = canonical_camera_id(camera_id)
            if camera_id not in self.cameras or not isinstance(update, dict):
                continue
            self.cameras[camera_id].update(_sanitize_camera_settings(update))

    def _save_settings(self):
        path = Path(self.settings_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "version": 1,
            "saved_at": time.time(),
            "cameras": {
                camera_id: _serializable_camera_settings(settings)
                for camera_id, settings in self.cameras.items()
            },
        }
        tmp_path = path.with_name(path.name + ".tmp")
        with self._settings_lock:
            tmp_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True), encoding="utf-8")
            os.replace(str(tmp_path), str(path))

    def capture_snapshot(self, camera_id, quality=None):
        camera_id = canonical_camera_id(camera_id)
        if camera_id not in self.cameras:
            raise KeyError(camera_id)
        camera = self.cameras[camera_id]
        topic = camera["topic"]
        quality = int(quality if quality is not None else camera.get("quality", 60))
        quality = max(1, min(100, quality))
        live = self._capture_live_snapshot(camera_id, topic, quality)
        if live is not None:
            return live

        script = Path(self.workspace_root) / "tools" / "capture_ros_image.py"
        command = [
            sys.executable,
            str(script),
            "--topic",
            topic,
            "--quality",
            str(quality),
            "--timeout",
            str(self.timeout),
        ]
        try:
            proc = subprocess.run(
                ["/bin/bash", "-lc", self._shell_command(command)],
                cwd=self.workspace_root,
                env=self._build_env(),
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=self.timeout + 2.0,
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            output = exc.stdout.decode("utf-8", errors="replace") if isinstance(exc.stdout, bytes) else exc.stdout
            detail = output.strip() if output else "timed out waiting for camera frame"
            raise CameraCaptureError(detail)
        if proc.returncode != 0:
            raise CameraCaptureError(proc.stdout.strip() or "camera capture failed")
        try:
            payload = json.loads(proc.stdout)
        except json.JSONDecodeError as exc:
            raise CameraCaptureError("invalid camera capture response: %s" % exc)
        if not payload.get("ok"):
            raise CameraCaptureError(payload.get("detail", "camera capture failed"))
        payload["camera_id"] = camera_id
        payload["topic"] = topic
        return payload

    def stream_mjpeg(self, camera_id, quality=None, fps=5.0, stream_owner=None):
        camera_id = canonical_camera_id(camera_id)
        if camera_id not in self.cameras:
            raise KeyError(camera_id)
        camera = self.cameras[camera_id]
        topic = camera["topic"]
        quality = int(quality if quality is not None else camera.get("quality", 60))
        quality = max(10, min(MAX_STREAM_QUALITY, quality))
        fps = max(0.5, min(MAX_STREAM_FPS, float(fps or 5.0)))
        period = 1.0 / fps
        boundary = b"--frame\r\n"
        stream_key = stream_owner or "camera:%s" % camera_id
        stream_generation = self._next_stream_generation(stream_key)

        from tools.capture_ros_image import image_to_encoded_image

        def generate():
            last_seq = None
            while True:
                if not self._is_current_stream(stream_key, stream_generation):
                    return
                started = time.monotonic()
                frame = self._wait_for_new_frame(
                    camera_id,
                    topic,
                    last_seq=last_seq,
                    timeout=max(0.2, period * 2.0),
                )
                if frame is None:
                    time.sleep(max(0.01, period))
                    continue
                last_seq = frame["seq"]
                message = frame["message"]
                if message is not None:
                    try:
                        image = image_to_encoded_image(message, quality)
                    except Exception as exc:
                        raise CameraStreamError("failed to encode camera frame: %s" % exc)
                    if image["mime_type"] == "image/jpeg":
                        self._record_stream_frame(
                            camera_id,
                            len(image["data"]),
                            quality,
                            fps,
                            source_info=self._message_info(message),
                        )
                        headers = (
                            boundary
                            + b"Content-Type: image/jpeg\r\n"
                            + b"Content-Length: "
                            + str(len(image["data"])).encode("ascii")
                            + b"\r\n\r\n"
                        )
                        yield headers + image["data"] + b"\r\n"

                elapsed = time.monotonic() - started
                time.sleep(max(0.01, period - elapsed))

        return generate()

    def _next_stream_generation(self, stream_key):
        with self._stream_lock:
            generation = self._stream_generations.get(stream_key, 0) + 1
            self._stream_generations[stream_key] = generation
            return generation

    def _is_current_stream(self, stream_key, generation):
        with self._stream_lock:
            return self._stream_generations.get(stream_key) == generation

    def _record_stream_frame(self, camera_id, frame_bytes, quality, target_fps, source_info=None):
        now = time.time()
        with self._stream_lock:
            stats = self._stream_stats.get(camera_id)
            if not stats:
                stats = {
                    "window_started_at": now,
                    "window_frames": 0,
                    "fps": 0.0,
                    "frame_count": 0,
                }
            stats["window_frames"] += 1
            stats["frame_count"] += 1
            elapsed = max(0.001, now - stats["window_started_at"])
            if elapsed >= 1.0:
                stats["fps"] = stats["window_frames"] / elapsed
                stats["window_frames"] = 0
                stats["window_started_at"] = now
            stats["updated_at"] = now
            stats["frame_bytes"] = int(frame_bytes)
            stats["quality"] = int(quality)
            stats["target_fps"] = float(target_fps)
            if source_info:
                stats.update(source_info)
            self._stream_stats[camera_id] = stats

    def _capture_live_snapshot(self, camera_id, topic, quality):
        try:
            frame = self._wait_for_new_frame(camera_id, topic, timeout=max(0.2, self.timeout))
        except CameraCaptureError:
            return None
        if frame is None:
            return None
        message = frame["message"]

        from tools.capture_ros_image import image_to_payload

        payload = image_to_payload(message, quality)
        payload["ok"] = True
        payload["camera_id"] = camera_id
        payload["topic"] = topic
        payload["source_topic"] = topic
        payload["cached"] = False
        payload.update(self._message_info(message))
        payload["cache_seq"] = int(frame["seq"])
        return payload

    def _capture_cached_snapshot(self, camera_id, topic, quality):
        try:
            self._ensure_ros_subscriber(camera_id, topic)
        except CameraCaptureError:
            return None

        frame = self._latest_frame(camera_id)
        if frame is None:
            frame = self._wait_for_new_frame(camera_id, topic, timeout=min(0.6, max(0.1, self.timeout)))
        if frame is None:
            return None
        message = frame["message"]

        from tools.capture_ros_image import image_to_payload

        payload = image_to_payload(message, quality)
        payload["ok"] = True
        payload["camera_id"] = camera_id
        payload["topic"] = topic
        payload["source_topic"] = topic
        payload["cached"] = True
        payload["cache_seq"] = int(frame["seq"])
        return payload

    def _latest_frame(self, camera_id):
        with self._frame_condition:
            frame = self._frames.get(camera_id)
            return dict(frame) if frame else None

    def _wait_for_new_frame(self, camera_id, topic, last_seq=None, timeout=0.5):
        self._ensure_ros_subscriber(camera_id, topic)
        deadline = time.monotonic() + max(0.05, float(timeout))
        with self._frame_condition:
            while True:
                frame = self._frames.get(camera_id)
                if frame and (last_seq is None or frame.get("seq") != last_seq):
                    return dict(frame)
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._frame_condition.wait(timeout=remaining)

    def _message_info(self, message):
        header = getattr(message, "header", None)
        stamp = getattr(header, "stamp", None)
        stamp_sec = None
        if stamp is not None:
            try:
                stamp_sec = float(stamp.to_sec())
            except Exception:
                stamp_sec = None
        info = {
            "source_seq": int(getattr(header, "seq", 0) or 0) if header is not None else 0,
            "source_received_at": time.time(),
        }
        if stamp_sec is not None:
            info["source_stamp"] = stamp_sec
            info["source_age_sec"] = max(0.0, time.time() - stamp_sec)
        return info

    def _ensure_ros_subscriber(self, camera_id, topic):
        existing = self._subscribers.get(camera_id)
        if existing and existing.get("topic") == topic and not self._subscriber_stale(camera_id):
            return
        with self._ros_lock:
            existing = self._subscribers.get(camera_id)
            if existing and existing.get("topic") == topic and not self._subscriber_stale(camera_id):
                return
            if existing and existing.get("subscriber") is not None:
                try:
                    existing["subscriber"].unregister()
                except Exception:
                    pass
            with self._frame_condition:
                self._frames.pop(camera_id, None)
            rospy, Image = self._ensure_ros()

            def callback(message):
                with self._frame_condition:
                    previous = self._frames.get(camera_id) or {}
                    self._frames[camera_id] = {
                        "message": message,
                        "received_at": time.time(),
                        "seq": int(previous.get("seq", 0)) + 1,
                    }
                    self._frame_condition.notify_all()

            subscriber = rospy.Subscriber(topic, Image, callback, queue_size=1)
            self._subscribers[camera_id] = {
                "topic": topic,
                "subscriber": subscriber,
            }

    def _subscriber_stale(self, camera_id):
        with self._frame_condition:
            frame = self._frames.get(camera_id)
            if not frame:
                return False
            try:
                age = time.time() - float(frame.get("received_at"))
            except (TypeError, ValueError):
                return True
            return age > SUBSCRIBER_STALE_SEC

    def _ensure_ros(self):
        if self._ros_initialized:
            import rospy
            from sensor_msgs.msg import Image

            return rospy, Image
        if self._ros_error:
            raise CameraCaptureError(self._ros_error)

        ros_python = "/opt/ros/noetic/lib/python3/dist-packages"
        if ros_python not in sys.path and Path(ros_python).exists():
            sys.path.append(ros_python)
        workspace_python = Path(self.workspace_root) / "devel" / "lib" / "python3" / "dist-packages"
        if str(workspace_python) not in sys.path and workspace_python.exists():
            sys.path.append(str(workspace_python))

        try:
            import rospy
            from sensor_msgs.msg import Image

            os.environ["ROS_MASTER_URI"] = "http://localhost:11311"
            if not rospy.core.is_initialized():
                rospy.init_node("gameuav_camera_stream_cache", anonymous=True, disable_signals=True)
        except Exception as exc:
            self._ros_error = "failed to initialize ROS camera cache: %s" % exc
            raise CameraCaptureError(self._ros_error)

        self._ros_initialized = True
        return rospy, Image

    def _build_env(self):
        env = os.environ.copy()
        env["ROS_MASTER_URI"] = "http://localhost:11311"
        return env

    def _shell_command(self, command):
        workspace_setup = Path(self.workspace_root) / self.workspace_setup
        parts = ["set -e"]
        if self.ros_setup:
            parts.append("source %s" % shlex.quote(self.ros_setup))
        if workspace_setup.exists():
            parts.append("source %s" % shlex.quote(str(workspace_setup)))
        parts.append("exec " + " ".join(shlex.quote(str(part)) for part in command))
        return " && ".join(parts)

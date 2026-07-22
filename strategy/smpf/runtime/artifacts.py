"""Per-task SMPF images, model responses, and automatically updated summaries."""

import json
import re
from pathlib import Path
import threading
import time

import cv2
import numpy as np

from .experiment_log import redact_sensitive


_SAFE_NAME = re.compile(r"[^a-zA-Z0-9_.-]+")


def _safe_name(value, fallback):
    name = _SAFE_NAME.sub("_", str(value or "").strip()).strip("._")
    return name or fallback


def colorize_depth_image(depth):
    """Convert aligned metric or uint16 depth into an inspectable BGR image."""
    values = np.asarray(depth)
    if values.ndim != 2 or values.size == 0:
        raise ValueError("depth input must be a non-empty H x W array")
    metric = values.astype(np.float32)
    valid = np.isfinite(metric) & (metric > 0.0)
    result = np.zeros(values.shape + (3,), dtype=np.uint8)
    if not np.any(valid):
        return result
    valid_values = metric[valid]
    near = float(np.percentile(valid_values, 2.0))
    far = float(np.percentile(valid_values, 98.0))
    if far <= near:
        far = near + 1.0
    normalized = np.zeros(values.shape, dtype=np.uint8)
    normalized[valid] = np.clip(
        (far - metric[valid]) * 255.0 / (far - near),
        0.0,
        255.0,
    ).astype(np.uint8)
    result = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
    result[~valid] = 0
    return result


class SmpfArtifactWriter:
    """Write inspectable artifacts under one directory per task ID."""

    def __init__(self, root):
        self.root = Path(root)
        self._lock = threading.RLock()
        self._summaries = {}
        self._counters = {}

    def _task_dir(self, task_id):
        return self.root / _safe_name(task_id, "unknown-task")

    def task_directory(self, task_id):
        return self._task_dir(task_id)

    def _next_index(self, task_id, kind):
        key = (str(task_id), str(kind))
        self._counters[key] = self._counters.get(key, 0) + 1
        return self._counters[key]

    @staticmethod
    def _write_json(path, value):
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(redact_sensitive(value), ensure_ascii=False, indent=2, sort_keys=True)
            + "\n",
            encoding="utf-8",
        )
        temporary.replace(path)

    def _summary(self, task_id):
        task_key = str(task_id)
        summary = self._summaries.get(task_key)
        if summary is None:
            summary = {
                "schema": "gameuav.smpf.trial_summary.v1",
                "task_id": task_key,
                "method": "smpf",
                "event_count": 0,
                "timeline": [],
                "artifacts": {"images": [], "depth": [], "geometry": [], "responses": []},
            }
            self._summaries[task_key] = summary
        return summary

    def _flush_summary(self, task_id):
        summary = self._summary(task_id)
        self._write_json(self._task_dir(task_id) / "summary.json", summary)

    def record_event(self, record):
        task_id = record.get("task_id")
        if task_id is None:
            return
        with self._lock:
            summary = self._summary(task_id)
            timestamp = record.get("timestamp", time.time())
            summary["updated_at"] = timestamp
            summary["event_count"] += 1
            timeline_item = {
                key: record[key]
                for key in ("event", "timestamp", "state", "success", "reason", "waypoint_index")
                if key in record
            }
            summary["timeline"].append(timeline_item)
            event = record.get("event")
            if event == "task_received":
                summary.update(
                    {
                        "started_at": timestamp,
                        "mode": record.get("mode"),
                        "instruction": record.get("instruction"),
                        "max_cycles": record.get("max_cycles"),
                        "models": record.get("models"),
                        "llm_reasoning_effort": record.get("llm_reasoning_effort"),
                        "execution_requested": record.get("execution_gate_open"),
                    }
                )
            elif event == "plan_verified":
                excluded = {"schema", "method", "event", "task_id", "timestamp"}
                summary["plan"] = {
                    key: value for key, value in record.items() if key not in excluded
                }
            elif event == "goal_published":
                summary.setdefault("published_goals", []).append(
                    {
                        key: record.get(key)
                        for key in (
                            "timestamp",
                            "waypoint_index",
                            "goal_world_m",
                            "target_facing_yaw_rad",
                            "metric_frame_age_at_publish_sec",
                        )
                    }
                )
            elif event in {"terminal", "cycle_error"}:
                summary["finished_at"] = timestamp
                if summary.get("started_at") is not None:
                    summary["elapsed_sec"] = max(0.0, timestamp - summary["started_at"])
                summary["outcome"] = {
                    "event": event,
                    "state": record.get("state", "ERROR" if event == "cycle_error" else None),
                    "success": record.get("success", False if event == "cycle_error" else None),
                    "reason": record.get("reason"),
                    "error_type": record.get("error_type"),
                }
            self._flush_summary(task_id)

    def write_response(self, task_id, kind, response):
        with self._lock:
            safe_kind = _safe_name(kind, "model")
            index = self._next_index(task_id, "response_" + safe_kind)
            relative = Path("responses") / ("%03d_%s.json" % (index, safe_kind))
            payload = {
                "schema": "gameuav.smpf.model_response.v1",
                "task_id": str(task_id),
                "kind": safe_kind,
                "captured_at": time.time(),
                "response": response,
            }
            self._write_json(self._task_dir(task_id) / relative, payload)
            summary = self._summary(task_id)
            summary["artifacts"]["responses"].append(str(relative))
            self._flush_summary(task_id)
            return self._task_dir(task_id) / relative

    def write_annotated_image(self, task_id, phase, image, annotations):
        annotated = np.asarray(image).copy()
        if annotated.ndim != 3 or annotated.shape[2] != 3 or annotated.size == 0:
            raise ValueError("annotated image input must be a non-empty BGR image")
        height, width = annotated.shape[:2]
        occupied_labels = []
        for annotation in annotations:
            ymin, xmin, ymax, xmax = annotation["bbox_yxyx"]
            x1 = max(0, min(width - 1, int(round(xmin))))
            y1 = max(0, min(height - 1, int(round(ymin))))
            x2 = max(0, min(width - 1, int(round(xmax))))
            y2 = max(0, min(height - 1, int(round(ymax))))
            color = tuple(int(value) for value in annotation.get("color_bgr", (0, 255, 0)))
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            label = str(annotation.get("label") or "object")
            (text_width, text_height), baseline = cv2.getTextSize(
                label,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                1,
            )
            text_x = max(2, min(x1, width - text_width - 4))
            text_y = max(text_height + 4, y1 - 6)
            while any(
                text_x < right
                and text_x + text_width + 4 > left
                and abs(text_y - previous_y) < text_height + baseline + 6
                for left, right, previous_y in occupied_labels
            ):
                text_y += text_height + baseline + 6
            if text_y + baseline + 2 >= height:
                text_y = max(text_height + 4, y1 - text_height - baseline - 8)
            occupied_labels.append((text_x, text_x + text_width + 4, text_y))
            cv2.rectangle(
                annotated,
                (text_x - 2, text_y - text_height - 3),
                (text_x + text_width + 2, text_y + baseline + 2),
                (0, 0, 0),
                -1,
            )
            cv2.putText(
                annotated,
                label,
                (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
                cv2.LINE_AA,
            )
            centroid = annotation.get("centroid_uv")
            if centroid is not None:
                cv2.circle(
                    annotated,
                    (int(round(centroid[0])), int(round(centroid[1]))),
                    4,
                    color,
                    -1,
                )
        with self._lock:
            safe_phase = _safe_name(phase, "perception")
            index = self._next_index(task_id, "image")
            relative = Path("images") / ("%03d_%s.jpg" % (index, safe_phase))
            path = self._task_dir(task_id) / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(path), annotated, [cv2.IMWRITE_JPEG_QUALITY, 92]):
                raise RuntimeError("failed to write SMPF annotated image")
            summary = self._summary(task_id)
            summary["artifacts"]["images"].append(str(relative))
            self._flush_summary(task_id)
        return annotated, path

    def write_depth_image(self, task_id, phase, depth, annotations):
        values = np.asarray(depth)
        colorized = colorize_depth_image(values)
        annotated, image_path = self.write_annotated_image(
            task_id,
            "depth_" + str(phase),
            colorized,
            annotations,
        )
        with self._lock:
            safe_phase = _safe_name(phase, "aligned")
            index = self._next_index(task_id, "depth_raw")
            relative = Path("depth") / ("%03d_%s.npy" % (index, safe_phase))
            path = self._task_dir(task_id) / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            np.save(str(path), values, allow_pickle=False)
            summary = self._summary(task_id)
            summary["artifacts"]["depth"].append(
                {"raw": str(relative), "visualization": str(image_path.relative_to(self._task_dir(task_id)))}
            )
            self._flush_summary(task_id)
        return annotated, image_path, path

    def write_geometry(self, task_id, name, payload):
        with self._lock:
            safe_name = _safe_name(name, "spheres")
            index = self._next_index(task_id, "geometry")
            relative = Path("geometry") / ("%03d_%s.json" % (index, safe_name))
            self._write_json(self._task_dir(task_id) / relative, payload)
            summary = self._summary(task_id)
            summary["artifacts"]["geometry"].append(str(relative))
            self._flush_summary(task_id)
            return self._task_dir(task_id) / relative

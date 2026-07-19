"""Strict OpenAI-compatible visual target detector for SMPF."""

from dataclasses import dataclass
import base64
import json
import math
import os
from typing import Tuple

import cv2
import numpy as np
import requests

from .model_defaults import DEFAULT_VLM_MODEL


DETECTION_SCHEMA = "smpf.detection.v1"
SCENE_DETECTION_SCHEMA = "smpf.scene_detection.v1"


class VisionDetectorError(RuntimeError):
    """Base error for target-detection failures."""


class DetectionSchemaError(VisionDetectorError):
    """The VLM response does not match the target-detection schema."""


class DetectionTransportError(VisionDetectorError):
    """The VLM endpoint could not return a completion."""


@dataclass(frozen=True)
class Detection:
    label: str
    bbox_yxyx_1000: Tuple[int, int, int, int]
    confidence: float

    def pixel_bbox(self, image_shape):
        if len(image_shape) < 2:
            raise ValueError("image shape must include height and width")
        height, width = int(image_shape[0]), int(image_shape[1])
        ymin, xmin, ymax, xmax = self.bbox_yxyx_1000
        return (
            int(round(ymin * (height - 1) / 1000.0)),
            int(round(xmin * (width - 1) / 1000.0)),
            int(round(ymax * (height - 1) / 1000.0)),
            int(round(xmax * (width - 1) / 1000.0)),
        )


@dataclass(frozen=True)
class SceneDetection:
    target: object
    obstacles: Tuple[Detection, ...]

    def __post_init__(self):
        if self.target is not None and not isinstance(self.target, Detection):
            raise ValueError("scene target must be a Detection or None")
        object.__setattr__(self, "obstacles", tuple(self.obstacles))
        if any(not isinstance(item, Detection) for item in self.obstacles):
            raise ValueError("scene obstacles must contain only Detection values")


def _parse_detection_object(data, name, allow_empty=True):
    if not isinstance(data, dict):
        raise DetectionSchemaError("%s must be an object" % name)
    required = {"label", "bbox_yxyx_1000", "confidence"}
    if set(data) != required:
        raise DetectionSchemaError("%s must contain exactly %s" % (name, sorted(required)))
    label = data["label"]
    if not isinstance(label, str):
        raise DetectionSchemaError("%s.label must be text" % name)
    label = label.strip()
    raw_bbox = data["bbox_yxyx_1000"]
    confidence = data["confidence"]
    if isinstance(confidence, bool) or not isinstance(confidence, (int, float)):
        raise DetectionSchemaError("%s.confidence must be numeric" % name)
    confidence = float(confidence)
    if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
        raise DetectionSchemaError("%s.confidence must be in [0, 1]" % name)
    if raw_bbox == []:
        if not allow_empty or label or confidence != 0.0:
            raise DetectionSchemaError("%s empty detection requires empty label and zero confidence" % name)
        return None
    if not isinstance(raw_bbox, list) or len(raw_bbox) != 4:
        raise DetectionSchemaError("%s.bbox_yxyx_1000 must contain four integers" % name)
    if any(isinstance(value, bool) or not isinstance(value, int) for value in raw_bbox):
        raise DetectionSchemaError("%s.bbox_yxyx_1000 must contain integers" % name)
    ymin, xmin, ymax, xmax = raw_bbox
    if not all(0 <= value <= 1000 for value in raw_bbox):
        raise DetectionSchemaError("%s.bbox_yxyx_1000 values must be in [0, 1000]" % name)
    if ymax <= ymin or xmax <= xmin:
        raise DetectionSchemaError("%s.bbox_yxyx_1000 must have positive area" % name)
    if not label:
        raise DetectionSchemaError("%s non-empty detection requires a label" % name)
    return Detection(label, (ymin, xmin, ymax, xmax), confidence)


def parse_detection(content):
    if not isinstance(content, str) or not content.strip():
        raise DetectionSchemaError("detection output must be a non-empty JSON string")
    raw = content.strip()
    if raw.startswith("```") or raw.endswith("```"):
        raise DetectionSchemaError("detection output must not contain Markdown fences")
    try:
        data = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise DetectionSchemaError("detection output is not valid JSON") from exc
    if not isinstance(data, dict):
        raise DetectionSchemaError("detection root must be an object")
    required = {"schema", "label", "bbox_yxyx_1000", "confidence"}
    if set(data) != required:
        raise DetectionSchemaError("detection must contain exactly %s" % sorted(required))
    if data["schema"] != DETECTION_SCHEMA:
        raise DetectionSchemaError("detection schema must be %s" % DETECTION_SCHEMA)
    return _parse_detection_object(
        {key: data[key] for key in ("label", "bbox_yxyx_1000", "confidence")},
        "detection",
    )


def parse_scene_detection(content):
    if not isinstance(content, str) or not content.strip():
        raise DetectionSchemaError("scene detection output must be a non-empty JSON string")
    raw = content.strip()
    if raw.startswith("```") or raw.endswith("```"):
        raise DetectionSchemaError("scene detection output must not contain Markdown fences")
    try:
        data = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise DetectionSchemaError("scene detection output is not valid JSON") from exc
    if not isinstance(data, dict):
        raise DetectionSchemaError("scene detection root must be an object")
    required = {"schema", "target", "obstacles"}
    if set(data) != required:
        raise DetectionSchemaError("scene detection must contain exactly %s" % sorted(required))
    if data["schema"] != SCENE_DETECTION_SCHEMA:
        raise DetectionSchemaError("scene detection schema must be %s" % SCENE_DETECTION_SCHEMA)
    target = _parse_detection_object(data["target"], "target", allow_empty=True)
    raw_obstacles = data["obstacles"]
    if not isinstance(raw_obstacles, list) or len(raw_obstacles) > 8:
        raise DetectionSchemaError("scene obstacles must be a list with at most 8 items")
    obstacles = tuple(
        _parse_detection_object(item, "obstacles[%d]" % index, allow_empty=False)
        for index, item in enumerate(raw_obstacles)
    )
    return SceneDetection(target, obstacles)


def detection_prompt(instruction):
    return """Locate the single physical target object required by this UAV instruction:
{instruction}

Return raw JSON only with exactly these keys:
{{"schema":"smpf.detection.v1","label":"short English noun phrase","bbox_yxyx_1000":[ymin,xmin,ymax,xmax],"confidence":0.0}}

The box uses [ymin,xmin,ymax,xmax], integer coordinates normalized to [0,1000].
Cover the complete visible target. If it is not confidently visible, return exactly:
{{"schema":"smpf.detection.v1","label":"","bbox_yxyx_1000":[],"confidence":0.0}}
Do not infer an off-screen location and do not return Markdown.
""".format(instruction=str(instruction or "").strip())


def scene_detection_prompt(instruction):
    return """Ground the physical objects needed for this UAV instruction in the current image:
{instruction}

Identify exactly one destination target and visible, concrete obstacles that are
either explicitly named by the instruction or plausibly intersect the direct approach corridor
from the UAV/camera toward the target.
For reasoning instructions, infer the visible object that satisfies the need and use it as target.
Do not duplicate the target in obstacles. Do not list floor, ceiling, distant
background objects, or objects clearly outside the approach corridor. Walls and
doors are obstacles only when they bound or cross the approach corridor.
Do not infer off-screen locations. Use complete-object boxes in [ymin,xmin,ymax,xmax] integer coordinates normalized to [0,1000].

Return raw JSON only with exactly this structure:
{{"schema":"smpf.scene_detection.v1","target":{{"label":"short English noun phrase","bbox_yxyx_1000":[ymin,xmin,ymax,xmax],"confidence":0.0}},"obstacles":[{{"label":"cone","bbox_yxyx_1000":[ymin,xmin,ymax,xmax],"confidence":0.0}}]}}

If the target is not confidently visible, target must be exactly:
{{"label":"","bbox_yxyx_1000":[],"confidence":0.0}}
Obstacles must contain only non-empty detections and may be an empty list. Do not return Markdown.
""".format(instruction=str(instruction or "").strip())


class VisionDetectorClient:
    def __init__(
        self,
        api_key=None,
        base_url=None,
        model_id=None,
        timeout_sec=60.0,
        temperature=0.0,
        max_attempts=2,
        session=None,
    ):
        self.api_key = str(
            api_key
            or os.environ.get("SMPF_VLM_API_KEY")
            or os.environ.get("SMPF_LLM_API_KEY", "")
        ).strip()
        self.base_url = str(
            base_url
            or os.environ.get("SMPF_VLM_BASE_URL")
            or os.environ.get("SMPF_LLM_BASE_URL", "")
        ).strip()
        self.model_id = str(model_id or os.environ.get("SMPF_VLM_MODEL", DEFAULT_VLM_MODEL)).strip()
        self.timeout_sec = float(timeout_sec)
        self.temperature = float(temperature)
        self.max_attempts = int(max_attempts)
        self.last_attempts = 0
        if not self.api_key:
            raise ValueError("SMPF_VLM_API_KEY or SMPF_LLM_API_KEY is required")
        if not self.base_url:
            raise ValueError("SMPF_VLM_BASE_URL or SMPF_LLM_BASE_URL is required")
        if self.max_attempts < 1 or self.max_attempts > 3:
            raise ValueError("VLM max_attempts must be in [1, 3]")
        if "/chat/completions" not in self.base_url:
            self.base_url = self.base_url.rstrip("/") + "/chat/completions"
        self._session = session or requests.Session()

    def detect(self, bgr_image, instruction):
        return self._detect_with_retry(bgr_image, detection_prompt(instruction), parse_detection)

    def detect_scene(self, bgr_image, instruction):
        return self._detect_with_retry(
            bgr_image,
            scene_detection_prompt(instruction),
            parse_scene_detection,
        )

    def _detect_with_retry(self, bgr_image, prompt, parser):
        feedback = ""
        last_error = None
        self.last_attempts = 0
        for attempt in range(1, self.max_attempts + 1):
            self.last_attempts = attempt
            attempt_prompt = prompt + feedback
            content = self._complete(bgr_image, attempt_prompt)
            try:
                return parser(content)
            except DetectionSchemaError as exc:
                last_error = exc
                feedback = (
                    "\nCORRECTION: The previous response was rejected by the strict JSON parser: "
                    + str(exc)
                    + ". Return the requested raw JSON object only.\n"
                )
        raise last_error

    def _complete(self, bgr_image, prompt):
        image = np.asarray(bgr_image)
        if image.ndim != 3 or image.shape[2] != 3 or image.size == 0:
            raise ValueError("VLM input must be a non-empty BGR H x W x 3 image")
        if image.dtype != np.uint8:
            image = np.clip(image, 0, 255).astype(np.uint8)
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            raise VisionDetectorError("failed to encode VLM input image")
        image_url = "data:image/jpeg;base64,%s" % base64.b64encode(encoded.tobytes()).decode("ascii")
        payload = {
            "model": self.model_id,
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                        {"type": "image_url", "image_url": {"url": image_url}},
                    ],
                }
            ],
            "temperature": self.temperature,
            "response_format": {"type": "json_object"},
        }
        try:
            response = self._session.post(
                self.base_url,
                headers={
                    "Authorization": "Bearer %s" % self.api_key,
                    "Content-Type": "application/json",
                },
                json=payload,
                timeout=self.timeout_sec,
            )
        except requests.RequestException as exc:
            raise DetectionTransportError("VLM request failed: %s" % exc) from exc
        except Exception as exc:
            raise DetectionTransportError("VLM request failed: %s" % exc) from exc
        if int(response.status_code) != 200:
            detail = str(getattr(response, "text", ""))[:300]
            raise DetectionTransportError("VLM returned HTTP %s: %s" % (response.status_code, detail))
        try:
            data = response.json()
            content = data["choices"][0]["message"]["content"]
        except Exception as exc:
            raise DetectionTransportError("VLM response is missing choices[0].message.content") from exc
        if not isinstance(content, str):
            raise DetectionTransportError("VLM completion content must be text")
        return content

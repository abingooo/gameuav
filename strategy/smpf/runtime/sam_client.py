"""Typed client for the remote text-prompted SAM service used by SMPF."""

from dataclasses import dataclass
import base64
import math
import os
from pathlib import Path
from typing import Any, Mapping, Optional, Tuple

import cv2
import numpy as np
import requests


DEFAULT_SAM_HOST = "10.246.1.94"
DEFAULT_SAM_PORT = 5002
DEFAULT_SAM_TIMEOUT_SEC = 20.0


class SamClientError(RuntimeError):
    """Base error for SAM transport and response failures."""


class SamTransportError(SamClientError):
    """The remote service could not be reached or returned an HTTP error."""


class SamProtocolError(SamClientError):
    """The remote response does not satisfy the SMPF SAM contract."""


def _finite_pair(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise SamProtocolError("%s must be a two-value coordinate" % name)
    result = (float(value[0]), float(value[1]))
    if not all(math.isfinite(item) for item in result):
        raise SamProtocolError("%s must contain finite values" % name)
    return result


@dataclass(frozen=True)
class SamMask:
    """Geometry returned for one segmented object, in input-image pixels."""

    bounding_box_xyxy: Tuple[float, float, float, float]
    centroid_uv: Tuple[float, float]
    sample_points_uv: Tuple[Tuple[float, float], ...]
    area_px: float

    @property
    def bbox_yxyx(self):
        x1, y1, x2, y2 = self.bounding_box_xyxy
        return (y1, x1, y2, x2)


@dataclass(frozen=True)
class SamPrediction:
    """Validated SAM prediction; an empty mask tuple means target not detected."""

    prompt: str
    masks: Tuple[SamMask, ...]

    @property
    def mask_count(self):
        return len(self.masks)

    @property
    def best_mask(self):
        if not self.masks:
            return None
        return max(self.masks, key=lambda mask: mask.area_px)


class SamClient:
    """Call ``POST /predict`` without coupling the flight loop to raw JSON."""

    def __init__(self, host=None, port=None, timeout_sec=None, session=None):
        self.host = str(host or os.environ.get("SMPF_SAM_HOST", DEFAULT_SAM_HOST)).strip()
        if not self.host or "://" in self.host or "/" in self.host:
            raise ValueError("SAM host must be a hostname or IP address without a URL scheme")
        self.port = int(port if port is not None else os.environ.get("SMPF_SAM_PORT", DEFAULT_SAM_PORT))
        if self.port <= 0 or self.port > 65535:
            raise ValueError("SAM port must be in [1, 65535]")
        self.timeout_sec = float(
            timeout_sec
            if timeout_sec is not None
            else os.environ.get("SMPF_SAM_TIMEOUT_SEC", DEFAULT_SAM_TIMEOUT_SEC)
        )
        if not math.isfinite(self.timeout_sec) or self.timeout_sec <= 0.0:
            raise ValueError("SAM timeout must be finite and positive")
        self._session = session or requests.Session()

    @property
    def endpoint(self):
        return "http://%s:%d/predict" % (self.host, self.port)

    def predict(self, image_input, text_prompt, timeout_sec=None):
        prompt = str(text_prompt or "").strip()
        if not prompt:
            raise ValueError("SAM text prompt cannot be empty")
        request_timeout_sec = self.timeout_sec
        if timeout_sec is not None:
            timeout_sec = float(timeout_sec)
            if not math.isfinite(timeout_sec) or timeout_sec <= 0.0:
                raise ValueError("per-call SAM timeout must be finite and positive")
            request_timeout_sec = min(request_timeout_sec, timeout_sec)
        payload = {
            "image": base64.b64encode(self._encode_image(image_input)).decode("ascii"),
            "text": prompt,
        }
        try:
            response = self._session.post(
                self.endpoint,
                json=payload,
                timeout=request_timeout_sec,
            )
        except requests.RequestException as exc:
            raise SamTransportError("SAM request failed: %s" % exc) from exc
        except Exception as exc:
            raise SamTransportError("SAM request failed: %s" % exc) from exc

        if int(response.status_code) != 200:
            detail = str(getattr(response, "text", ""))[:300]
            raise SamTransportError("SAM returned HTTP %s: %s" % (response.status_code, detail))
        try:
            data = response.json()
        except Exception as exc:
            raise SamProtocolError("SAM response is not valid JSON") from exc
        return self.parse_prediction(data, prompt)

    @staticmethod
    def _encode_image(image_input):
        if isinstance(image_input, (str, Path)):
            path = Path(image_input)
            if not path.is_file():
                raise FileNotFoundError("image does not exist: %s" % path)
            return path.read_bytes()

        image = np.asarray(image_input)
        if image.ndim == 2:
            pass
        elif image.ndim == 3 and image.shape[2] in (3, 4):
            pass
        else:
            raise ValueError("image must be H x W, H x W x 3, or H x W x 4")
        if image.size == 0:
            raise ValueError("image cannot be empty")
        if image.dtype != np.uint8:
            if np.issubdtype(image.dtype, np.floating) and np.any(np.isfinite(image)):
                scale = 255.0 if float(np.nanmax(image)) <= 1.0 else 1.0
                image = np.nan_to_num(image, nan=0.0, posinf=255.0, neginf=0.0) * scale
            image = np.clip(image, 0, 255).astype(np.uint8)
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not ok:
            raise SamClientError("failed to encode SAM input image")
        return encoded.tobytes()

    @staticmethod
    def parse_prediction(data: Mapping[str, Any], prompt=""):
        if not isinstance(data, Mapping):
            raise SamProtocolError("SAM response root must be an object")
        mask_count = data.get("mask_count")
        if isinstance(mask_count, bool) or not isinstance(mask_count, int) or mask_count < 0:
            raise SamProtocolError("SAM mask_count must be a non-negative integer")
        raw_masks = data.get("masks")
        if not isinstance(raw_masks, list):
            raise SamProtocolError("SAM masks must be a list")
        if mask_count != len(raw_masks):
            raise SamProtocolError("SAM mask_count does not match masks length")

        masks = []
        for index, raw_mask in enumerate(raw_masks):
            if not isinstance(raw_mask, Mapping):
                raise SamProtocolError("SAM masks[%d] must be an object" % index)
            raw_box = raw_mask.get("bounding_box")
            if not isinstance(raw_box, Mapping):
                raise SamProtocolError("SAM masks[%d].bounding_box must be an object" % index)
            try:
                box = tuple(float(raw_box[key]) for key in ("x1", "y1", "x2", "y2"))
            except (KeyError, TypeError, ValueError) as exc:
                raise SamProtocolError(
                    "SAM masks[%d].bounding_box requires numeric x1/y1/x2/y2" % index
                ) from exc
            if not all(math.isfinite(value) for value in box):
                raise SamProtocolError("SAM masks[%d].bounding_box must be finite" % index)
            x1, y1, x2, y2 = box
            if x2 <= x1 or y2 <= y1:
                raise SamProtocolError("SAM masks[%d].bounding_box has no positive area" % index)

            centroid = _finite_pair(raw_mask.get("centroid"), "SAM masks[%d].centroid" % index)
            raw_points = raw_mask.get("random_points")
            if not isinstance(raw_points, list):
                raise SamProtocolError("SAM masks[%d].random_points must be a list" % index)
            points = tuple(
                _finite_pair(point, "SAM masks[%d].random_points" % index) for point in raw_points
            )
            raw_area = raw_mask.get("area", (x2 - x1) * (y2 - y1))
            try:
                area = float(raw_area)
            except (TypeError, ValueError) as exc:
                raise SamProtocolError("SAM masks[%d].area must be numeric" % index) from exc
            if not math.isfinite(area) or area <= 0.0:
                raise SamProtocolError("SAM masks[%d].area must be finite and positive" % index)
            masks.append(SamMask(box, centroid, points, area))

        return SamPrediction(str(prompt), tuple(masks))

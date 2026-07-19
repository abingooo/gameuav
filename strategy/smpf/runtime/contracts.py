"""Typed contracts shared by the SMPF perception and planning layers."""

from dataclasses import dataclass, field
import math
from typing import Tuple


Vector3 = Tuple[float, float, float]


def _finite_vector3(value, name):
    if len(value) != 3:
        raise ValueError("%s must contain exactly three values" % name)
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError("%s must contain finite values" % name)
    return result


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    def __post_init__(self):
        values = (self.fx, self.fy, self.cx, self.cy)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("camera intrinsics must be finite")
        if float(self.fx) <= 0.0 or float(self.fy) <= 0.0:
            raise ValueError("camera focal lengths must be positive")
        if int(self.width) <= 0 or int(self.height) <= 0:
            raise ValueError("camera dimensions must be positive")


@dataclass(frozen=True)
class DepthEstimate:
    value_m: float
    std_m: float
    sample_count: int
    minimum_m: float
    maximum_m: float

    def __post_init__(self):
        if not math.isfinite(float(self.value_m)) or float(self.value_m) <= 0.0:
            raise ValueError("depth estimate must be finite and positive")
        if not math.isfinite(float(self.std_m)) or float(self.std_m) < 0.0:
            raise ValueError("depth standard deviation must be finite and non-negative")
        if int(self.sample_count) <= 0:
            raise ValueError("depth estimate requires at least one sample")


@dataclass(frozen=True)
class ObjectSphere:
    label: str
    center: Vector3
    radius: float
    confidence: float = 1.0
    frame_id: str = "body_flu"
    source: str = ""

    def __post_init__(self):
        label = str(self.label or "").strip()
        if not label:
            raise ValueError("object sphere label cannot be empty")
        object.__setattr__(self, "label", label)
        object.__setattr__(self, "center", _finite_vector3(self.center, "center"))
        radius = float(self.radius)
        if not math.isfinite(radius) or radius <= 0.0:
            raise ValueError("object sphere radius must be finite and positive")
        object.__setattr__(self, "radius", radius)
        confidence = float(self.confidence)
        if not math.isfinite(confidence) or confidence < 0.0 or confidence > 1.0:
            raise ValueError("object sphere confidence must be in [0, 1]")
        object.__setattr__(self, "confidence", confidence)


@dataclass(frozen=True)
class ValidationIssue:
    kind: str
    index: int
    object_label: str = ""
    clearance_m: float = math.inf
    message: str = ""


@dataclass(frozen=True)
class ValidationResult:
    valid: bool
    issues: Tuple[ValidationIssue, ...] = field(default_factory=tuple)
    minimum_clearance_m: float = math.inf

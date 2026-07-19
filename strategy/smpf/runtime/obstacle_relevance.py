"""Deterministic 3-D flight-corridor relevance for semantic obstacles."""

from dataclasses import dataclass
import math
from typing import Iterable, Tuple

import numpy as np

from .contracts import ObjectSphere
from .geometry import point_to_segment_distance


@dataclass(frozen=True)
class CorridorObstacleAssessment:
    obstacle: ObjectSphere
    centerline_clearance_m: float
    relevant: bool


def _point3(value, name):
    point = np.asarray(value, dtype=float)
    if point.shape != (3,) or not np.all(np.isfinite(point)):
        raise ValueError("%s must be a finite three-vector" % name)
    return point


def assess_corridor_obstacles(
    start,
    corridor_goal,
    obstacles: Iterable[ObjectSphere],
    corridor_margin_m=0.25,
) -> Tuple[CorridorObstacleAssessment, ...]:
    """Classify spheres by surface clearance to a continuous approach segment."""
    start = _point3(start, "corridor start")
    goal = _point3(corridor_goal, "corridor goal")
    margin = float(corridor_margin_m)
    if not math.isfinite(margin) or margin < 0.0:
        raise ValueError("corridor margin must be finite and non-negative")
    result = []
    for obstacle in obstacles:
        clearance = (
            point_to_segment_distance(obstacle.center, start, goal) - obstacle.radius
        )
        result.append(
            CorridorObstacleAssessment(
                obstacle=obstacle,
                centerline_clearance_m=float(clearance),
                relevant=clearance <= margin,
            )
        )
    return tuple(result)

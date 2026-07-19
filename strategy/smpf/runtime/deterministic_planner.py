"""Deterministic sphere visibility-graph fallback for verified SMPF paths."""

from dataclasses import dataclass
import heapq
import math
from typing import Iterable, Mapping, Optional, Tuple

import numpy as np

from .contracts import ObjectSphere, ValidationResult
from .geometry import validate_polyline


class VisibilityGraphError(RuntimeError):
    """No validated path can be constructed inside the supplied model."""


@dataclass(frozen=True)
class VisibilityGraphPlan:
    guidepoints_m: Tuple[Tuple[float, float, float], ...]
    validation: ValidationResult
    candidate_count: int
    expanded_nodes: int
    path_length_m: float


def _point3(value, name):
    point = np.asarray(value, dtype=float)
    if point.shape != (3,) or not np.all(np.isfinite(point)):
        raise ValueError("%s must be a finite three-vector" % name)
    return point


def _bound(bounds, axis, side):
    if not bounds:
        return None
    for key in ("%s_%s" % (axis, side), "%s%s" % (side, axis), "%s_%s" % (side, axis)):
        if key in bounds:
            value = float(bounds[key])
            if not math.isfinite(value):
                raise ValueError("bounds must contain finite values")
            return value
    return None


def _inside_bounds(point, bounds):
    for index, axis in enumerate(("x", "y", "z")):
        minimum = _bound(bounds, axis, "min")
        maximum = _bound(bounds, axis, "max")
        if minimum is not None and point[index] < minimum:
            return False
        if maximum is not None and point[index] > maximum:
            return False
    return True


def _segment_clear(start, end, spheres, clearance_margin_m):
    return validate_polyline(
        (start, end),
        spheres,
        clearance_margin_m=clearance_margin_m,
    ).valid


def _sample_directions():
    directions = []
    for x in (-1.0, 0.0, 1.0):
        for y in (-1.0, 0.0, 1.0):
            for z in (-1.0, 0.0, 1.0):
                direction = np.asarray((x, y, z), dtype=float)
                norm = float(np.linalg.norm(direction))
                if norm > 0.0:
                    directions.append(direction / norm)
    return tuple(directions)


def approach_goal_for_sphere(sphere, clearance_margin_m=0.0, standoff_m=0.15):
    """Choose the nearest line-of-sight standoff point on the observer side."""
    center = _point3(sphere.center, "sphere.center")
    distance = float(np.linalg.norm(center))
    required = float(sphere.radius) + float(clearance_margin_m) + float(standoff_m)
    if required <= 0.0 or not math.isfinite(required):
        raise ValueError("approach clearance must be finite and positive")
    if distance <= required:
        raise VisibilityGraphError("current position is already inside the target approach distance")
    goal = center - center / distance * required
    return tuple(float(value) for value in goal)


def approach_goal_candidates_for_sphere(
    sphere,
    clearance_margin_m=0.0,
    standoff_m=0.15,
    bounds=None,
    max_candidates=27,
    allow_inside=False,
):
    """Sample bounded target-standoff goals, ordered from shortest to longest flight."""
    center = _point3(sphere.center, "sphere.center")
    distance = float(np.linalg.norm(center))
    required = float(sphere.radius) + float(clearance_margin_m) + float(standoff_m)
    if required <= 0.0 or not math.isfinite(required):
        raise ValueError("approach clearance must be finite and positive")
    if distance <= required and not bool(allow_inside):
        raise VisibilityGraphError("current position is already inside the target approach distance")
    limit = int(max_candidates)
    if limit < 1 or limit > 64:
        raise ValueError("max_candidates must be in [1, 64]")

    observer_side = (
        -center / distance
        if distance > 1e-9
        else np.asarray((-1.0, 0.0, 0.0), dtype=float)
    )
    sampled = sorted(
        _sample_directions(),
        key=lambda direction: (
            float(np.linalg.norm(center + direction * required)),
            -float(direction[2]),
            float(direction[1]),
            float(direction[0]),
        ),
    )
    candidates = []
    seen = set()
    for direction in (observer_side,) + tuple(sampled):
        goal = center + direction * required
        if not _inside_bounds(goal, bounds):
            continue
        key = tuple(round(float(value), 8) for value in goal)
        if key in seen:
            continue
        seen.add(key)
        candidates.append(tuple(float(value) for value in goal))
        if len(candidates) >= limit:
            break
    if not candidates:
        raise VisibilityGraphError("no target approach candidate is inside flight bounds")
    return tuple(candidates)


def _shortcut_path(path, spheres, clearance_margin_m):
    if len(path) <= 2:
        return list(path)
    result = [path[0]]
    index = 0
    while index < len(path) - 1:
        next_index = len(path) - 1
        while next_index > index + 1:
            if _segment_clear(path[index], path[next_index], spheres, clearance_margin_m):
                break
            next_index -= 1
        result.append(path[next_index])
        index = next_index
    return result


def plan_visibility_graph(
    start,
    goal,
    object_spheres: Iterable[ObjectSphere] = (),
    bounds: Optional[Mapping[str, float]] = None,
    clearance_margin_m=0.0,
    sample_padding_m=0.08,
    max_guidepoints=12,
):
    """Use deterministic surface samples and A* to find a verified 3-D path."""
    start = _point3(start, "start")
    goal = _point3(goal, "goal")
    spheres = tuple(object_spheres)
    margin = float(clearance_margin_m)
    padding = float(sample_padding_m)
    if not math.isfinite(margin) or margin < 0.0:
        raise ValueError("clearance margin must be finite and non-negative")
    if not math.isfinite(padding) or padding <= 0.0:
        raise ValueError("sample padding must be finite and positive")
    if not _inside_bounds(goal, bounds):
        raise VisibilityGraphError("fallback goal is outside flight bounds")
    endpoint_validation = validate_polyline((start, goal), spheres, clearance_margin_m=margin)
    endpoint_collisions = [
        issue for issue in endpoint_validation.issues if issue.kind == "point_collision"
    ]
    if any(issue.index == 0 for issue in endpoint_collisions):
        raise VisibilityGraphError("fallback start is inside an object safety sphere")
    if any(issue.index == 1 for issue in endpoint_collisions):
        raise VisibilityGraphError("fallback goal is inside an object safety sphere")

    nodes = [start, goal]
    for sphere in spheres:
        center = _point3(sphere.center, "sphere.center")
        sample_radius = float(sphere.radius) + margin + padding
        for direction in _sample_directions():
            candidate = center + direction * sample_radius
            if not _inside_bounds(candidate, bounds):
                continue
            if validate_polyline((candidate, candidate), spheres, clearance_margin_m=margin).valid:
                nodes.append(candidate)

    unique_nodes = []
    seen = set()
    for node in nodes:
        key = tuple(round(float(value), 8) for value in node)
        if key not in seen:
            seen.add(key)
            unique_nodes.append(node)
    nodes = unique_nodes

    adjacency = [[] for _node in nodes]
    for left in range(len(nodes)):
        for right in range(left + 1, len(nodes)):
            if not _segment_clear(nodes[left], nodes[right], spheres, margin):
                continue
            distance = float(np.linalg.norm(nodes[right] - nodes[left]))
            adjacency[left].append((right, distance))
            adjacency[right].append((left, distance))

    frontier = [(float(np.linalg.norm(goal - start)), 0.0, 0)]
    best_cost = {0: 0.0}
    parent = {}
    expanded = 0
    while frontier:
        _estimated_total, cost, node_index = heapq.heappop(frontier)
        if cost > best_cost.get(node_index, math.inf) + 1e-12:
            continue
        expanded += 1
        if node_index == 1:
            break
        for neighbor, edge_cost in adjacency[node_index]:
            candidate_cost = cost + edge_cost
            if candidate_cost + 1e-12 >= best_cost.get(neighbor, math.inf):
                continue
            best_cost[neighbor] = candidate_cost
            parent[neighbor] = node_index
            heuristic = float(np.linalg.norm(nodes[neighbor] - goal))
            heapq.heappush(frontier, (candidate_cost + heuristic, candidate_cost, neighbor))
    if 1 not in best_cost:
        raise VisibilityGraphError("no collision-free visibility-graph path exists")

    indices = [1]
    while indices[-1] != 0:
        indices.append(parent[indices[-1]])
    path = [nodes[index] for index in reversed(indices)]
    path = _shortcut_path(path, spheres, margin)
    if len(path) == 2:
        path.insert(1, (path[0] + path[1]) * 0.5)
    if len(path) > int(max_guidepoints):
        raise VisibilityGraphError("validated fallback path exceeds guidepoint limit")
    points = tuple(tuple(float(value) for value in point) for point in path)
    validation = validate_polyline(
        points,
        spheres,
        bounds=bounds,
        clearance_margin_m=margin,
        require_origin_start=True,
        origin_tolerance_m=0.02,
        bounds_start_index=1,
    )
    if not validation.valid:
        raise VisibilityGraphError("visibility-graph result failed final geometry validation")
    length = sum(
        float(np.linalg.norm(np.asarray(points[index + 1]) - np.asarray(points[index])))
        for index in range(len(points) - 1)
    )
    return VisibilityGraphPlan(points, validation, len(nodes) - 2, expanded, length)

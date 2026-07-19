"""Pure terminal policy for bounded re-observation in SMPF Follow tasks."""

from dataclasses import dataclass
import math

from .deterministic_planner import (
    VisibilityGraphError,
    approach_goal_candidates_for_sphere,
)
from .geometry import validate_polyline


FOLLOW_CONTINUE = "continue"
FOLLOW_SUCCESS = "success"
FOLLOW_TIMEOUT = "timeout"
FOLLOW_UNSAFE = "unsafe"


@dataclass(frozen=True)
class FollowGoal:
    goal: tuple
    desired_standoff_goal: tuple
    distance_m: float
    distance_to_standoff_m: float
    target_center_distance_m: float
    target_surface_distance_m: float
    requested_surface_standoff_m: float
    safety_limited: bool
    target_visible: bool
    clipped: bool
    candidate_index: int = 0
    candidate_count: int = 1


class FollowGoalSelectionError(RuntimeError):
    """No point-safe target standoff candidate is available."""


def validate_follow_goal_point(
    goal,
    object_spheres=(),
    bounds=None,
    clearance_margin_m=0.0,
):
    """Validate only the tracking point; EGO validates and generates the route."""
    return validate_polyline(
        (goal, goal),
        object_spheres,
        bounds=bounds,
        clearance_margin_m=clearance_margin_m,
    )


def select_follow_goal(
    target_sphere,
    object_spheres=(),
    bounds=None,
    clearance_margin_m=0.0,
    surface_standoff_m=0.15,
    safety_padding_m=0.01,
    max_step_m=None,
    require_target_visibility=True,
):
    """Select one free point at the requested distance outside the target sphere."""
    requested_standoff = float(surface_standoff_m)
    margin = float(clearance_margin_m)
    padding = float(safety_padding_m)
    if not math.isfinite(requested_standoff) or requested_standoff <= 0.0:
        raise ValueError("Follow surface standoff must be finite and positive")
    if not math.isfinite(margin) or margin < 0.0:
        raise ValueError("Follow clearance margin must be finite and non-negative")
    if not math.isfinite(padding) or padding <= 0.0:
        raise ValueError("Follow safety padding must be finite and positive")
    selected_surface_standoff = max(requested_standoff, margin + padding)
    planner_standoff = selected_surface_standoff - margin
    try:
        candidates = approach_goal_candidates_for_sphere(
            target_sphere,
            clearance_margin_m=margin,
            standoff_m=planner_standoff,
            bounds=bounds,
            allow_inside=True,
        )
    except VisibilityGraphError as exc:
        raise FollowGoalSelectionError(str(exc)) from exc
    limit = None if max_step_m is None else float(max_step_m)
    if limit is not None and (not math.isfinite(limit) or limit <= 0.0):
        raise ValueError("follow max step must be finite and positive when enabled")

    target_center = tuple(float(value) for value in target_sphere.center)
    target_obstacles = []
    target_removed = False
    for sphere in object_spheres:
        same_target = (
            sphere is target_sphere
            or (
                sphere.label == target_sphere.label
                and math.isclose(sphere.radius, target_sphere.radius, abs_tol=1e-7)
                and all(
                    math.isclose(left, right, abs_tol=1e-7)
                    for left, right in zip(sphere.center, target_sphere.center)
                )
            )
        )
        if same_target and not target_removed:
            target_removed = True
            continue
        target_obstacles.append(sphere)
    rejected_kinds = set()
    for index, desired_goal in enumerate(candidates):
        desired_distance = math.sqrt(sum(value * value for value in desired_goal))
        if desired_distance <= 1e-9:
            continue
        step_distance = desired_distance if limit is None else min(limit, desired_distance)
        ratio = step_distance / desired_distance
        goal = tuple(float(value) * ratio for value in desired_goal)
        validation = validate_follow_goal_point(
            goal,
            object_spheres,
            bounds=bounds,
            clearance_margin_m=margin,
        )
        if not validation.valid:
            rejected_kinds.update(issue.kind for issue in validation.issues)
            continue
        goal_target_distance = math.sqrt(
            sum((goal[axis] - target_center[axis]) ** 2 for axis in range(3))
        )
        target_visible = None
        if require_target_visibility:
            if goal_target_distance <= 1e-9:
                rejected_kinds.add("target_occluded")
                continue
            direction = tuple(
                (goal[axis] - target_center[axis]) / goal_target_distance
                for axis in range(3)
            )
            visible_surface = tuple(
                target_center[axis]
                + direction[axis] * (float(target_sphere.radius) + margin)
                for axis in range(3)
            )
            sight = validate_polyline(
                (goal, visible_surface),
                target_obstacles,
                clearance_margin_m=margin,
            )
            target_visible = sight.valid
            if not target_visible:
                rejected_kinds.add("target_occluded")
                continue
        return FollowGoal(
            goal=goal,
            desired_standoff_goal=tuple(float(value) for value in desired_goal),
            distance_m=step_distance,
            distance_to_standoff_m=desired_distance,
            target_center_distance_m=goal_target_distance,
            target_surface_distance_m=goal_target_distance - float(target_sphere.radius),
            requested_surface_standoff_m=requested_standoff,
            safety_limited=selected_surface_standoff > requested_standoff + 1e-9,
            target_visible=target_visible,
            clipped=limit is not None and desired_distance > limit + 1e-9,
            candidate_index=index,
            candidate_count=len(candidates),
        )
    detail = ",".join(sorted(rejected_kinds)) or "no_valid_candidate"
    raise FollowGoalSelectionError("all Follow standoff points were rejected: %s" % detail)


def evaluate_follow_surface_standoff(
    target_surface_distance_m,
    desired_standoff_m=0.15,
    tolerance_m=0.10,
    minimum_safe_surface_distance_m=0.0,
    final_observation=False,
):
    distance = float(target_surface_distance_m)
    desired = float(desired_standoff_m)
    tolerance = float(tolerance_m)
    minimum = float(minimum_safe_surface_distance_m)
    if not all(math.isfinite(value) for value in (distance, desired, tolerance, minimum)):
        raise ValueError("Follow surface distances must be finite")
    if desired <= 0.0 or tolerance <= 0.0 or minimum < 0.0:
        raise ValueError("Follow surface-standoff policy is invalid")
    if distance < minimum:
        return FOLLOW_UNSAFE
    if abs(distance - desired) <= tolerance:
        return FOLLOW_SUCCESS
    if final_observation:
        return FOLLOW_TIMEOUT
    return FOLLOW_CONTINUE


def next_follow_observation_is_final(completed_cycles, max_cycles):
    completed_cycles = int(completed_cycles)
    max_cycles = int(max_cycles)
    if completed_cycles < 0 or max_cycles < 1:
        raise ValueError("follow cycle counts are invalid")
    return completed_cycles >= max_cycles

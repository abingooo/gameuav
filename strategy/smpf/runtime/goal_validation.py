"""Goal-conditioned terminal validation layered on SMPF path safety checks."""

from dataclasses import dataclass
import math
from typing import Iterable, Mapping, Optional

import numpy as np

from .contracts import ObjectSphere, ValidationIssue, ValidationResult
from .geometry import validate_polyline


@dataclass(frozen=True)
class GoalValidationResult:
    validation: ValidationResult
    target_surface_distance_m: Optional[float] = None
    target_progress_m: Optional[float] = None
    target_visible: Optional[bool] = None


def _same_sphere(left, right):
    return (
        left.label == right.label
        and math.isclose(left.radius, right.radius, rel_tol=0.0, abs_tol=1e-7)
        and np.allclose(left.center, right.center, atol=1e-7)
    )


def validate_goal_conditioned_polyline(
    guidepoints,
    object_spheres: Iterable[ObjectSphere] = (),
    target_sphere: Optional[ObjectSphere] = None,
    bounds: Optional[Mapping[str, float]] = None,
    clearance_margin_m=0.0,
    min_target_standoff_m=0.15,
    max_target_standoff_m=1.0,
    min_target_progress_m=0.10,
    require_target_visibility=True,
    require_origin_start=False,
    origin_tolerance_m=0.05,
    bounds_start_index=0,
):
    """Verify collision safety and that the final point actually reaches the target."""
    spheres = tuple(object_spheres)
    base = validate_polyline(
        guidepoints,
        spheres,
        bounds=bounds,
        clearance_margin_m=clearance_margin_m,
        require_origin_start=require_origin_start,
        origin_tolerance_m=origin_tolerance_m,
        bounds_start_index=bounds_start_index,
    )
    if target_sphere is None:
        return GoalValidationResult(base)

    points = np.asarray(guidepoints, dtype=float)
    if points.ndim != 2 or points.shape[1:] != (3,) or len(points) < 2 or not np.all(np.isfinite(points)):
        return GoalValidationResult(base)
    maximum_standoff = float(max_target_standoff_m)
    minimum_standoff = float(min_target_standoff_m)
    minimum_progress = float(min_target_progress_m)
    if not all(math.isfinite(value) for value in (minimum_standoff, maximum_standoff)):
        raise ValueError("target standoff bounds must be finite")
    if minimum_standoff < 0.0 or maximum_standoff <= minimum_standoff:
        raise ValueError("target standoff band is invalid")
    if not math.isfinite(minimum_progress) or minimum_progress < 0.0:
        raise ValueError("minimum target progress must be finite and non-negative")

    center = np.asarray(target_sphere.center, dtype=float)
    start_distance = float(np.linalg.norm(points[0] - center))
    final_distance = float(np.linalg.norm(points[-1] - center))
    start_surface_distance = start_distance - target_sphere.radius
    final_surface_distance = final_distance - target_sphere.radius
    progress = start_distance - final_distance
    issues = list(base.issues)
    if final_surface_distance < minimum_standoff:
        issues.append(
            ValidationIssue(
                "target_too_close",
                len(points) - 1,
                object_label=target_sphere.label,
                clearance_m=final_surface_distance - minimum_standoff,
                message="final guidepoint is inside the minimum target standoff",
            )
        )
    if final_surface_distance > maximum_standoff:
        issues.append(
            ValidationIssue(
                "target_standoff",
                len(points) - 1,
                object_label=target_sphere.label,
                clearance_m=maximum_standoff - final_surface_distance,
                message="final guidepoint remains too far from the target surface",
            )
        )
    required_progress = min(
        minimum_progress,
        max(0.0, start_surface_distance - maximum_standoff),
    )
    if progress + 1e-7 < required_progress:
        issues.append(
            ValidationIssue(
                "target_progress",
                len(points) - 1,
                object_label=target_sphere.label,
                clearance_m=progress - required_progress,
                message="path does not make the required progress toward the target",
            )
        )

    target_visible = None
    minimum_clearance = base.minimum_clearance_m
    if require_target_visibility:
        target_visible = False
        if final_distance > 1e-9:
            direction = (points[-1] - center) / final_distance
            visible_surface = center + direction * (
                target_sphere.radius + max(0.0, float(clearance_margin_m))
            )
            obstacles = []
            target_removed = False
            for sphere in spheres:
                if not target_removed and _same_sphere(sphere, target_sphere):
                    target_removed = True
                    continue
                obstacles.append(sphere)
            sight = validate_polyline(
                (points[-1], visible_surface),
                obstacles,
                clearance_margin_m=clearance_margin_m,
            )
            target_visible = sight.valid
            minimum_clearance = min(minimum_clearance, sight.minimum_clearance_m)
            if not sight.valid:
                blocker = next((issue.object_label for issue in sight.issues if issue.object_label), "")
                issues.append(
                    ValidationIssue(
                        "target_occluded",
                        len(points) - 1,
                        object_label=blocker,
                        message="final guidepoint has no verified clear line of sight to the target",
                    )
                )
    validation = ValidationResult(
        valid=not issues,
        issues=tuple(issues),
        minimum_clearance_m=minimum_clearance,
    )
    return GoalValidationResult(
        validation,
        target_surface_distance_m=final_surface_distance,
        target_progress_m=progress,
        target_visible=target_visible,
    )

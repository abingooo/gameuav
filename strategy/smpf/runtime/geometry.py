"""Calibrated RGB-D geometry and deterministic guidepoint validation for SMPF."""

import math
from typing import Iterable, Mapping, Optional, Sequence, Tuple

import numpy as np

from .contracts import (
    CameraIntrinsics,
    DepthEstimate,
    ObjectSphere,
    ValidationIssue,
    ValidationResult,
)


def _vector3(value, name="vector"):
    array = np.asarray(value, dtype=float)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError("%s must be a finite three-vector" % name)
    return array


def _rotation3(value, name="rotation"):
    array = np.asarray(value, dtype=float)
    if array.shape != (3, 3) or not np.all(np.isfinite(array)):
        raise ValueError("%s must be a finite 3x3 matrix" % name)
    return array


def rotation_matrix_from_quaternion(quaternion_xyzw):
    """Return a 3x3 rotation matrix from an ``[x, y, z, w]`` quaternion."""
    quaternion = np.asarray(quaternion_xyzw, dtype=float)
    if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
        raise ValueError("quaternion must contain four finite values")
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-12:
        raise ValueError("quaternion norm is zero")
    x, y, z, w = quaternion / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def rigid_transform(rotation, translation):
    """Build ``T_target_source`` such that ``p_target = T @ p_source``."""
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = _rotation3(rotation)
    transform[:3, 3] = _vector3(translation, "translation")
    return transform


def _transform4(value, name="transform"):
    transform = np.asarray(value, dtype=float)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError("%s must be a finite 4x4 matrix" % name)
    if not np.allclose(transform[3], (0.0, 0.0, 0.0, 1.0), atol=1e-7):
        raise ValueError("%s must be a rigid homogeneous transform" % name)
    return transform


def validate_extrinsic_transform(transform, max_translation_m, name="extrinsic"):
    """Reject non-rigid or physically implausible calibrated sensor transforms."""
    transform = _transform4(transform, name)
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T.dot(rotation), np.eye(3), atol=1e-5):
        raise ValueError("%s rotation is not orthonormal" % name)
    determinant = float(np.linalg.det(rotation))
    if not math.isclose(determinant, 1.0, rel_tol=0.0, abs_tol=1e-5):
        raise ValueError("%s rotation determinant must be +1" % name)
    maximum = float(max_translation_m)
    if not math.isfinite(maximum) or maximum <= 0.0:
        raise ValueError("maximum extrinsic translation must be finite and positive")
    translation_norm = float(np.linalg.norm(transform[:3, 3]))
    if translation_norm > maximum:
        raise ValueError(
            "%s translation %.3f m exceeds physical limit %.3f m"
            % (name, translation_norm, maximum)
        )
    return transform.copy()


def invert_rigid_transform(transform):
    transform = _transform4(transform)
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    inverse = np.eye(4, dtype=float)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T.dot(translation)
    return inverse


def transform_points(transform, points):
    """Transform one point or an ``N x 3`` array while preserving single-point shape."""
    transform = _transform4(transform)
    array = np.asarray(points, dtype=float)
    single = array.shape == (3,)
    if single:
        array = array.reshape(1, 3)
    if array.ndim != 2 or array.shape[1] != 3 or not np.all(np.isfinite(array)):
        raise ValueError("points must be a finite three-vector or N x 3 array")
    result = array.dot(transform[:3, :3].T) + transform[:3, 3]
    return result[0] if single else result


def body_from_color_transform(body_from_depth, color_from_depth):
    """Compose VINS and RealSense extrinsics into ``T_body_color``.

    ``body_from_depth`` is the online VINS ``body_T_cam0`` transform. RealSense
    publishes ``depth_to_color`` as ``T_color_depth``. Aligned depth samples are
    expressed in the color optical frame, so they first pass through the inverse
    RealSense transform and then through VINS.
    """
    return _transform4(body_from_depth).dot(invert_rigid_transform(color_from_depth))


def body_from_color_via_infra1(body_from_infra1, infra1_from_depth, color_from_depth):
    """Compose the live VINS cam0 and both required RealSense extrinsics.

    VINS cam0 consumes ``/camera/infra1/image_rect_raw`` and publishes
    ``T_body_infra1``. RealSense publishes ``T_infra1_depth`` and
    ``T_color_depth``. A point deprojected from color-aligned depth is in the
    color optical frame, so ``T_body_color`` is::

        T_body_infra1 * T_infra1_depth * inverse(T_color_depth)
    """
    body_from_depth = _transform4(body_from_infra1).dot(_transform4(infra1_from_depth))
    return body_from_color_transform(body_from_depth, color_from_depth)


def deproject_color_pixel(pixel_uv, depth_m, intrinsics: CameraIntrinsics):
    """Deproject an aligned color pixel into ROS optical coordinates (right, down, forward)."""
    if len(pixel_uv) != 2:
        raise ValueError("pixel must be [u, v]")
    u, v = float(pixel_uv[0]), float(pixel_uv[1])
    depth_m = float(depth_m)
    if not all(math.isfinite(value) for value in (u, v, depth_m)) or depth_m <= 0.0:
        raise ValueError("pixel and depth must be finite; depth must be positive")
    return np.array(
        [
            (u - float(intrinsics.cx)) * depth_m / float(intrinsics.fx),
            (v - float(intrinsics.cy)) * depth_m / float(intrinsics.fy),
            depth_m,
        ],
        dtype=float,
    )


def robust_depth_estimate(
    depth_map,
    pixel_coords,
    window_radius=2,
    minimum_depth_m=0.15,
    maximum_depth_m=10.0,
    lower_quantile=0.1,
    upper_quantile=0.9,
):
    """Estimate target depth from aligned samples while rejecting zeros and outliers."""
    depth = np.asarray(depth_map, dtype=float)
    if depth.ndim != 2:
        raise ValueError("depth_map must be a two-dimensional array in meters")
    if not 0.0 <= lower_quantile < upper_quantile <= 1.0:
        raise ValueError("invalid depth quantile range")
    radius = max(0, int(window_radius))
    height, width = depth.shape
    samples = []
    for point in pixel_coords:
        if point is None or len(point) < 2:
            continue
        u = int(round(float(point[0])))
        v = int(round(float(point[1])))
        if u < 0 or u >= width or v < 0 or v >= height:
            continue
        u0, u1 = max(0, u - radius), min(width, u + radius + 1)
        v0, v1 = max(0, v - radius), min(height, v + radius + 1)
        samples.extend(depth[v0:v1, u0:u1].reshape(-1).tolist())
    values = np.asarray(samples, dtype=float)
    values = values[
        np.isfinite(values)
        & (values >= float(minimum_depth_m))
        & (values <= float(maximum_depth_m))
    ]
    if values.size == 0:
        raise ValueError("no valid aligned depth samples")
    if values.size >= 5:
        low, high = np.quantile(values, (lower_quantile, upper_quantile))
        clipped = values[(values >= low) & (values <= high)]
        if clipped.size:
            values = clipped
    median = float(np.median(values))
    mad = float(np.median(np.abs(values - median)))
    robust_std = 1.4826 * mad
    if robust_std <= 1e-9 and values.size > 1:
        robust_std = float(np.std(values))
    return DepthEstimate(
        value_m=median,
        std_m=max(0.0, robust_std),
        sample_count=int(values.size),
        minimum_m=float(np.min(values)),
        maximum_m=float(np.max(values)),
    )


def _default_bbox_samples(ymin, xmin, ymax, xmax):
    samples = []
    for v_ratio in (0.25, 0.5, 0.75):
        for u_ratio in (0.25, 0.5, 0.75):
            samples.append(
                (
                    xmin + (xmax - xmin) * u_ratio,
                    ymin + (ymax - ymin) * v_ratio,
                )
            )
    return samples


def sphere_from_aligned_bbox(
    label,
    bbox_yxyx,
    depth_map,
    intrinsics: CameraIntrinsics,
    target_from_color,
    sample_points_uv=None,
    safety_margin_m=0.30,
    minimum_radius_m=0.20,
    confidence=1.0,
    frame_id="body_flu",
    source="aligned_rgbd",
):
    """Build a conservative metric sphere from a segmented, color-aligned target box."""
    if bbox_yxyx is None or len(bbox_yxyx) != 4:
        raise ValueError("bbox must be [ymin, xmin, ymax, xmax]")
    ymin, xmin, ymax, xmax = (float(value) for value in bbox_yxyx)
    ymin, ymax = sorted((ymin, ymax))
    xmin, xmax = sorted((xmin, xmax))
    ymin = min(max(ymin, 0.0), intrinsics.height - 1.0)
    ymax = min(max(ymax, 0.0), intrinsics.height - 1.0)
    xmin = min(max(xmin, 0.0), intrinsics.width - 1.0)
    xmax = min(max(xmax, 0.0), intrinsics.width - 1.0)
    if xmax - xmin < 1.0 or ymax - ymin < 1.0:
        raise ValueError("bbox has no usable area")

    samples = list(sample_points_uv or _default_bbox_samples(ymin, xmin, ymax, xmax))
    center_uv = ((xmin + xmax) * 0.5, (ymin + ymax) * 0.5)
    samples.append(center_uv)
    estimate = robust_depth_estimate(depth_map, samples)

    optical_points = [deproject_color_pixel(center_uv, estimate.value_m, intrinsics)]
    for point in ((xmin, ymin), (xmax, ymin), (xmin, ymax), (xmax, ymax)):
        optical_points.append(deproject_color_pixel(point, estimate.value_m, intrinsics))
    target_points = transform_points(target_from_color, optical_points)
    center = target_points[0]
    visible_radius = max(float(np.linalg.norm(point - center)) for point in target_points[1:])
    radius = max(float(minimum_radius_m), visible_radius)
    radius += max(0.0, float(safety_margin_m)) + 2.0 * estimate.std_m
    sphere = ObjectSphere(
        label=str(label),
        center=tuple(float(value) for value in center),
        radius=radius,
        confidence=float(confidence),
        frame_id=str(frame_id),
        source=str(source),
    )
    return sphere, estimate


def point_to_segment_distance(point, segment_start, segment_end):
    point = _vector3(point, "point")
    start = _vector3(segment_start, "segment_start")
    end = _vector3(segment_end, "segment_end")
    delta = end - start
    length_squared = float(delta.dot(delta))
    if length_squared <= 1e-12:
        return float(np.linalg.norm(point - start))
    fraction = float((point - start).dot(delta) / length_squared)
    fraction = min(1.0, max(0.0, fraction))
    closest = start + fraction * delta
    return float(np.linalg.norm(point - closest))


def _bound_value(bounds: Mapping[str, float], axis, side):
    for key in ("%s_%s" % (axis, side), "%s%s" % (side, axis), "%s_%s" % (side, axis)):
        if key in bounds:
            return float(bounds[key])
    return None


def validate_polyline(
    guidepoints,
    object_spheres: Iterable[ObjectSphere] = (),
    bounds: Optional[Mapping[str, float]] = None,
    clearance_margin_m=0.0,
    require_origin_start=False,
    origin_tolerance_m=0.05,
    bounds_start_index=0,
):
    """Validate guidepoints and every connecting segment against metric spheres."""
    points = np.asarray(guidepoints, dtype=float)
    issues = []
    if points.ndim != 2 or points.shape[1:] != (3,) or len(points) < 2:
        return ValidationResult(
            valid=False,
            issues=(ValidationIssue("shape", -1, message="guidepoints must be an N x 3 list with N >= 2"),),
            minimum_clearance_m=-math.inf,
        )
    if not np.all(np.isfinite(points)):
        return ValidationResult(
            valid=False,
            issues=(ValidationIssue("non_finite", -1, message="guidepoints contain non-finite values"),),
            minimum_clearance_m=-math.inf,
        )
    if require_origin_start and float(np.linalg.norm(points[0])) > float(origin_tolerance_m):
        issues.append(
            ValidationIssue(
                "start",
                0,
                message="first guidepoint is not the current body-frame origin",
            )
        )

    if bounds:
        bounds_start_index = max(0, int(bounds_start_index))
        for index, point in enumerate(points[bounds_start_index:], start=bounds_start_index):
            for axis_index, axis in enumerate(("x", "y", "z")):
                minimum = _bound_value(bounds, axis, "min")
                maximum = _bound_value(bounds, axis, "max")
                if minimum is not None and point[axis_index] < minimum:
                    issues.append(ValidationIssue("bounds", index, message="%s below minimum" % axis))
                if maximum is not None and point[axis_index] > maximum:
                    issues.append(ValidationIssue("bounds", index, message="%s above maximum" % axis))

    minimum_clearance = math.inf
    margin = max(0.0, float(clearance_margin_m))
    for sphere in object_spheres:
        center = _vector3(sphere.center, "sphere.center")
        required_distance = float(sphere.radius) + margin
        for index, point in enumerate(points):
            clearance = float(np.linalg.norm(point - center)) - required_distance
            minimum_clearance = min(minimum_clearance, clearance)
            if clearance < 0.0:
                issues.append(
                    ValidationIssue(
                        "point_collision",
                        index,
                        object_label=sphere.label,
                        clearance_m=clearance,
                        message="guidepoint enters the object safety sphere",
                    )
                )
        for index in range(len(points) - 1):
            clearance = point_to_segment_distance(center, points[index], points[index + 1]) - required_distance
            minimum_clearance = min(minimum_clearance, clearance)
            if clearance < 0.0:
                issues.append(
                    ValidationIssue(
                        "segment_collision",
                        index,
                        object_label=sphere.label,
                        clearance_m=clearance,
                        message="guidepoint segment crosses the object safety sphere",
                    )
                )
    return ValidationResult(
        valid=not issues,
        issues=tuple(issues),
        minimum_clearance_m=minimum_clearance,
    )

"""Deterministic world-frame target-facing yaw policy for SMPF waypoints."""

import math


def normalize_yaw(yaw):
    yaw = float(yaw)
    if not math.isfinite(yaw):
        raise ValueError("yaw must be finite")
    return math.atan2(math.sin(yaw), math.cos(yaw))


def target_facing_yaws(world_waypoints, target_world, fallback_yaw=0.0):
    target = tuple(float(value) for value in target_world)
    if len(target) != 3 or not all(math.isfinite(value) for value in target):
        raise ValueError("target_world must be a finite three-vector")
    previous = normalize_yaw(fallback_yaw)
    result = []
    for raw_point in world_waypoints:
        point = tuple(float(value) for value in raw_point)
        if len(point) != 3 or not all(math.isfinite(value) for value in point):
            raise ValueError("world waypoints must be finite three-vectors")
        dx = target[0] - point[0]
        dy = target[1] - point[1]
        if math.hypot(dx, dy) > 1e-6:
            previous = math.atan2(dy, dx)
        result.append(previous)
    return tuple(result)

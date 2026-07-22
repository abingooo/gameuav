"""ROS-independent sequential waypoint execution state for SMPF."""

import math


ACTIVE_EXECUTION_STATES = {"WAITING_ARRIVAL"}
TERMINAL_EXECUTION_STATES = {"SUCCESS", "TIMEOUT", "ABORTED", "ERROR"}


class ExecutionStateError(RuntimeError):
    pass


class WaypointExecutionLoop:
    """Advance a verified world-frame path using arrival and settle checks."""

    def __init__(
        self,
        goal_timeout_sec=45.0,
        task_timeout_sec=300.0,
        arrival_settle_sec=0.0,
        goal_tolerance_xy=0.25,
        goal_tolerance_z=0.20,
        arrival_max_speed=0.25,
        goal_tolerance_yaw_rad=math.radians(10.0),
        odom_timeout_sec=1.0,
    ):
        self.goal_timeout_sec = float(goal_timeout_sec)
        self.task_timeout_sec = float(task_timeout_sec)
        self.arrival_settle_sec = float(arrival_settle_sec)
        self.goal_tolerance_xy = float(goal_tolerance_xy)
        self.goal_tolerance_z = float(goal_tolerance_z)
        self.arrival_max_speed = float(arrival_max_speed)
        self.goal_tolerance_yaw_rad = float(goal_tolerance_yaw_rad)
        self.odom_timeout_sec = float(odom_timeout_sec)
        self.enabled = False
        self.state = "DISABLED"
        self.reason = "execution is disabled"
        self.task_id = None
        self._waypoints = ()
        self._waypoint_yaws = ()
        self._index = -1
        self.started_at = None
        self.goal_started_at = None
        self.arrival_since = None
        self.updated_at = None
        self.distance_xy = None
        self.distance_z = None
        self.speed = None
        self.yaw_error = None

    def set_enabled(self, enabled, now):
        now = float(now)
        if not enabled:
            if self.state in ACTIVE_EXECUTION_STATES:
                self._finish("ABORTED", "execution disabled", now)
            self.enabled = False
            self.state = "DISABLED"
            self.reason = "execution is disabled"
            self.updated_at = now
            return []
        self.enabled = True
        if self.state == "DISABLED":
            self.state = "IDLE"
            self.reason = "ready"
            self.updated_at = now
        return []

    def start(self, task_id, world_waypoints, now, waypoint_yaws=None):
        now = float(now)
        if not self.enabled:
            raise ExecutionStateError("execution is disabled")
        if self.state in ACTIVE_EXECUTION_STATES:
            raise ExecutionStateError("another waypoint path is active")
        points = tuple(self._point(point) for point in world_waypoints)
        if not points:
            raise ExecutionStateError("waypoint path cannot be empty")
        if waypoint_yaws is None:
            yaws = (None,) * len(points)
        else:
            yaws = tuple(self._yaw(value) for value in waypoint_yaws)
            if len(yaws) != len(points):
                raise ExecutionStateError("waypoint yaw count must match waypoint count")
        self.task_id = str(task_id or "")
        self._waypoints = points
        self._waypoint_yaws = yaws
        self._index = 0
        self.started_at = now
        self.goal_started_at = now
        self.arrival_since = None
        self.updated_at = now
        self.state = "WAITING_ARRIVAL"
        self.reason = "first verified waypoint ready"
        return [
            {
                "type": "publish_goal",
                "goal": self.current_goal,
                "yaw": self.current_goal_yaw,
                "index": self._index,
            }
        ]

    def tick(self, now, odom):
        now = float(now)
        if self.state not in ACTIVE_EXECUTION_STATES:
            return []
        if now - self.started_at > self.task_timeout_sec:
            self._finish("TIMEOUT", "task timeout", now)
            return [{"type": "terminal", "state": self.state, "reason": self.reason}]
        if odom is None or now - float(odom.get("stamp", -math.inf)) > self.odom_timeout_sec:
            self._finish("ERROR", "odometry is stale", now)
            return [{"type": "terminal", "state": self.state, "reason": self.reason}]
        is_final_waypoint = self._index == len(self._waypoints) - 1
        if is_final_waypoint and now - self.goal_started_at > self.goal_timeout_sec:
            self._finish("TIMEOUT", "waypoint timeout", now)
            return [{"type": "terminal", "state": self.state, "reason": self.reason}]

        goal = self.current_goal
        dx = float(odom["x"]) - goal[0]
        dy = float(odom["y"]) - goal[1]
        dz = float(odom["z"]) - goal[2]
        self.distance_xy = math.hypot(dx, dy)
        self.distance_z = abs(dz)
        self.speed = float(odom["speed"])
        goal_yaw = self.current_goal_yaw
        if goal_yaw is None:
            self.yaw_error = None
            yaw_arrived = True
        else:
            if "yaw" not in odom or not math.isfinite(float(odom["yaw"])):
                self._finish("ERROR", "odometry yaw is unavailable", now)
                return [{"type": "terminal", "state": self.state, "reason": self.reason}]
            self.yaw_error = abs(self._angle_error(float(odom["yaw"]), goal_yaw))
            yaw_arrived = self.yaw_error <= self.goal_tolerance_yaw_rad
        arrived = (
            self.distance_xy <= self.goal_tolerance_xy
            and self.distance_z <= self.goal_tolerance_z
            and self.speed <= self.arrival_max_speed
            and yaw_arrived
        )
        if not arrived:
            self.arrival_since = None
            return []
        if self.arrival_since is None:
            self.arrival_since = now
            self.reason = "waypoint reached; waiting to settle"
            self.updated_at = now
            if self.arrival_settle_sec > 0.0:
                return []
        if now - self.arrival_since < self.arrival_settle_sec:
            return []

        self._index += 1
        self.arrival_since = None
        self.updated_at = now
        if self._index >= len(self._waypoints):
            self._finish("SUCCESS", "verified waypoint path completed", now)
            return [{"type": "terminal", "state": self.state, "reason": self.reason}]
        self.goal_started_at = now
        self.reason = "next verified waypoint ready"
        return [
            {
                "type": "publish_goal",
                "goal": self.current_goal,
                "yaw": self.current_goal_yaw,
                "index": self._index,
            }
        ]

    def abort(self, reason, now):
        if self.state in ACTIVE_EXECUTION_STATES:
            self._finish("ABORTED", str(reason or "operator aborted"), float(now))
            return [{"type": "terminal", "state": self.state, "reason": self.reason}]
        return []

    @property
    def current_goal(self):
        if self._index < 0 or self._index >= len(self._waypoints):
            return None
        return self._waypoints[self._index]

    @property
    def current_goal_yaw(self):
        if self._index < 0 or self._index >= len(self._waypoint_yaws):
            return None
        return self._waypoint_yaws[self._index]

    def status(self):
        return {
            "enabled": self.enabled,
            "state": self.state,
            "reason": self.reason,
            "task_id": self.task_id,
            "waypoint_index": self._index,
            "waypoint_count": len(self._waypoints),
            "current_goal": self.current_goal,
            "current_goal_yaw": self.current_goal_yaw,
            "distance_xy": self.distance_xy,
            "distance_z": self.distance_z,
            "speed": self.speed,
            "yaw_error": self.yaw_error,
            "updated_at": self.updated_at,
        }

    @staticmethod
    def _point(value):
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            raise ExecutionStateError("each world waypoint must contain three values")
        point = tuple(float(item) for item in value)
        if not all(math.isfinite(item) for item in point):
            raise ExecutionStateError("world waypoints must be finite")
        return point

    @staticmethod
    def _yaw(value):
        yaw = float(value)
        if not math.isfinite(yaw):
            raise ExecutionStateError("waypoint yaws must be finite")
        return math.atan2(math.sin(yaw), math.cos(yaw))

    @staticmethod
    def _angle_error(actual, target):
        return math.atan2(math.sin(actual - target), math.cos(actual - target))

    def _finish(self, state, reason, now):
        self.state = state
        self.reason = reason
        self.updated_at = float(now)
        self.arrival_since = None

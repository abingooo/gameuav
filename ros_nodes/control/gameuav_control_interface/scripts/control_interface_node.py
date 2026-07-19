#!/usr/bin/env python3

import json
import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Bool, Empty, Float64, String


WORLD_FRAMES = {"", "world", "map", "odom", "local", "enu"}
BODY_FRAMES = {"body", "body_enu", "base_link", "base", "ego", "local_body"}


def _clamp(value, low, high):
    return max(low, min(high, value))


def _finite(value, default=0.0):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def _normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _roll_pitch_from_quaternion(q):
    values = (_finite(q.x, math.nan), _finite(q.y, math.nan), _finite(q.z, math.nan), _finite(q.w, math.nan))
    if not all(math.isfinite(value) for value in values):
        return None
    x, y, z, w = values
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return None
    x, y, z, w = (value / norm for value in values)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_sin = _clamp(2.0 * (w * y - z * x), -1.0, 1.0)
    return roll, math.asin(pitch_sin)


def _roll_pitch_error_deg(estimate, reference):
    estimate_angles = _roll_pitch_from_quaternion(estimate)
    reference_angles = _roll_pitch_from_quaternion(reference)
    if estimate_angles is None or reference_angles is None:
        return None
    roll_error = abs(_normalize_angle(estimate_angles[0] - reference_angles[0]))
    pitch_error = abs(_normalize_angle(estimate_angles[1] - reference_angles[1]))
    return math.degrees(roll_error), math.degrees(pitch_error)


def _yaw_to_quaternion(yaw, orientation):
    half = yaw * 0.5
    orientation.x = 0.0
    orientation.y = 0.0
    orientation.z = math.sin(half)
    orientation.w = math.cos(half)


class ControlInterfaceNode:
    def __init__(self):
        self.rate_hz = rospy.get_param("~rate_hz", 50.0)
        self.direct_position_timeout = rospy.get_param("~direct_position_timeout", 5.0)
        self.spf_position_timeout = rospy.get_param("~spf_position_timeout", 0.0)
        self.speed_timeout = rospy.get_param("~speed_timeout", 0.6)
        self.ego_cmd_timeout = rospy.get_param("~ego_cmd_timeout", 0.6)
        self.max_speed = rospy.get_param("~max_speed", 1.0)
        self.max_vertical_speed = rospy.get_param("~max_vertical_speed", 0.5)
        self.max_position_step = rospy.get_param("~max_position_step", 3.0)
        self.min_z = rospy.get_param("~min_z", 0.05)
        self.max_z = rospy.get_param("~max_z", 3.0)
        self.attitude_guard_enabled = bool(rospy.get_param("~attitude_guard_enabled", True))
        self.attitude_timeout = float(rospy.get_param("~attitude_timeout", 0.5))
        self.max_roll_pitch_error_deg = float(
            rospy.get_param("~max_roll_pitch_error_deg", 15.0)
        )
        self.spf_mavros_state_timeout_sec = float(
            rospy.get_param("~spf_mavros_state_timeout_sec", 2.5)
        )

        odom_topic = rospy.get_param("~odom_topic", "/vins_fusion/imu_propagate")
        attitude_reference_topic = rospy.get_param(
            "~attitude_reference_topic", "/mavros/local_position/pose"
        )
        ego_cmd_topic = rospy.get_param("~ego_position_cmd_topic", "/control/ego_position_cmd")
        position_topic = rospy.get_param("~direct_position_topic", "/control/position")
        spf_position_topic = rospy.get_param("~spf_position_topic", "/control/spf_position")
        spf_enable_topic = rospy.get_param("~spf_enable_topic", "/spf/enable")
        mavros_state_topic = rospy.get_param("~mavros_state_topic", "/mavros/state")
        speed_topic = rospy.get_param("~speed_topic", "/control/speed")
        ego_position_topic = rospy.get_param("~ego_position_topic", "/control/ego_position")
        stop_topic = rospy.get_param("~stop_topic", "/control/stop")
        output_topic = rospy.get_param("~output_position_cmd_topic", "/control/position_cmd")
        planning_goal_topic = rospy.get_param("~planning_goal_topic", "/planning/goal")
        planning_yaw_topic = rospy.get_param("~planning_yaw_topic", "/planning/goal_yaw_deg")
        status_topic = rospy.get_param("~status_topic", "/control/interface_status")

        self.output_pub = rospy.Publisher(output_topic, PositionCommand, queue_size=20)
        self.goal_pub = rospy.Publisher(planning_goal_topic, PoseStamped, queue_size=10, latch=True)
        self.yaw_pub = rospy.Publisher(planning_yaw_topic, Float64, queue_size=10, latch=True)
        self.status_pub = rospy.Publisher(status_topic, String, queue_size=10)

        self.odom = None
        self.odom_stamp = rospy.Time(0)
        self.attitude_reference = None
        self.attitude_reference_stamp = rospy.Time(0)
        self.spf_lock = threading.RLock()
        self.spf_execution_enabled = False
        self.spf_mavros_state = None
        self.spf_mavros_state_stamp = rospy.Time(0)
        self.motion_rearm_required = self.attitude_guard_enabled
        self.latest_ego_cmd = None
        self.latest_ego_stamp = rospy.Time(0)
        self.mode = "ego_passthrough"
        self.direct_target = None
        self.direct_yaw = 0.0
        self.direct_stamp = rospy.Time(0)
        self.speed_target = None
        self.speed_yaw_rate = 0.0
        self.speed_reference = None
        self.speed_stamp = rospy.Time(0)
        self.last_timer_stamp = rospy.Time.now()
        self.last_status_stamp = rospy.Time(0)

        rospy.Subscriber(odom_topic, Odometry, self.odom_cb, queue_size=20)
        if self.attitude_guard_enabled:
            rospy.Subscriber(
                attitude_reference_topic,
                PoseStamped,
                self.attitude_reference_cb,
                queue_size=20,
            )
        rospy.Subscriber(ego_cmd_topic, PositionCommand, self.ego_cmd_cb, queue_size=20)
        rospy.Subscriber(position_topic, PoseStamped, self.position_cb, queue_size=10)
        rospy.Subscriber(spf_position_topic, PoseStamped, self.spf_position_cb, queue_size=10)
        rospy.Subscriber(spf_enable_topic, Bool, self.spf_enable_cb, queue_size=10)
        rospy.Subscriber(mavros_state_topic, State, self.mavros_state_cb, queue_size=10)
        rospy.Subscriber(speed_topic, TwistStamped, self.speed_cb, queue_size=10)
        rospy.Subscriber(ego_position_topic, PoseStamped, self.ego_position_cb, queue_size=10)
        rospy.Subscriber(stop_topic, Empty, self.stop_cb, queue_size=10)

        period = 1.0 / max(1.0, float(self.rate_hz))
        self.timer = rospy.Timer(rospy.Duration(period), self.timer_cb)
        rospy.loginfo(
            "[control_interface] ready: ego=%s position=%s spf_position=%s speed=%s output=%s attitude_guard=%s",
            ego_cmd_topic,
            position_topic,
            spf_position_topic,
            speed_topic,
            output_topic,
            self.attitude_guard_enabled,
        )

    def odom_cb(self, msg):
        self.odom = msg
        self.odom_stamp = rospy.Time.now()

    def attitude_reference_cb(self, msg):
        self.attitude_reference = msg
        self.attitude_reference_stamp = rospy.Time.now()

    def ego_cmd_cb(self, msg):
        with self.spf_lock:
            now = rospy.Time.now()
            if not self._guard_allows(now, "ego_cmd") or self.motion_rearm_required:
                return
            self.latest_ego_cmd = msg
            self.latest_ego_stamp = now
            if not self._direct_active(now):
                self.mode = "ego_passthrough"
                self.output_pub.publish(msg)

    def position_cb(self, msg):
        with self.spf_lock:
            self._accept_position(msg, "direct_position")

    def spf_position_cb(self, msg):
        now = rospy.Time.now()
        with self.spf_lock:
            execution_error = self._spf_execution_error_locked(now)
            if execution_error:
                self.spf_execution_enabled = False
                self._clear_spf_target_locked()
                self._publish_status("reject_spf_position", execution_error)
                return
            self._accept_position(msg, "spf_position")

    def spf_enable_cb(self, msg):
        now = rospy.Time.now()
        with self.spf_lock:
            requested = bool(msg.data)
            if requested:
                vehicle_error = self._spf_vehicle_state_error_locked(now)
                if vehicle_error:
                    self.spf_execution_enabled = False
                    self._clear_spf_target_locked()
                    self._publish_status("reject_spf_enable", vehicle_error)
                    return
            self.spf_execution_enabled = requested
            if not requested:
                self._clear_spf_target_locked()
            self._publish_status("spf_enable", "enabled=%s" % requested)

    def mavros_state_cb(self, msg):
        now = rospy.Time.now()
        with self.spf_lock:
            self.spf_mavros_state = msg
            self.spf_mavros_state_stamp = now
            if msg.connected and msg.armed:
                return
            was_active = self.spf_execution_enabled or self.mode == "spf_position"
            self.spf_execution_enabled = False
            self._clear_spf_target_locked()
            if was_active:
                self._publish_status(
                    "spf_gate_closed",
                    "MAVROS disconnected or PX4 disarmed",
                )

    def _accept_position(self, msg, mode):
        now = rospy.Time.now()
        if not self._guard_allows(now, mode):
            return
        resolved = self._resolve_pose(msg, use_current_as_default=False)
        if resolved is None:
            self._publish_status("reject_%s" % mode, "odom unavailable or invalid frame")
            return
        x, y, z, yaw = resolved
        self.direct_target = (x, y, _clamp(z, self.min_z, self.max_z))
        self.direct_yaw = yaw
        self.direct_stamp = now
        self.motion_rearm_required = False
        self.mode = mode
        self._publish_status(mode, "accepted")

    def speed_cb(self, msg):
        with self.spf_lock:
            now = rospy.Time.now()
            if not self._guard_allows(now, "speed"):
                return
            if self.odom is None:
                self._publish_status("reject_speed", "odom unavailable")
                return
            vx, vy, vz, yaw_rate = self._resolve_twist(msg)
            speed_xy = math.hypot(vx, vy)
            if speed_xy > self.max_speed > 0.0:
                scale = self.max_speed / speed_xy
                vx *= scale
                vy *= scale
            vz = _clamp(vz, -self.max_vertical_speed, self.max_vertical_speed)
            self.speed_target = (vx, vy, vz)
            self.speed_yaw_rate = _clamp(yaw_rate, -math.pi, math.pi)
            self.speed_reference = self._current_pose_tuple()
            self.speed_stamp = now
            self.motion_rearm_required = False
            self.mode = "speed"
            self._publish_status("speed", "accepted")

    def ego_position_cb(self, msg):
        with self.spf_lock:
            now = rospy.Time.now()
            if not self._guard_allows(now, "ego_position"):
                return
            resolved = self._resolve_pose(msg, use_current_as_default=True)
            if resolved is None:
                self._publish_status("reject_ego_position", "odom unavailable or invalid frame")
                return
            x, y, z, yaw = resolved
            goal = PoseStamped()
            goal.header.stamp = now
            goal.header.frame_id = "world"
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = _clamp(z, self.min_z, self.max_z)
            _yaw_to_quaternion(yaw, goal.pose.orientation)
            self.goal_pub.publish(goal)
            yaw_msg = Float64()
            yaw_msg.data = math.degrees(yaw)
            self.yaw_pub.publish(yaw_msg)
            self.latest_ego_cmd = None
            self.motion_rearm_required = False
            self.mode = "ego_passthrough"
            self._publish_status("ego_position", "published planning goal")

    def stop_cb(self, _msg):
        with self.spf_lock:
            self.latest_ego_cmd = None
            self.speed_target = None
            self.motion_rearm_required = True
            if not self._guard_allows(rospy.Time.now(), "stop"):
                self.direct_target = None
                self.mode = "attitude_guard"
                return
            if self.odom is None:
                self.mode = "ego_passthrough"
                self.direct_target = None
                self._publish_status("stop", "odom unavailable; returned to ego passthrough")
                return
            x, y, z, yaw = self._current_pose_tuple()
            self.direct_target = (x, y, _clamp(z, self.min_z, self.max_z))
            self.direct_yaw = yaw
            self.direct_stamp = rospy.Time.now()
            self.mode = "direct_position"
            self._publish_status("stop", "holding current position")

    def timer_cb(self, event):
        with self.spf_lock:
            self._timer_cb_locked(event)

    def _timer_cb_locked(self, event):
        now = rospy.Time.now()
        dt = max(0.0, min(0.1, (now - self.last_timer_stamp).to_sec()))
        self.last_timer_stamp = now

        with self.spf_lock:
            if self.spf_execution_enabled:
                vehicle_error = self._spf_vehicle_state_error_locked(now)
                if vehicle_error:
                    self.spf_execution_enabled = False
                    self._clear_spf_target_locked()
                    self._publish_status("spf_gate_closed", vehicle_error)

        guard_ok, guard_detail, _guard_value = self._attitude_guard_state(now)
        if not guard_ok:
            if self.mode != "attitude_guard":
                self._invalidate_motion()
                self.mode = "attitude_guard"
                self._publish_status("attitude_guard", guard_detail)
            elif (now - self.last_status_stamp).to_sec() > 1.0:
                self._publish_status("attitude_guard", guard_detail)
            return
        if self.mode == "attitude_guard":
            self.mode = "ego_passthrough"
            self._publish_status("attitude_guard_recovered", "waiting for a new motion command")

        spf_position_published = False
        if self.mode == "spf_position":
            with self.spf_lock:
                execution_error = self._spf_execution_error_locked(now)
                if execution_error:
                    self.spf_execution_enabled = False
                    self._clear_spf_target_locked()
                    self._publish_status("reject_spf_position", execution_error)
                elif self._position_active(now):
                    self.output_pub.publish(
                        self._build_position_cmd(
                            self.direct_target,
                            (0.0, 0.0, 0.0),
                            self.direct_yaw,
                        )
                    )
                    spf_position_published = True

        if spf_position_published:
            pass
        elif self.mode == "direct_position" and self._position_active(now):
            self.output_pub.publish(self._build_position_cmd(self.direct_target, (0.0, 0.0, 0.0), self.direct_yaw))
        elif self.mode == "speed" and self._speed_active(now):
            self._integrate_speed(dt)
            self.output_pub.publish(self._build_position_cmd(self.speed_reference[:3], self.speed_target, self.speed_reference[3], self.speed_yaw_rate))
        else:
            if self.mode in {"direct_position", "spf_position", "speed"}:
                self._publish_status("timeout", "returned to ego passthrough")
            self.mode = "ego_passthrough"
            if self.latest_ego_cmd is not None and (now - self.latest_ego_stamp).to_sec() <= self.ego_cmd_timeout:
                self.output_pub.publish(self.latest_ego_cmd)

        if (now - self.last_status_stamp).to_sec() > 1.0:
            self._publish_status(self.mode, "alive")

    def _direct_active(self, now):
        return self._position_active(now) or self._speed_active(now)

    def _guard_allows(self, now, source):
        ok, detail, _value = self._attitude_guard_state(now)
        if not ok and (now - self.last_status_stamp).to_sec() > 0.5:
            self._publish_status("reject_%s" % source, detail)
        return ok

    def _attitude_guard_state(self, now):
        if not self.attitude_guard_enabled:
            return True, "disabled", None
        if self.odom is None:
            return False, "VINS attitude unavailable", None
        odom_age = (now - self.odom_stamp).to_sec()
        if odom_age < 0.0 or odom_age > self.attitude_timeout:
            return False, "VINS attitude stale: %.3fs" % odom_age, {"odom_age_sec": odom_age}
        if self.attitude_reference is None:
            return False, "PX4 attitude unavailable", None
        reference_age = (now - self.attitude_reference_stamp).to_sec()
        if reference_age < 0.0 or reference_age > self.attitude_timeout:
            return (
                False,
                "PX4 attitude stale: %.3fs" % reference_age,
                {"reference_age_sec": reference_age},
            )
        errors = _roll_pitch_error_deg(
            self.odom.pose.pose.orientation,
            self.attitude_reference.pose.orientation,
        )
        if errors is None:
            return False, "invalid VINS or PX4 attitude quaternion", None
        roll_error, pitch_error = errors
        value = {
            "roll_error_deg": roll_error,
            "pitch_error_deg": pitch_error,
            "max_roll_pitch_error_deg": self.max_roll_pitch_error_deg,
            "odom_age_sec": odom_age,
            "reference_age_sec": reference_age,
        }
        ok = (
            roll_error <= self.max_roll_pitch_error_deg
            and pitch_error <= self.max_roll_pitch_error_deg
        )
        detail = "roll_error=%.2fdeg, pitch_error=%.2fdeg, max=%.2fdeg" % (
            roll_error,
            pitch_error,
            self.max_roll_pitch_error_deg,
        )
        return ok, detail, value

    def _invalidate_motion(self):
        self.direct_target = None
        self.speed_target = None
        self.speed_reference = None
        self.latest_ego_cmd = None
        self.motion_rearm_required = True

    def _spf_execution_error_locked(self, now):
        if not self.spf_execution_enabled:
            return "SPF execution gate is closed"
        return self._spf_vehicle_state_error_locked(now)

    def _spf_vehicle_state_error_locked(self, now):
        if self.spf_mavros_state is None:
            return "MAVROS state is unavailable"
        state_age = (now - self.spf_mavros_state_stamp).to_sec()
        if state_age < 0.0 or state_age > self.spf_mavros_state_timeout_sec:
            return "MAVROS state is stale"
        if not self.spf_mavros_state.connected:
            return "MAVROS is disconnected"
        if not self.spf_mavros_state.armed:
            return "PX4 is not armed"
        return None

    def _clear_spf_target_locked(self):
        if self.mode != "spf_position":
            return False
        self.direct_target = None
        self.direct_stamp = rospy.Time(0)
        self.mode = "ego_passthrough"
        return True

    def _position_active(self, now):
        if self.direct_target is None:
            return False
        timeout = self.spf_position_timeout if self.mode == "spf_position" else self.direct_position_timeout
        return timeout <= 0.0 or (now - self.direct_stamp).to_sec() <= timeout

    def _speed_active(self, now):
        return self.speed_target is not None and self.speed_reference is not None and (now - self.speed_stamp).to_sec() <= self.speed_timeout

    def _integrate_speed(self, dt):
        x, y, z, yaw = self.speed_reference
        vx, vy, vz = self.speed_target
        z = _clamp(z + vz * dt, self.min_z, self.max_z)
        self.speed_reference = (x + vx * dt, y + vy * dt, z, _normalize_angle(yaw + self.speed_yaw_rate * dt))

    def _build_position_cmd(self, position, velocity, yaw, yaw_rate=0.0):
        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "world"
        cmd.position.x, cmd.position.y, cmd.position.z = position
        cmd.velocity.x, cmd.velocity.y, cmd.velocity.z = velocity
        cmd.acceleration.x = 0.0
        cmd.acceleration.y = 0.0
        cmd.acceleration.z = 0.0
        cmd.jerk.x = 0.0
        cmd.jerk.y = 0.0
        cmd.jerk.z = 0.0
        cmd.yaw = yaw
        cmd.yaw_dot = yaw_rate
        cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        return cmd

    def _resolve_pose(self, msg, use_current_as_default):
        frame = (msg.header.frame_id or "world").strip()
        px = _finite(msg.pose.position.x)
        py = _finite(msg.pose.position.y)
        pz = _finite(msg.pose.position.z)
        yaw = _yaw_from_quaternion(msg.pose.orientation)
        if frame in WORLD_FRAMES:
            return px, py, pz, yaw
        if frame not in BODY_FRAMES:
            return None
        if self.odom is None:
            return None
        cx, cy, cz, current_yaw = self._current_pose_tuple()
        step = math.sqrt(px * px + py * py + pz * pz)
        if self.max_position_step > 0.0 and step > self.max_position_step:
            scale = self.max_position_step / step
            px *= scale
            py *= scale
            pz *= scale
        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)
        wx = cx + cos_yaw * px - sin_yaw * py
        wy = cy + sin_yaw * px + cos_yaw * py
        wz = cz + pz
        resolved_yaw = current_yaw + yaw if use_current_as_default else yaw
        return wx, wy, wz, _normalize_angle(resolved_yaw)

    def _resolve_twist(self, msg):
        frame = (msg.header.frame_id or "world").strip()
        vx = _finite(msg.twist.linear.x)
        vy = _finite(msg.twist.linear.y)
        vz = _finite(msg.twist.linear.z)
        yaw_rate = _finite(msg.twist.angular.z)
        if frame in BODY_FRAMES and self.odom is not None:
            current_yaw = self._current_pose_tuple()[3]
            cos_yaw = math.cos(current_yaw)
            sin_yaw = math.sin(current_yaw)
            vx, vy = cos_yaw * vx - sin_yaw * vy, sin_yaw * vx + cos_yaw * vy
        return vx, vy, vz, yaw_rate

    def _current_pose_tuple(self):
        pose = self.odom.pose.pose
        return (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            _yaw_from_quaternion(pose.orientation),
        )

    def _publish_status(self, mode, detail):
        self.last_status_stamp = rospy.Time.now()
        guard_ok, guard_detail, guard_value = self._attitude_guard_state(self.last_status_stamp)
        payload = {
            "mode": mode,
            "detail": detail,
            "has_odom": self.odom is not None,
            "attitude_guard_enabled": self.attitude_guard_enabled,
            "attitude_guard_ok": guard_ok,
            "attitude_guard_detail": guard_detail,
            "motion_rearm_required": self.motion_rearm_required,
        }
        with self.spf_lock:
            payload["spf_execution_enabled"] = self.spf_execution_enabled
            payload["spf_vehicle_state_error"] = self._spf_vehicle_state_error_locked(
                self.last_status_stamp
            )
        if guard_value is not None:
            payload["attitude_guard"] = guard_value
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main():
    rospy.init_node("gameuav_control_interface")
    ControlInterfaceNode()
    rospy.spin()


if __name__ == "__main__":
    main()

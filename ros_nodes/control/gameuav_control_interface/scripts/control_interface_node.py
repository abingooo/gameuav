#!/usr/bin/env python3

import json
import math

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Empty, Float64, String


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

        odom_topic = rospy.get_param("~odom_topic", "/vins_fusion/imu_propagate")
        ego_cmd_topic = rospy.get_param("~ego_position_cmd_topic", "/control/ego_position_cmd")
        position_topic = rospy.get_param("~direct_position_topic", "/control/position")
        spf_position_topic = rospy.get_param("~spf_position_topic", "/control/spf_position")
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
        rospy.Subscriber(ego_cmd_topic, PositionCommand, self.ego_cmd_cb, queue_size=20)
        rospy.Subscriber(position_topic, PoseStamped, self.position_cb, queue_size=10)
        rospy.Subscriber(spf_position_topic, PoseStamped, self.spf_position_cb, queue_size=10)
        rospy.Subscriber(speed_topic, TwistStamped, self.speed_cb, queue_size=10)
        rospy.Subscriber(ego_position_topic, PoseStamped, self.ego_position_cb, queue_size=10)
        rospy.Subscriber(stop_topic, Empty, self.stop_cb, queue_size=10)

        period = 1.0 / max(1.0, float(self.rate_hz))
        self.timer = rospy.Timer(rospy.Duration(period), self.timer_cb)
        rospy.loginfo(
            "[control_interface] ready: ego=%s position=%s spf_position=%s speed=%s output=%s",
            ego_cmd_topic,
            position_topic,
            spf_position_topic,
            speed_topic,
            output_topic,
        )

    def odom_cb(self, msg):
        self.odom = msg
        self.odom_stamp = rospy.Time.now()

    def ego_cmd_cb(self, msg):
        self.latest_ego_cmd = msg
        self.latest_ego_stamp = rospy.Time.now()
        if not self._direct_active(rospy.Time.now()):
            self.mode = "ego_passthrough"
            self.output_pub.publish(msg)

    def position_cb(self, msg):
        self._accept_position(msg, "direct_position")

    def spf_position_cb(self, msg):
        self._accept_position(msg, "spf_position")

    def _accept_position(self, msg, mode):
        resolved = self._resolve_pose(msg, use_current_as_default=False)
        if resolved is None:
            self._publish_status("reject_%s" % mode, "odom unavailable or invalid frame")
            return
        x, y, z, yaw = resolved
        self.direct_target = (x, y, _clamp(z, self.min_z, self.max_z))
        self.direct_yaw = yaw
        self.direct_stamp = rospy.Time.now()
        self.mode = mode
        self._publish_status(mode, "accepted")

    def speed_cb(self, msg):
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
        self.speed_stamp = rospy.Time.now()
        self.mode = "speed"
        self._publish_status("speed", "accepted")

    def ego_position_cb(self, msg):
        resolved = self._resolve_pose(msg, use_current_as_default=True)
        if resolved is None:
            self._publish_status("reject_ego_position", "odom unavailable or invalid frame")
            return
        x, y, z, yaw = resolved
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "world"
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = _clamp(z, self.min_z, self.max_z)
        _yaw_to_quaternion(yaw, goal.pose.orientation)
        self.goal_pub.publish(goal)
        yaw_msg = Float64()
        yaw_msg.data = math.degrees(yaw)
        self.yaw_pub.publish(yaw_msg)
        self.mode = "ego_passthrough"
        self._publish_status("ego_position", "published planning goal")

    def stop_cb(self, _msg):
        if self.odom is None:
            self.mode = "ego_passthrough"
            self.direct_target = None
            self.speed_target = None
            self._publish_status("stop", "odom unavailable; returned to ego passthrough")
            return
        x, y, z, yaw = self._current_pose_tuple()
        self.direct_target = (x, y, _clamp(z, self.min_z, self.max_z))
        self.direct_yaw = yaw
        self.direct_stamp = rospy.Time.now()
        self.speed_target = None
        self.mode = "direct_position"
        self._publish_status("stop", "holding current position")

    def timer_cb(self, event):
        now = rospy.Time.now()
        dt = max(0.0, min(0.1, (now - self.last_timer_stamp).to_sec()))
        self.last_timer_stamp = now

        if self.mode in {"direct_position", "spf_position"} and self._position_active(now):
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
        payload = {
            "mode": mode,
            "detail": detail,
            "has_odom": self.odom is not None,
        }
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main():
    rospy.init_node("gameuav_control_interface")
    ControlInterfaceNode()
    rospy.spin()


if __name__ == "__main__":
    main()

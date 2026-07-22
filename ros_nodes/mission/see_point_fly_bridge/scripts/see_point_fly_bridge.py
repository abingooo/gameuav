#!/usr/bin/env python3

import base64
import json
import math
import threading
import time
import urllib.error
import urllib.request

import cv2
import rospy
import tf.transformations
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Bool, Empty, String


class SeePointFlyBridge:
    def __init__(self):
        self.worker_url = rospy.get_param("~worker_url", "http://127.0.0.1:9310/infer")
        self.worker_timeout_sec = float(rospy.get_param("~worker_timeout_sec", 30.0))

        self.image_topic = rospy.get_param("~image_topic", "/rgb1/image_raw")
        self.odom_topic = rospy.get_param("~odom_topic", "/vins_fusion/imu_propagate")
        self.command_topic = rospy.get_param("~command_topic", "/spf/user_command")
        self.action_topic = rospy.get_param("~action_topic", "/spf/action_command")
        self.enable_topic = rospy.get_param("~enable_topic", "/spf/enable")
        self.goal_topic = rospy.get_param("~goal_topic", "/control/ego_position")
        self.stop_topic = rospy.get_param("~stop_topic", "/control/stop")
        self.status_topic = rospy.get_param("~status_topic", "/spf/status")
        self.last_goal_topic = rospy.get_param("~last_goal_topic", "/spf/last_goal")
        self.mavros_state_topic = rospy.get_param("~mavros_state_topic", "/mavros/state")

        self.manual_enable_required = bool(rospy.get_param("~manual_enable_required", True))
        self.require_armed_for_execution = bool(
            rospy.get_param("~require_armed_for_execution", True)
        )
        self.mavros_state_timeout_sec = float(
            rospy.get_param("~mavros_state_timeout_sec", 2.5)
        )
        self.max_step_xy = float(rospy.get_param("~max_step_xy", 1.5))
        self.max_step_z = float(rospy.get_param("~max_step_z", 0.3))
        self.min_goal_distance_xy = float(rospy.get_param("~min_goal_distance_xy", 0.8))
        self.min_z = float(rospy.get_param("~min_z", 0.4))
        self.max_z = float(rospy.get_param("~max_z", 1.5))
        self.rate_limit_sec = float(rospy.get_param("~rate_limit_sec", 2.0))
        self.preview_max_age_sec = float(rospy.get_param("~preview_max_age_sec", 30.0))
        self.image_timeout_sec = float(rospy.get_param("~image_timeout_sec", 2.0))
        self.odom_timeout_sec = float(rospy.get_param("~odom_timeout_sec", 1.0))
        self.max_abs_odom_position = float(rospy.get_param("~max_abs_odom_position", 100.0))
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.goal_projection_enabled = bool(rospy.get_param("~goal_projection_enabled", False))
        self.occupancy_topic = rospy.get_param(
            "~occupancy_topic",
            "/drone_0_ego_planner_node/grid_map/occupancy_inflate",
        )
        self.occupancy_timeout_sec = float(rospy.get_param("~occupancy_timeout_sec", 1.0))
        self.goal_clearance_radius = float(rospy.get_param("~goal_clearance_radius", 0.25))
        self.goal_projection_step = float(rospy.get_param("~goal_projection_step", 0.1))
        self.goal_projection_min_backoff = float(rospy.get_param("~goal_projection_min_backoff", 0.4))
        self.goal_projection_max_backoff = float(rospy.get_param("~goal_projection_max_backoff", 1.2))
        self.goal_projection_max_points = int(rospy.get_param("~goal_projection_max_points", 20000))

        self.bridge = CvBridge()
        self.execution_lock = threading.RLock()
        self.enabled = not self.manual_enable_required
        self.last_image_msg = None
        self.last_image_time = None
        self.last_odom_msg = None
        self.last_odom_time = None
        self.last_mavros_state = None
        self.last_mavros_state_time = None
        self.last_goal_time = 0.0
        self.last_occupancy_points = []
        self.last_occupancy_time = None
        self.last_projection = None

        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=10)
        self.stop_pub = rospy.Publisher(self.stop_topic, Empty, queue_size=1)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10, latch=True)
        self.last_goal_pub = rospy.Publisher(self.last_goal_topic, String, queue_size=1, latch=True)
        rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=10)
        rospy.Subscriber(
            self.mavros_state_topic,
            State,
            self.mavros_state_callback,
            queue_size=10,
        )
        rospy.Subscriber(self.command_topic, String, self.command_callback, queue_size=10)
        rospy.Subscriber(self.action_topic, String, self.action_command_callback, queue_size=10)
        rospy.Subscriber(self.enable_topic, Bool, self.enable_callback, queue_size=10)
        if self.goal_projection_enabled:
            rospy.Subscriber(self.occupancy_topic, PointCloud2, self.occupancy_callback, queue_size=1)
        self.watchdog_timer = rospy.Timer(rospy.Duration(0.1), self.watchdog_callback)

        self.publish_status("ready")

    def image_callback(self, msg):
        self.last_image_msg = msg
        self.last_image_time = time.time()

    def odom_callback(self, msg):
        self.last_odom_msg = msg
        self.last_odom_time = time.time()

    def mavros_state_callback(self, msg):
        should_stop = False
        with self.execution_lock:
            previous = self.last_mavros_state
            self.last_mavros_state = msg
            self.last_mavros_state_time = time.time()
            lost_flight_state = not msg.connected or not msg.armed
            should_report = lost_flight_state and (
                self.enabled
                or (previous is not None and previous.connected and previous.armed)
            )
            if lost_flight_state:
                should_stop = self.enabled
                self.enabled = False
            if should_report:
                self.publish_status(
                    "execution gate closed: MAVROS disconnected or PX4 disarmed"
                )
        if should_stop:
            self.stop_pub.publish(Empty())

    def watchdog_callback(self, _event):
        now = time.time()
        should_stop = False
        with self.execution_lock:
            if not self.enabled:
                return
            vehicle_error = self._vehicle_state_error_locked(now)
            if not vehicle_error:
                return
            self.enabled = False
            should_stop = True
            self.publish_status("execution gate closed: %s" % vehicle_error)
        if should_stop:
            self.stop_pub.publish(Empty())

    def occupancy_callback(self, msg):
        points = []
        try:
            for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                if len(points) >= self.goal_projection_max_points:
                    break
                x, y, z = float(point[0]), float(point[1]), float(point[2])
                if all(math.isfinite(value) for value in (x, y, z)):
                    points.append((x, y, z))
        except Exception as exc:
            rospy.logwarn("see_point_fly_bridge: failed to read occupancy cloud: %s", exc)
            return
        self.last_occupancy_points = points
        self.last_occupancy_time = time.time()

    def enable_callback(self, msg):
        should_stop = False
        with self.execution_lock:
            requested = bool(msg.data)
            if requested:
                vehicle_error = self._vehicle_state_error_locked(time.time())
                if vehicle_error:
                    should_stop = self.enabled
                    self.enabled = False
                    self.publish_status("enable rejected: %s" % vehicle_error)
                else:
                    self.enabled = True
                    self.publish_status("enabled=True")
            else:
                should_stop = self.enabled
                self.enabled = False
                self.publish_status("enabled=False")
        if should_stop:
            self.stop_pub.publish(Empty())

    def command_callback(self, msg):
        command = (msg.data or "").strip()
        if not self.execution_ready(command, time.time()):
            return

        try:
            image_jpeg_b64 = self.encode_latest_image()
            response = self.call_worker(command, image_jpeg_b64)
            action = self.extract_action(response)
        except Exception as exc:
            self.publish_status("rejected: %s" % exc)
            return
        self.publish_action(command, action)

    def action_command_callback(self, msg):
        try:
            payload = json.loads(msg.data or "")
            command, action = self.extract_cached_action(payload, time.time())
        except Exception as exc:
            self.publish_status("rejected cached action: %s" % exc)
            return
        self.publish_action(command, action)

    def extract_cached_action(self, payload, now):
        if not isinstance(payload, dict):
            raise RuntimeError("payload must be an object")
        if payload.get("schema") != "gameuav.spf.cached_action.v1":
            raise RuntimeError("unsupported payload schema")
        command = str(payload.get("command") or "").strip()
        if not command:
            raise RuntimeError("empty command")
        try:
            completed_at = float(payload.get("completed_at"))
        except (TypeError, ValueError):
            raise RuntimeError("missing preview completion time")
        if not math.isfinite(completed_at):
            raise RuntimeError("invalid preview completion time")
        age_sec = now - completed_at
        if age_sec < -5.0:
            raise RuntimeError("preview completion time is in the future")
        if age_sec > self.preview_max_age_sec:
            raise RuntimeError("preview expired: %.1fs" % age_sec)
        action = payload.get("action")
        if not isinstance(action, dict):
            raise RuntimeError("missing action")
        return command, action

    def execution_ready(self, command, now):
        with self.execution_lock:
            return self._execution_ready_locked(command, now)

    def _execution_ready_locked(self, command, now):
        if not command:
            self.publish_status("rejected: empty command")
            return False
        if self.manual_enable_required and not self.enabled:
            self.publish_status("rejected: /spf/enable is false")
            return False
        if now - self.last_goal_time < self.rate_limit_sec:
            self.publish_status("rejected: rate limited")
            return False
        vehicle_error = self._vehicle_state_error_locked(now)
        if vehicle_error:
            if self.enabled:
                self.enabled = False
            self.publish_status("rejected: %s" % vehicle_error)
            return False
        return self.inputs_ready(now)

    def vehicle_state_error(self, now):
        with self.execution_lock:
            return self._vehicle_state_error_locked(now)

    def _vehicle_state_error_locked(self, now):
        if not self.require_armed_for_execution:
            return None
        if self.last_mavros_state is None or self.last_mavros_state_time is None:
            return "no MAVROS state"
        state_age = now - self.last_mavros_state_time
        if state_age < 0.0 or state_age > self.mavros_state_timeout_sec:
            return "stale MAVROS state"
        if not self.last_mavros_state.connected:
            return "MAVROS is disconnected"
        if not self.last_mavros_state.armed:
            return "PX4 is not armed"
        return None

    def publish_action(self, command, action):
        if not self.execution_ready(command, time.time()):
            return False
        try:
            goal, yaw_deg = self.action_to_goal(action)
        except Exception as exc:
            self.publish_status("rejected: %s" % exc)
            return False

        if goal is None:
            self.publish_status("rejected: action produced no goal")
            return False

        raw_goal = self.copy_goal(goal)
        goal = self.project_goal_if_needed(goal)
        with self.execution_lock:
            if not self._execution_ready_locked(command, time.time()):
                return False
            self.goal_pub.publish(goal)
            self.last_goal_time = time.time()
            self.publish_last_goal(
                command,
                action,
                goal,
                yaw_deg,
                raw_goal=raw_goal,
                projection=self.last_projection,
            )
            if self.last_projection and self.last_projection.get("adjusted"):
                self.publish_status(
                    "published projected goal: %s (backoff %.2fm)"
                    % (command, self.last_projection.get("backoff_m", 0.0))
                )
            else:
                self.publish_status("published goal: %s" % command)
        return True

    def inputs_ready(self, now):
        if self.last_image_msg is None or self.last_image_time is None:
            self.publish_status("rejected: no image")
            return False
        if now - self.last_image_time > self.image_timeout_sec:
            self.publish_status("rejected: stale image")
            return False
        if self.last_odom_msg is None or self.last_odom_time is None:
            self.publish_status("rejected: no odom")
            return False
        if now - self.last_odom_time > self.odom_timeout_sec:
            self.publish_status("rejected: stale odom")
            return False
        if not self.odom_position_sane(self.last_odom_msg):
            self.publish_status("rejected: invalid odom position")
            return False
        return True

    def odom_position_sane(self, msg):
        position = msg.pose.pose.position
        values = (position.x, position.y, position.z)
        return all(math.isfinite(value) and abs(value) <= self.max_abs_odom_position for value in values)

    def encode_latest_image(self):
        cv_image = self.bridge.imgmsg_to_cv2(self.last_image_msg, desired_encoding="bgr8")
        ok, encoded = cv2.imencode(".jpg", cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            raise RuntimeError("failed to encode image")
        return base64.b64encode(encoded.tobytes()).decode("ascii")

    def call_worker(self, command, image_jpeg_b64):
        payload = {
            "command": command,
            "image_jpeg_b64": image_jpeg_b64,
            "odom": self.odom_payload(),
        }
        data = json.dumps(payload).encode("utf-8")
        request = urllib.request.Request(
            self.worker_url,
            data=data,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.worker_timeout_sec) as response:
                return json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            body = exc.read().decode("utf-8", "replace")
            raise RuntimeError("worker http %s: %s" % (exc.code, body))
        except urllib.error.URLError as exc:
            raise RuntimeError("worker unavailable: %s" % exc)

    def odom_payload(self):
        pose = self.last_odom_msg.pose.pose
        twist = self.last_odom_msg.twist.twist
        return {
            "position": {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
            },
            "orientation": {
                "x": pose.orientation.x,
                "y": pose.orientation.y,
                "z": pose.orientation.z,
                "w": pose.orientation.w,
            },
            "linear_velocity": {
                "x": twist.linear.x,
                "y": twist.linear.y,
                "z": twist.linear.z,
            },
        }

    def extract_action(self, response):
        if not isinstance(response, dict):
            raise RuntimeError("worker returned non-object response")
        if not response.get("ok"):
            raise RuntimeError(response.get("error") or "worker returned ok=false")
        action = response.get("action")
        if not isinstance(action, dict):
            raise RuntimeError("worker response missing action")
        return action

    def action_to_goal(self, action):
        if bool(action.get("yaw_only", False)):
            yaw_deg = float(action.get("yaw_deg", 0.0))
            if not math.isfinite(yaw_deg):
                raise RuntimeError("yaw angle must be finite")
            pose = self.last_odom_msg.pose.pose
            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = self.frame_id
            goal.pose.position = pose.position
            # SPF/Tello defines positive image-space yaw as a right turn; ENU yaw is left-positive.
            target_yaw = self.current_yaw_rad() - math.radians(yaw_deg)
            quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, target_yaw)
            goal.pose.orientation.x = quaternion[0]
            goal.pose.orientation.y = quaternion[1]
            goal.pose.orientation.z = quaternion[2]
            goal.pose.orientation.w = quaternion[3]
            return goal, yaw_deg

        dx_right = float(action.get("dx", 0.0))
        dy_forward = float(action.get("dy", 0.0))
        dz_up = float(action.get("dz", 0.0))
        if not all(math.isfinite(value) for value in (dx_right, dy_forward, dz_up)):
            raise RuntimeError("action vector must be finite")

        body_x_forward = dy_forward
        body_y_left = -dx_right
        body_z_up = max(-self.max_step_z, min(self.max_step_z, dz_up))

        xy_norm = math.hypot(body_x_forward, body_y_left)
        if xy_norm < self.min_goal_distance_xy:
            raise RuntimeError("target too close: %.2fm" % xy_norm)
        if xy_norm > self.max_step_xy:
            scale = self.max_step_xy / xy_norm
            body_x_forward *= scale
            body_y_left *= scale
            xy_norm = self.max_step_xy

        pose = self.last_odom_msg.pose.pose
        yaw = self.current_yaw_rad()
        world_dx = math.cos(yaw) * body_x_forward - math.sin(yaw) * body_y_left
        world_dy = math.sin(yaw) * body_x_forward + math.cos(yaw) * body_y_left
        world_z = max(self.min_z, min(self.max_z, pose.position.z + body_z_up))
        relative_yaw_right = math.atan2(dx_right, dy_forward) if xy_norm > 1e-6 else 0.0
        target_yaw = yaw - relative_yaw_right

        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.frame_id
        goal.pose.position.x = pose.position.x + world_dx
        goal.pose.position.y = pose.position.y + world_dy
        goal.pose.position.z = world_z
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, target_yaw)
        goal.pose.orientation.x = quaternion[0]
        goal.pose.orientation.y = quaternion[1]
        goal.pose.orientation.z = quaternion[2]
        goal.pose.orientation.w = quaternion[3]
        return goal, math.degrees(relative_yaw_right)

    def copy_goal(self, goal):
        copied = PoseStamped()
        copied.header.stamp = goal.header.stamp
        copied.header.frame_id = goal.header.frame_id
        copied.pose.position.x = goal.pose.position.x
        copied.pose.position.y = goal.pose.position.y
        copied.pose.position.z = goal.pose.position.z
        copied.pose.orientation = goal.pose.orientation
        return copied

    def project_goal_if_needed(self, goal):
        self.last_projection = {
            "enabled": self.goal_projection_enabled,
            "adjusted": False,
            "reason": "disabled" if not self.goal_projection_enabled else "not_checked",
        }
        if not self.goal_projection_enabled:
            return goal
        now = time.time()
        if self.last_occupancy_time is None or now - self.last_occupancy_time > self.occupancy_timeout_sec:
            self.last_projection["reason"] = "occupancy_unavailable"
            return goal
        if not self.last_occupancy_points:
            self.last_projection["reason"] = "occupancy_empty"
            return goal

        original = (
            goal.pose.position.x,
            goal.pose.position.y,
            goal.pose.position.z,
        )
        original_nearest = self.nearest_occupancy_distance(original)
        self.last_projection.update(
            {
                "clearance_radius": self.goal_clearance_radius,
                "original_nearest_occupancy_m": original_nearest,
            }
        )
        if original_nearest is not None and original_nearest >= self.goal_clearance_radius:
            self.last_projection["reason"] = "already_clear"
            return goal

        odom_position = self.last_odom_msg.pose.pose.position
        direction_x = odom_position.x - original[0]
        direction_y = odom_position.y - original[1]
        direction_norm = math.hypot(direction_x, direction_y)
        if direction_norm < 1e-3:
            self.last_projection["reason"] = "no_xy_direction"
            return goal
        unit_x = direction_x / direction_norm
        unit_y = direction_y / direction_norm

        candidate = self.find_projected_goal(original, unit_x, unit_y)
        if candidate is None:
            self.last_projection["reason"] = "no_clear_candidate"
            return goal

        projected_goal = self.copy_goal(goal)
        projected_goal.header.stamp = rospy.Time.now()
        projected_goal.pose.position.x = candidate["x"]
        projected_goal.pose.position.y = candidate["y"]
        projected_goal.pose.position.z = candidate["z"]
        self.last_projection.update(candidate)
        self.last_projection["adjusted"] = True
        self.last_projection["reason"] = "projected_from_occupied_goal"
        return projected_goal

    def find_projected_goal(self, original, unit_x, unit_y):
        step = max(0.02, self.goal_projection_step)
        min_backoff = max(0.0, self.goal_projection_min_backoff)
        max_backoff = max(min_backoff, self.goal_projection_max_backoff)
        steps = int(math.floor((max_backoff - min_backoff) / step)) + 1
        for index in range(steps):
            backoff = min_backoff + index * step
            candidate = (
                original[0] + unit_x * backoff,
                original[1] + unit_y * backoff,
                original[2],
            )
            nearest = self.nearest_occupancy_distance(candidate)
            if nearest is not None and nearest >= self.goal_clearance_radius:
                return {
                    "x": candidate[0],
                    "y": candidate[1],
                    "z": candidate[2],
                    "backoff_m": backoff,
                    "projected_nearest_occupancy_m": nearest,
                }
        return None

    def nearest_occupancy_distance(self, position):
        if not self.last_occupancy_points:
            return None
        px, py, pz = position
        nearest_sq = None
        for ox, oy, oz in self.last_occupancy_points:
            dx = ox - px
            dy = oy - py
            dz = oz - pz
            dist_sq = dx * dx + dy * dy + dz * dz
            if nearest_sq is None or dist_sq < nearest_sq:
                nearest_sq = dist_sq
        if nearest_sq is None:
            return None
        return math.sqrt(nearest_sq)

    def current_yaw_rad(self):
        q = self.last_odom_msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        rospy.loginfo("see_point_fly_bridge: %s", text)

    def publish_last_goal(self, command, action, goal, yaw_deg, raw_goal=None, projection=None):
        pose = self.last_odom_msg.pose.pose
        payload = {
            "stamp": time.time(),
            "command": command,
            "action": {
                "dx": action.get("dx"),
                "dy": action.get("dy"),
                "dz": action.get("dz"),
                "yaw_deg": yaw_deg,
                "yaw_only": bool(action.get("yaw_only", False)),
                "screen_x": action.get("screen_x"),
                "screen_y": action.get("screen_y"),
            },
            "odom_position": {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
            },
            "goal": {
                "x": goal.pose.position.x,
                "y": goal.pose.position.y,
                "z": goal.pose.position.z,
                "yaw": tf.transformations.euler_from_quaternion(
                    [
                        goal.pose.orientation.x,
                        goal.pose.orientation.y,
                        goal.pose.orientation.z,
                        goal.pose.orientation.w,
                    ]
                )[2],
                "frame_id": goal.header.frame_id,
            },
        }
        if raw_goal is not None:
            payload["raw_goal"] = {
                "x": raw_goal.pose.position.x,
                "y": raw_goal.pose.position.y,
                "z": raw_goal.pose.position.z,
                "frame_id": raw_goal.header.frame_id,
            }
        if projection is not None:
            payload["projection"] = projection
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.last_goal_pub.publish(msg)


def main():
    rospy.init_node("see_point_fly_bridge")
    SeePointFlyBridge()
    rospy.spin()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import json
import math
import threading
import time
import uuid

import rospy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


ACTIVE_STATES = {"WAITING_GOAL", "WAITING_ARRIVAL", "WAITING_NEXT"}
TERMINAL_STATES = {"SUCCESS", "TIMEOUT", "ABORTED", "ERROR"}


class TaskLoopError(RuntimeError):
    pass


class TaskLoop:
    """ROS-independent SPF task loop state.

    The loop deliberately does not arm, take off, land, or cancel control. It only
    requests the next SPF action after the previous local goal has settled.
    """

    def __init__(
        self,
        goal_ack_timeout_sec=95.0,
        goal_timeout_sec=45.0,
        task_timeout_sec=300.0,
        cycle_delay_sec=1.0,
        arrival_settle_sec=1.0,
        goal_tolerance_xy=0.25,
        goal_tolerance_z=0.20,
        arrival_max_speed=0.25,
        odom_timeout_sec=1.0,
        min_start_z=0.4,
        max_start_z=1.5,
        start_max_speed=0.5,
        allow_tabletop_start_disarmed=False,
        tabletop_min_start_z=-0.2,
        max_cycles=20,
    ):
        self.goal_ack_timeout_sec = float(goal_ack_timeout_sec)
        self.goal_timeout_sec = float(goal_timeout_sec)
        self.task_timeout_sec = float(task_timeout_sec)
        self.cycle_delay_sec = float(cycle_delay_sec)
        self.arrival_settle_sec = float(arrival_settle_sec)
        self.goal_tolerance_xy = float(goal_tolerance_xy)
        self.goal_tolerance_z = float(goal_tolerance_z)
        self.arrival_max_speed = float(arrival_max_speed)
        self.odom_timeout_sec = float(odom_timeout_sec)
        self.min_start_z = float(min_start_z)
        self.max_start_z = float(max_start_z)
        self.start_max_speed = float(start_max_speed)
        self.allow_tabletop_start_disarmed = bool(allow_tabletop_start_disarmed)
        self.tabletop_min_start_z = float(tabletop_min_start_z)
        self.max_cycles = int(max_cycles)

        self.enabled = False
        self.state = "DISABLED"
        self.task_id = None
        self.command = None
        self.reason = "task execution is disabled"
        self.started_at = None
        self.updated_at = None
        self.cycle_count = 0
        self.cycle_requested_at = None
        self.goal_ack_deadline = None
        self.goal_deadline = None
        self.current_goal = None
        self.arrival_since = None
        self.next_cycle_at = None
        self.distance_xy = None
        self.distance_z = None
        self.speed = None
        self.last_rejection = None
        self.last_rejection_at = None

    def set_enabled(self, enabled, now):
        now = float(now)
        enabled = bool(enabled)
        if not enabled:
            if self.state in ACTIVE_STATES:
                self._finish("ABORTED", "task execution disabled", now)
            self.enabled = False
            self.state = "DISABLED"
            self.reason = "task execution is disabled"
            self.last_rejection = None
            self.last_rejection_at = None
            self.updated_at = now
            return []

        self.enabled = True
        if self.state == "DISABLED":
            self.state = "IDLE"
            self.reason = "ready for a task command"
            self.updated_at = now
        return []

    def start(self, command, now, odom, vehicle_state=None, task_id=None):
        now = float(now)
        command = str(command or "").strip()
        if not self.enabled:
            raise TaskLoopError("task execution is disabled")
        if self.state in ACTIVE_STATES:
            raise TaskLoopError("task already active")
        if not command:
            raise TaskLoopError("empty task command")
        self._validate_start_odom(now, odom, vehicle_state)

        self.task_id = task_id or uuid.uuid4().hex[:12]
        self.command = command
        self.started_at = now
        self.updated_at = now
        self.cycle_count = 0
        self.current_goal = None
        self.last_rejection = None
        self.last_rejection_at = None
        self.arrival_since = None
        self.next_cycle_at = None
        self.distance_xy = None
        self.distance_z = None
        self.speed = float(odom["speed"])
        return self._request_cycle(now)

    def receive_goal(self, payload, now):
        now = float(now)
        if self.state != "WAITING_GOAL" or not isinstance(payload, dict):
            return False
        if str(payload.get("command") or "").strip() != self.command:
            return False
        try:
            stamp = float(payload["stamp"])
            goal = payload["goal"]
            parsed_goal = {
                "x": float(goal["x"]),
                "y": float(goal["y"]),
                "z": float(goal["z"]),
            }
        except (KeyError, TypeError, ValueError):
            return False
        if stamp < self.cycle_requested_at - 0.25:
            return False
        if not all(math.isfinite(value) for value in parsed_goal.values()):
            return False

        self.current_goal = parsed_goal
        self.goal_deadline = now + self.goal_timeout_sec
        self.goal_ack_deadline = None
        self.arrival_since = None
        self.state = "WAITING_ARRIVAL"
        self.reason = "direct position target accepted"
        self.updated_at = now
        return True

    def control(self, command, now):
        now = float(now)
        command = str(command or "").strip().lower()
        if command in {"abort", "stop"}:
            if self.state in ACTIVE_STATES:
                self._finish("ABORTED", "operator aborted task", now)
            return []
        if command in {"complete", "success"}:
            if self.state not in ACTIVE_STATES:
                raise TaskLoopError("no active task to complete")
            self._finish("SUCCESS", "operator confirmed task completion", now)
            return []
        if command == "reset":
            if self.state in ACTIVE_STATES:
                raise TaskLoopError("abort the active task before reset")
            self._clear_task()
            self.last_rejection = None
            self.last_rejection_at = None
            self.state = "IDLE" if self.enabled else "DISABLED"
            self.reason = "ready for a task command" if self.enabled else "task execution is disabled"
            self.updated_at = now
            return []
        raise TaskLoopError("unsupported task control: %s" % command)

    def tick(self, now, odom):
        now = float(now)
        if self.state not in ACTIVE_STATES:
            return []
        if now - self.started_at > self.task_timeout_sec:
            self._finish("TIMEOUT", "task timeout", now)
            return []
        if self.state == "WAITING_GOAL":
            if now > self.goal_ack_deadline:
                self._finish("ERROR", "timed out waiting for SPF position target", now)
            return []
        if self.state == "WAITING_NEXT":
            if now >= self.next_cycle_at:
                return self._request_cycle(now)
            return []

        if not self._odom_fresh(now, odom):
            self._finish("ERROR", "odometry became stale", now)
            return []
        if now > self.goal_deadline:
            self._finish("TIMEOUT", "direct position target timeout", now)
            return []

        dx = float(odom["x"]) - self.current_goal["x"]
        dy = float(odom["y"]) - self.current_goal["y"]
        dz = float(odom["z"]) - self.current_goal["z"]
        self.distance_xy = math.hypot(dx, dy)
        self.distance_z = abs(dz)
        self.speed = float(odom["speed"])
        arrived = (
            self.distance_xy <= self.goal_tolerance_xy
            and self.distance_z <= self.goal_tolerance_z
            and self.speed <= self.arrival_max_speed
        )
        if not arrived:
            self.arrival_since = None
            return []
        if self.arrival_since is None:
            self.arrival_since = now
            self.reason = "local goal reached; waiting to settle"
            self.updated_at = now
            return []
        if now - self.arrival_since < self.arrival_settle_sec:
            return []
        if self.cycle_count >= self.max_cycles:
            self._finish("TIMEOUT", "maximum SPF action cycles reached", now)
            return []

        self.state = "WAITING_NEXT"
        self.reason = "local goal settled; scheduling next SPF inference"
        self.next_cycle_at = now + self.cycle_delay_sec
        self.updated_at = now
        return []

    def status(self):
        return {
            "schema": "gameuav.spf.task_status.v1",
            "enabled": self.enabled,
            "active": self.state in ACTIVE_STATES,
            "state": self.state,
            "task_id": self.task_id,
            "command": self.command,
            "reason": self.reason,
            "started_at": self.started_at,
            "updated_at": self.updated_at,
            "cycle_count": self.cycle_count,
            "current_goal": self.current_goal,
            "distance_xy": self.distance_xy,
            "distance_z": self.distance_z,
            "speed": self.speed,
            "last_rejection": self.last_rejection,
            "last_rejection_at": self.last_rejection_at,
            "tabletop_start_allowed": self.allow_tabletop_start_disarmed,
        }

    def record_rejection(self, reason, now):
        self.last_rejection = str(reason or "")
        self.last_rejection_at = float(now)

    def _request_cycle(self, now):
        self.cycle_count += 1
        self.cycle_requested_at = now
        self.goal_ack_deadline = now + self.goal_ack_timeout_sec
        self.goal_deadline = None
        self.current_goal = None
        self.arrival_since = None
        self.next_cycle_at = None
        self.distance_xy = None
        self.distance_z = None
        self.state = "WAITING_GOAL"
        self.reason = "requesting SPF inference for action %d" % self.cycle_count
        self.updated_at = now
        return [("publish_command", self.command)]

    def _finish(self, state, reason, now):
        if state not in TERMINAL_STATES:
            raise ValueError("invalid terminal state")
        self.state = state
        self.reason = reason
        self.updated_at = now
        self.goal_ack_deadline = None
        self.goal_deadline = None
        self.arrival_since = None
        self.next_cycle_at = None

    def _clear_task(self):
        self.task_id = None
        self.command = None
        self.started_at = None
        self.cycle_count = 0
        self.cycle_requested_at = None
        self.goal_ack_deadline = None
        self.goal_deadline = None
        self.current_goal = None
        self.arrival_since = None
        self.next_cycle_at = None
        self.distance_xy = None
        self.distance_z = None
        self.speed = None

    def _validate_start_odom(self, now, odom, vehicle_state=None):
        if not self._odom_fresh(now, odom):
            raise TaskLoopError("fresh odometry is required")
        z = float(odom["z"])
        speed = float(odom["speed"])
        if z < self.min_start_z or z > self.max_start_z:
            if not self._tabletop_start_ok(now, z, vehicle_state):
                raise TaskLoopError("vehicle must already be hovering inside the configured altitude range")
        if speed > self.start_max_speed:
            raise TaskLoopError("vehicle speed is too high to start SPF task")

    def _tabletop_start_ok(self, now, z, vehicle_state):
        if not self.allow_tabletop_start_disarmed:
            return False
        if z < self.tabletop_min_start_z or z > self.max_start_z:
            return False
        if not isinstance(vehicle_state, dict):
            return False
        try:
            stamp = float(vehicle_state["stamp"])
            armed = bool(vehicle_state["armed"])
        except (KeyError, TypeError, ValueError):
            return False
        if now - stamp > self.odom_timeout_sec:
            return False
        return not armed

    def _odom_fresh(self, now, odom):
        if not isinstance(odom, dict):
            return False
        try:
            values = [float(odom[key]) for key in ("stamp", "x", "y", "z", "speed")]
        except (KeyError, TypeError, ValueError):
            return False
        return all(math.isfinite(value) for value in values) and now - values[0] <= self.odom_timeout_sec


class SpfTaskExecutorNode:
    def __init__(self):
        self.start_topic = rospy.get_param("~start_topic", "/spf/task/start")
        self.control_topic = rospy.get_param("~control_topic", "/spf/task/control")
        self.enable_topic = rospy.get_param("~enable_topic", "/spf/task/enable")
        self.status_topic = rospy.get_param("~status_topic", "/spf/task/status")
        self.command_topic = rospy.get_param("~command_topic", "/spf/user_command")
        self.last_goal_topic = rospy.get_param("~last_goal_topic", "/spf/last_goal")
        self.odom_topic = rospy.get_param("~odom_topic", "/vins_fusion/imu_propagate")
        self.mavros_state_topic = rospy.get_param("~mavros_state_topic", "/mavros/state")

        self.loop = TaskLoop(
            goal_ack_timeout_sec=rospy.get_param("~goal_ack_timeout_sec", 95.0),
            goal_timeout_sec=rospy.get_param("~goal_timeout_sec", 45.0),
            task_timeout_sec=rospy.get_param("~task_timeout_sec", 300.0),
            cycle_delay_sec=rospy.get_param("~cycle_delay_sec", 1.0),
            arrival_settle_sec=rospy.get_param("~arrival_settle_sec", 1.0),
            goal_tolerance_xy=rospy.get_param("~goal_tolerance_xy", 0.25),
            goal_tolerance_z=rospy.get_param("~goal_tolerance_z", 0.20),
            arrival_max_speed=rospy.get_param("~arrival_max_speed", 0.25),
            odom_timeout_sec=rospy.get_param("~odom_timeout_sec", 1.0),
            min_start_z=rospy.get_param("~min_start_z", 0.4),
            max_start_z=rospy.get_param("~max_start_z", 1.5),
            start_max_speed=rospy.get_param("~start_max_speed", 0.5),
            allow_tabletop_start_disarmed=rospy.get_param("~allow_tabletop_start_disarmed", False),
            tabletop_min_start_z=rospy.get_param("~tabletop_min_start_z", -0.2),
            max_cycles=rospy.get_param("~max_cycles", 20),
        )
        self.lock = threading.RLock()
        self.odom = None
        self.vehicle_state = None
        self.last_status_json = None
        self.last_status_signature = None
        self.last_status_publish_at = 0.0

        self.command_pub = rospy.Publisher(self.command_topic, String, queue_size=1)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=1, latch=True)
        rospy.Subscriber(self.start_topic, String, self.start_callback, queue_size=1)
        rospy.Subscriber(self.control_topic, String, self.control_callback, queue_size=5)
        rospy.Subscriber(self.enable_topic, Bool, self.enable_callback, queue_size=5)
        rospy.Subscriber(self.last_goal_topic, String, self.last_goal_callback, queue_size=5)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=20)
        rospy.Subscriber(self.mavros_state_topic, State, self.mavros_state_callback, queue_size=5)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        self.publish_status(force=True)

    def odom_callback(self, msg):
        velocity = msg.twist.twist.linear
        position = msg.pose.pose.position
        odom = {
            "stamp": time.time(),
            "x": position.x,
            "y": position.y,
            "z": position.z,
            "speed": math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2),
        }
        with self.lock:
            self.odom = odom

    def mavros_state_callback(self, msg):
        vehicle_state = {
            "stamp": time.time(),
            "connected": bool(msg.connected),
            "armed": bool(msg.armed),
            "mode": msg.mode,
        }
        with self.lock:
            self.vehicle_state = vehicle_state

    def enable_callback(self, msg):
        with self.lock:
            self.loop.set_enabled(msg.data, time.time())
            self.publish_status(force=True)

    def start_callback(self, msg):
        with self.lock:
            try:
                events = self.loop.start(msg.data, time.time(), self.odom, vehicle_state=self.vehicle_state)
            except TaskLoopError as exc:
                rospy.logwarn("spf_task_executor: rejected start: %s", exc)
                self.loop.record_rejection(str(exc), time.time())
                self.publish_status(force=True, rejection=str(exc))
                return
            self.handle_events(events)
            self.publish_status(force=True)

    def control_callback(self, msg):
        with self.lock:
            try:
                events = self.loop.control(msg.data, time.time())
            except TaskLoopError as exc:
                rospy.logwarn("spf_task_executor: rejected control: %s", exc)
                self.loop.record_rejection(str(exc), time.time())
                self.publish_status(force=True, rejection=str(exc))
                return
            self.handle_events(events)
            self.publish_status(force=True)

    def last_goal_callback(self, msg):
        try:
            payload = json.loads(msg.data or "")
        except (TypeError, ValueError):
            return
        with self.lock:
            if self.loop.receive_goal(payload, time.time()):
                self.publish_status(force=True)

    def timer_callback(self, _event):
        with self.lock:
            events = self.loop.tick(time.time(), self.odom)
            self.handle_events(events)
            self.publish_status()

    def handle_events(self, events):
        for event, value in events:
            if event == "publish_command":
                self.command_pub.publish(String(data=value))
                rospy.loginfo(
                    "spf_task_executor: task=%s cycle=%d requested SPF action",
                    self.loop.task_id,
                    self.loop.cycle_count,
                )

    def publish_status(self, force=False, rejection=None):
        now = time.time()
        payload = self.loop.status()
        if rejection:
            payload["rejection"] = rejection
        encoded = json.dumps(payload, sort_keys=True)
        signature = (
            payload["state"],
            payload["reason"],
            payload["task_id"],
            payload["cycle_count"],
            payload["last_rejection"],
            payload["last_rejection_at"],
        )
        state_changed = signature != self.last_status_signature
        if not force and not state_changed and now - self.last_status_publish_at < 1.0:
            return
        self.status_pub.publish(String(data=encoded))
        if state_changed:
            rospy.loginfo("spf_task_executor: state=%s reason=%s", payload["state"], payload["reason"])
        self.last_status_json = encoded
        self.last_status_signature = signature
        self.last_status_publish_at = now


def main():
    rospy.init_node("spf_task_executor")
    SpfTaskExecutorNode()
    rospy.spin()


if __name__ == "__main__":
    main()

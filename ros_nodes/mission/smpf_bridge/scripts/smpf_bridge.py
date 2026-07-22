#!/usr/bin/env python3

import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
import sys
import threading
import time
import uuid

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
import message_filters
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
import numpy as np
from realsense2_camera.msg import Extrinsics
import rospy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Empty, Float64, String
from visualization_msgs.msg import Marker, MarkerArray


REPO_ROOT = Path(__file__).resolve().parents[4]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from strategy.smpf.runtime import (  # noqa: E402
    CameraIntrinsics,
    CompletedTargetError,
    DEFAULT_LLM_MODEL,
    DEFAULT_LLM_REASONING_EFFORT,
    DEFAULT_VLM_MODEL,
    FOLLOW_SUCCESS,
    FOLLOW_TIMEOUT,
    FOLLOW_UNSAFE,
    GoalValidationResult,
    GuidepointPlan,
    JsonlExperimentLogger,
    ModelPlannerClient,
    ObjectSphere,
    PlanningRequest,
    SamClient,
    SamClientError,
    SemanticSceneMemory,
    SmpfArtifactWriter,
    TaskStageClient,
    TargetIdentityState,
    VisibilityGraphError,
    VisionDetectorClient,
    WaypointExecutionLoop,
    approach_goal_candidates_for_sphere,
    approach_goal_for_sphere,
    assess_corridor_obstacles,
    associate_target_observation,
    body_from_color_via_infra1,
    evaluate_follow_surface_standoff,
    invert_rigid_transform,
    next_follow_observation_is_final,
    rigid_transform,
    rotation_matrix_from_quaternion,
    resolve_llm_reasoning_effort,
    select_follow_goal,
    sphere_from_aligned_bbox,
    target_facing_yaws,
    transform_points,
    validate_extrinsic_transform,
    validate_follow_goal_point,
    validate_goal_conditioned_polyline,
)


SUPPORTED_MODES = {"navigate", "obstacle", "long_horizon", "reasoning", "search", "follow"}
MODE_ALIASES = {
    "navigation": "navigate",
    "avoidance": "obstacle",
    "long-view": "long_horizon",
    "long_view": "long_horizon",
    "inference": "reasoning",
    "tracking": "follow",
}


@dataclass(frozen=True)
class FrameSnapshot:
    stamp: float
    color_stamp: float
    depth_stamp: float
    color: np.ndarray
    depth: np.ndarray
    intrinsics: CameraIntrinsics


def _pose_transform(pose):
    q = pose.orientation
    p = pose.position
    return rigid_transform(
        rotation_matrix_from_quaternion((q.x, q.y, q.z, q.w)),
        (p.x, p.y, p.z),
    )


def _extrinsics_transform(msg):
    return rigid_transform(np.asarray(msg.rotation, dtype=float).reshape(3, 3), msg.translation)


def _yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _angle_error(target, actual):
    return abs(math.atan2(math.sin(target - actual), math.cos(target - actual)))


def _set_yaw(orientation, yaw):
    orientation.x = 0.0
    orientation.y = 0.0
    orientation.z = math.sin(yaw * 0.5)
    orientation.w = math.cos(yaw * 0.5)


def validate_follow_metric_timing(
    grounding_stamp,
    metric_stamp,
    odom_stamp,
    now,
    max_frame_age_sec=1.0,
    max_rgbd_odom_skew_sec=0.08,
    color_stamp=None,
    depth_stamp=None,
):
    """Validate the post-grounding RGB-D/VINS snapshot used by Follow."""
    metric_stamp = float(metric_stamp)
    color_stamp = metric_stamp if color_stamp is None else float(color_stamp)
    depth_stamp = metric_stamp if depth_stamp is None else float(depth_stamp)
    values = tuple(
        float(value)
        for value in (
            grounding_stamp,
            metric_stamp,
            color_stamp,
            depth_stamp,
            odom_stamp,
            now,
        )
    )
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError("Follow observation timestamps must be finite")
    max_age = float(max_frame_age_sec)
    max_skew = float(max_rgbd_odom_skew_sec)
    if not math.isfinite(max_age) or max_age <= 0.0:
        raise ValueError("Follow metric frame max age must be finite and positive")
    if not math.isfinite(max_skew) or max_skew < 0.0:
        raise ValueError("Follow RGB-D/odometry skew limit must be finite and non-negative")
    grounding, metric, color, depth, odom, current = values
    if not math.isclose(metric, max(color, depth), rel_tol=0.0, abs_tol=1e-9):
        raise RuntimeError("Follow metric RGB-D composite timestamp is inconsistent")

    ages = {}
    skews = {}
    for name, sensor_stamp in (("color", color), ("depth", depth)):
        if sensor_stamp <= grounding:
            raise RuntimeError(
                "Follow metric %s frame is not newer than the grounding frame" % name
            )
        age = current - sensor_stamp
        if age < 0.0:
            raise RuntimeError("Follow metric %s frame timestamp is in the future" % name)
        if age > max_age + 1e-9:
            raise RuntimeError(
                "Follow metric %s frame is stale (%.3f s > %.3f s)"
                % (name, age, max_age)
            )
        skew = abs(sensor_stamp - odom)
        if skew > max_skew + 1e-9:
            raise RuntimeError(
                "Follow %s/VINS timestamp skew is too large (%.3f s > %.3f s)"
                % (name, skew, max_skew)
            )
        ages[name] = age
        skews[name] = skew
    return {
        "frame_age_sec": max(ages.values()),
        "rgbd_odom_skew_sec": max(skews.values()),
        "color_frame_age_sec": ages["color"],
        "depth_frame_age_sec": ages["depth"],
        "color_odom_skew_sec": skews["color"],
        "depth_odom_skew_sec": skews["depth"],
    }


def select_follow_sam_mask(prediction):
    """Select the largest valid Follow mask while rejecting no detection."""
    mask = prediction.best_mask
    if mask is None:
        raise RuntimeError("Follow target SAM did not return a mask")
    return mask


class SmpfBridgeNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.RLock()
        self.latest_frame = None
        self.latest_odom = None
        self.vehicle_state = None
        self.body_from_infra1 = None
        self.infra1_from_depth = None
        self.color_from_depth = None
        self.calibration_errors = {}
        self.inference_busy = False
        self.task = None
        self.pending_replan_at = None
        self.search_target = None
        self.last_error = None
        self.model_config_source = "not_initialized"
        self.vlm_model_id = str(
            rospy.get_param(
                "~vlm_model",
                os.environ.get("SMPF_VLM_MODEL", DEFAULT_VLM_MODEL),
            )
        ).strip()
        self.llm_model_id = str(
            rospy.get_param(
                "~llm_model",
                os.environ.get("SMPF_LLM_MODEL", DEFAULT_LLM_MODEL),
            )
        ).strip()
        self.llm_reasoning_effort = resolve_llm_reasoning_effort(
            rospy.get_param(
                "~llm_reasoning_effort",
                os.environ.get(
                    "SMPF_LLM_REASONING_EFFORT",
                    DEFAULT_LLM_REASONING_EFFORT,
                ),
            )
        )
        if not self.vlm_model_id or not self.llm_model_id:
            raise ValueError("SMPF model identifiers cannot be empty")
        self.detector = None
        self.planner = None
        self.stage_client = None
        self.sam = SamClient()
        self.experiment_logger = JsonlExperimentLogger(
            rospy.get_param(
                "~experiment_log_path",
                str(REPO_ROOT / "runtime" / "smpf_trials.jsonl"),
            )
        )
        self.artifact_writer = SmpfArtifactWriter(
            rospy.get_param(
                "~artifact_root",
                str(REPO_ROOT / "runtime" / "smpf_artifacts"),
            )
        )
        self.memory = SemanticSceneMemory(
            association_distance_m=rospy.get_param("~memory_association_distance_m", 0.35),
            ttl_sec=rospy.get_param("~memory_ttl_sec", 120.0),
        )
        self.dynamic_association_distance_m = float(
            rospy.get_param("~dynamic_memory_association_distance_m", 1.5)
        )
        self.deterministic_fallback_enabled = bool(
            rospy.get_param("~deterministic_fallback_enabled", True)
        )
        self.completed_target_exclusion_enabled = bool(
            rospy.get_param("~completed_target_exclusion_enabled", True)
        )
        self.goal_condition_validation_enabled = bool(
            rospy.get_param("~goal_condition_validation_enabled", True)
        )
        self.corridor_obstacle_filter_enabled = bool(
            rospy.get_param("~corridor_obstacle_filter_enabled", True)
        )
        self.follow_step_limit_enabled = bool(
            rospy.get_param("~follow_step_limit_enabled", False)
        )
        self.follow_max_step_m = float(rospy.get_param("~follow_max_step_m", 0.50))
        if not math.isfinite(self.follow_max_step_m) or self.follow_max_step_m <= 0.0:
            raise ValueError("follow_max_step_m must be finite and positive")
        self.follow_target_surface_standoff_m = float(
            rospy.get_param("~follow_target_surface_standoff_m", 0.15)
        )
        self.follow_target_surface_tolerance_m = float(
            rospy.get_param("~follow_target_surface_tolerance_m", 0.10)
        )
        self.follow_metric_frame_max_age_sec = float(
            rospy.get_param("~follow_metric_frame_max_age_sec", 1.0)
        )
        self.follow_metric_odom_skew_sec = float(
            rospy.get_param("~follow_metric_odom_skew_sec", 0.08)
        )
        self.follow_sam_timeout_sec = float(
            rospy.get_param("~follow_sam_timeout_sec", 0.75)
        )
        if (
            not math.isfinite(self.follow_target_surface_standoff_m)
            or self.follow_target_surface_standoff_m <= 0.0
        ):
            raise ValueError("follow_target_surface_standoff_m must be finite and positive")
        if (
            not math.isfinite(self.follow_target_surface_tolerance_m)
            or self.follow_target_surface_tolerance_m <= 0.0
        ):
            raise ValueError("follow_target_surface_tolerance_m must be finite and positive")
        if (
            not math.isfinite(self.follow_metric_frame_max_age_sec)
            or self.follow_metric_frame_max_age_sec <= 0.0
        ):
            raise ValueError("follow_metric_frame_max_age_sec must be finite and positive")
        if (
            not math.isfinite(self.follow_metric_odom_skew_sec)
            or self.follow_metric_odom_skew_sec < 0.0
        ):
            raise ValueError("follow_metric_odom_skew_sec must be finite and non-negative")
        if not math.isfinite(self.follow_sam_timeout_sec) or self.follow_sam_timeout_sec <= 0.0:
            raise ValueError("follow_sam_timeout_sec must be finite and positive")

        self.allow_execution = bool(rospy.get_param("~execution_enabled", True))
        self.require_armed = bool(rospy.get_param("~require_armed_for_execution", False))
        self.min_execution_z = float(rospy.get_param("~min_execution_z", 0.0))
        self.enable_max_speed = float(rospy.get_param("~enable_max_speed", 0.0))
        self.runtime_execution_enabled = self.allow_execution
        self.executor = WaypointExecutionLoop(
            goal_timeout_sec=rospy.get_param("~goal_timeout_sec", 45.0),
            task_timeout_sec=rospy.get_param("~task_timeout_sec", 300.0),
            arrival_settle_sec=rospy.get_param("~arrival_settle_sec", 0.0),
            goal_tolerance_xy=rospy.get_param("~goal_tolerance_xy", 0.25),
            goal_tolerance_z=rospy.get_param("~goal_tolerance_z", 0.20),
            arrival_max_speed=rospy.get_param("~arrival_max_speed", 0.25),
            goal_tolerance_yaw_rad=math.radians(
                float(rospy.get_param("~goal_tolerance_yaw_deg", 10.0))
            ),
            odom_timeout_sec=rospy.get_param("~odom_timeout_sec", 1.0),
        )
        self.executor.set_enabled(self.runtime_execution_enabled, time.time())

        self.frame_timeout_sec = float(rospy.get_param("~frame_timeout_sec", 1.0))
        self.sphere_margin_m = float(rospy.get_param("~sphere_safety_margin_m", 0.30))
        self.path_margin_m = float(rospy.get_param("~path_clearance_margin_m", 0.05))
        self.fallback_standoff_m = float(rospy.get_param("~fallback_standoff_m", 0.15))
        self.min_target_standoff_m = float(rospy.get_param("~min_target_standoff_m", 0.15))
        self.max_target_standoff_m = float(rospy.get_param("~max_target_standoff_m", 1.0))
        self.min_target_progress_m = float(rospy.get_param("~min_target_progress_m", 0.10))
        self.require_target_visibility = bool(rospy.get_param("~require_target_visibility", True))
        self.corridor_obstacle_margin_m = float(
            rospy.get_param("~corridor_obstacle_margin_m", 0.25)
        )
        self.yaw_refresh_hz = float(rospy.get_param("~yaw_refresh_hz", 2.0))
        self.last_yaw_refresh_at = None
        self.max_body_camera_translation_m = float(
            rospy.get_param("~max_body_camera_translation_m", 0.75)
        )
        self.max_realsense_extrinsic_translation_m = float(
            rospy.get_param("~max_realsense_extrinsic_translation_m", 0.10)
        )
        self.local_bounds = {
            "x_min": float(rospy.get_param("~local_min_x", -1.0)),
            "x_max": float(rospy.get_param("~local_max_x", 8.0)),
            "y_min": float(rospy.get_param("~local_min_y", -5.0)),
            "y_max": float(rospy.get_param("~local_max_y", 5.0)),
            "z_min": float(rospy.get_param("~local_min_z", -2.0)),
            "z_max": float(rospy.get_param("~local_max_z", 3.0)),
        }
        self.world_bounds = {
            "x_min": float(rospy.get_param("~world_min_x", -20.0)),
            "x_max": float(rospy.get_param("~world_max_x", 20.0)),
            "y_min": float(rospy.get_param("~world_min_y", -20.0)),
            "y_max": float(rospy.get_param("~world_max_y", 20.0)),
            "z_min": float(rospy.get_param("~world_min_z", 0.05)),
            "z_max": float(rospy.get_param("~world_max_z", 3.0)),
        }

        self.goal_pub = rospy.Publisher(
            rospy.get_param("~goal_topic", "/control/ego_position"), PoseStamped, queue_size=10
        )
        self.stop_pub = rospy.Publisher("/control/stop", Empty, queue_size=1)
        self.planning_yaw_pub = rospy.Publisher(
            rospy.get_param("~planning_yaw_topic", "/planning/goal_yaw_deg"),
            Float64,
            queue_size=5,
        )
        self.status_pub = rospy.Publisher(
            rospy.get_param("~status_topic", "/smpf/status"), String, queue_size=10, latch=True
        )
        self.dry_run_pub = rospy.Publisher(
            rospy.get_param("~dry_run_plan_topic", "/smpf/dry_run_plan"),
            String,
            queue_size=10,
            latch=True,
        )
        self.debug_image_topic = rospy.get_param(
            "~debug_image_topic", "/smpf/debug/annotated_image"
        )
        self.debug_image_pub = rospy.Publisher(
            self.debug_image_topic,
            Image,
            queue_size=2,
            latch=True,
        )
        self.debug_depth_topic = rospy.get_param(
            "~debug_depth_topic", "/smpf/debug/depth_image"
        )
        self.debug_depth_pub = rospy.Publisher(
            self.debug_depth_topic,
            Image,
            queue_size=2,
            latch=True,
        )
        self.debug_spheres_topic = rospy.get_param(
            "~debug_spheres_topic", "/smpf/debug/object_spheres"
        )
        self.debug_spheres_pub = rospy.Publisher(
            self.debug_spheres_topic,
            MarkerArray,
            queue_size=2,
            latch=True,
        )

        rospy.Subscriber(
            rospy.get_param("~odom_topic", "/vins_fusion/imu_propagate"),
            Odometry,
            self._odom_cb,
            queue_size=20,
        )
        rospy.Subscriber("/mavros/state", State, self._mavros_state_cb, queue_size=10)
        rospy.Subscriber(
            rospy.get_param("~vins_extrinsic_topic", "/vins_fusion/extrinsic"),
            Odometry,
            self._vins_extrinsic_cb,
            queue_size=5,
        )
        rospy.Subscriber(
            rospy.get_param("~depth_to_color_topic", "/camera/extrinsics/depth_to_color"),
            Extrinsics,
            self._depth_to_color_cb,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~depth_to_infra1_topic", "/camera/extrinsics/depth_to_infra1"),
            Extrinsics,
            self._depth_to_infra1_cb,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~command_topic", "/smpf/task_command"),
            String,
            self._command_cb,
            queue_size=5,
        )
        rospy.Subscriber(
            rospy.get_param("~control_topic", "/smpf/task_control"),
            String,
            self._control_cb,
            queue_size=5,
        )
        rospy.Subscriber(
            rospy.get_param("~runtime_enable_topic", "/smpf/execution_enable"),
            Bool,
            self._enable_cb,
            queue_size=2,
        )

        color_sub = message_filters.Subscriber(
            rospy.get_param("~color_topic", "/camera/color/image_raw"), Image
        )
        depth_sub = message_filters.Subscriber(
            rospy.get_param("~aligned_depth_topic", "/camera/aligned_depth_to_color/image_raw"), Image
        )
        info_sub = message_filters.Subscriber(
            rospy.get_param("~camera_info_topic", "/camera/color/camera_info"), CameraInfo
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub, info_sub],
            queue_size=int(rospy.get_param("~sync_queue_size", 10)),
            slop=float(rospy.get_param("~sync_slop_sec", 0.08)),
        )
        self.sync.registerCallback(self._frame_cb)
        self.timer = rospy.Timer(rospy.Duration(0.1), self._timer_cb)
        self.status_timer = rospy.Timer(rospy.Duration(1.0), self._status_timer_cb)
        self._publish_status("READY", "SMPF bridge started; execution gate is closed")

    def _frame_cb(self, color_msg, depth_msg, info_msg):
        try:
            color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            depth = np.asarray(depth)
            if depth_msg.encoding.lower() in {"16uc1", "16sc1", "mono16"}:
                depth = depth.astype(np.float32) / 1000.0
            else:
                depth = depth.astype(np.float32)
            if color.shape[:2] != depth.shape[:2]:
                raise ValueError("aligned depth dimensions do not match color")
            intrinsics = CameraIntrinsics(
                fx=info_msg.K[0],
                fy=info_msg.K[4],
                cx=info_msg.K[2],
                cy=info_msg.K[5],
                width=info_msg.width,
                height=info_msg.height,
            )
            color_stamp = color_msg.header.stamp.to_sec()
            depth_stamp = depth_msg.header.stamp.to_sec()
            stamp = max(color_stamp, depth_stamp)
            with self.lock:
                self.latest_frame = FrameSnapshot(
                    stamp=stamp,
                    color_stamp=color_stamp,
                    depth_stamp=depth_stamp,
                    color=color,
                    depth=depth,
                    intrinsics=intrinsics,
                )
        except Exception as exc:
            with self.lock:
                self.last_error = "RGB-D decode failed: %s" % exc

    def _odom_cb(self, msg):
        speed = math.sqrt(
            msg.twist.twist.linear.x ** 2
            + msg.twist.twist.linear.y ** 2
            + msg.twist.twist.linear.z ** 2
        )
        with self.lock:
            self.latest_odom = {
                "msg": msg,
                "stamp": msg.header.stamp.to_sec(),
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "z": msg.pose.pose.position.z,
                "speed": speed,
                "yaw": _yaw_from_quaternion(msg.pose.pose.orientation),
            }

    def _mavros_state_cb(self, msg):
        with self.lock:
            self.vehicle_state = msg

    def _vins_extrinsic_cb(self, msg):
        self._accept_extrinsic(
            "body_from_infra1",
            _pose_transform(msg.pose.pose),
            self.max_body_camera_translation_m,
        )

    def _depth_to_color_cb(self, msg):
        self._accept_extrinsic(
            "color_from_depth",
            _extrinsics_transform(msg),
            self.max_realsense_extrinsic_translation_m,
        )

    def _depth_to_infra1_cb(self, msg):
        self._accept_extrinsic(
            "infra1_from_depth",
            _extrinsics_transform(msg),
            self.max_realsense_extrinsic_translation_m,
        )

    def _accept_extrinsic(self, attribute, transform, maximum_translation_m):
        try:
            accepted = validate_extrinsic_transform(
                transform,
                maximum_translation_m,
                name=attribute,
            )
        except ValueError as exc:
            with self.lock:
                setattr(self, attribute, None)
                self.calibration_errors[attribute] = str(exc)
            rospy.logerr_throttle(5.0, "SMPF rejected calibration: %s", exc)
            return
        with self.lock:
            setattr(self, attribute, accepted)
            self.calibration_errors.pop(attribute, None)

    def _enable_cb(self, msg):
        with self.lock:
            self.runtime_execution_enabled = self.allow_execution
            self.executor.set_enabled(self.allow_execution, rospy.Time.now().to_sec())
        self._publish_status(
            "DIRECT_EXECUTION",
            "runtime authorization commands are ignored; task execution is direct",
        )

    def _command_cb(self, msg):
        try:
            command = self._parse_command(msg.data)
        except Exception as exc:
            self._publish_status("COMMAND_REJECTED", str(exc))
            return
        with self.lock:
            if (
                self.task is not None
                or self.inference_busy
                or self.executor.state == "WAITING_ARRIVAL"
            ):
                self._publish_status("COMMAND_REJECTED", "another task cycle is active")
                return
            requested = command.get("execution_requested")
            if requested and not self._execution_open():
                _ready, reason = self._execution_preconditions(check_speed=False)
                self._publish_status(
                    "COMMAND_REJECTED",
                    "direct execution unavailable: %s" % reason,
                )
                return
            command["execution_requested_at_submit"] = bool(requested)
            self.last_error = None
            self.task = command
            self.pending_replan_at = None
            self.search_target = None
        self._log_event(
            "task_received",
            command["task_id"],
            mode=command["mode"],
            instruction=command["instruction"],
            max_cycles=command["max_cycles"],
            models={"vlm": self.vlm_model_id, "llm": self.llm_model_id},
            llm_reasoning_effort=self.llm_reasoning_effort,
            execution_gate_open=command["execution_requested_at_submit"],
        )
        self._start_cycle()

    def _control_cb(self, msg):
        command = str(msg.data or "").strip().lower()
        now = rospy.Time.now().to_sec()
        if command in {"abort", "stop"}:
            with self.lock:
                task_id = self.task["task_id"] if self.task else None
                request_in_flight = self.inference_busy
                self.executor.abort("operator aborted SMPF task", now)
                self.runtime_execution_enabled = self.allow_execution
                self.executor.set_enabled(self.allow_execution, now)
                self.task = None
                self.pending_replan_at = None
                self.search_target = None
            self.stop_pub.publish(Empty())
            self._log_event("terminal", task_id, state="ABORTED", reason="operator aborted SMPF task")
            reason = "operator aborted SMPF task"
            if request_in_flight:
                reason += "; in-flight model response will be discarded"
            self._publish_status("ABORTED", reason)
        elif command in {"complete", "success"}:
            with self.lock:
                self.task = None
                self.pending_replan_at = None
                self.search_target = None
            self._publish_status("SUCCESS", "operator completed SMPF task")
        elif command == "clear_memory":
            with self.lock:
                self.memory = SemanticSceneMemory(
                    association_distance_m=self.memory.association_distance_m,
                    ttl_sec=self.memory.ttl_sec,
                )
            self._publish_status("MEMORY_CLEARED", "semantic scene memory cleared")
        elif command == "replan":
            self._start_cycle()
        else:
            self._publish_status("CONTROL_REJECTED", "unsupported control command")

    def _parse_command(self, raw):
        raw = str(raw or "").strip()
        if not raw:
            raise ValueError("task instruction cannot be empty")
        if raw.startswith("{"):
            payload = json.loads(raw)
            if not isinstance(payload, dict):
                raise ValueError("task command JSON must be an object")
            instruction = str(payload.get("instruction") or "").strip()
            mode = str(payload.get("mode") or "navigate").strip().lower()
            max_cycles = int(payload.get("max_cycles", 10))
            execution_requested = payload.get("execute", True)
            if not isinstance(execution_requested, bool):
                raise ValueError("execute must be a boolean when provided")
        else:
            instruction = raw
            mode = "navigate"
            max_cycles = 10
            execution_requested = True
        mode = MODE_ALIASES.get(mode, mode)
        if not instruction:
            raise ValueError("task instruction cannot be empty")
        if mode not in SUPPORTED_MODES:
            raise ValueError("unsupported SMPF mode: %s" % mode)
        if max_cycles < 1 or max_cycles > 50:
            raise ValueError("max_cycles must be in [1, 50]")
        return {
            "task_id": uuid.uuid4().hex[:12],
            "instruction": instruction,
            "mode": mode,
            "cycles": 0,
            "max_cycles": max_cycles,
            "search_index": 0,
            "search_base_yaw": None,
            "stages": None,
            "stage_index": 0,
            "target_identity": TargetIdentityState(),
            "follow_final_observation": False,
            "follow_metric_observation": None,
            "execution_requested": execution_requested,
        }

    def _start_cycle(self):
        with self.lock:
            if self.task is None:
                self._publish_status("REPLAN_REJECTED", "no active task")
                return
            if self.inference_busy or self.executor.state == "WAITING_ARRIVAL":
                return
            self.inference_busy = True
            task_id = self.task["task_id"]
        self._publish_status("INFERENCING", "starting synchronized perception and planning")
        thread = threading.Thread(target=self._run_cycle, args=(task_id,), daemon=True)
        thread.start()

    def _run_cycle(self, task_id):
        try:
            self._run_cycle_impl(task_id)
        except Exception as exc:
            failed_follow = False
            failed_metric_observation = None
            cancelled = False
            with self.lock:
                if self.task is None or self.task["task_id"] != task_id:
                    cancelled = True
                else:
                    self.last_error = str(exc)
                    failed_follow = self.task["mode"] == "follow"
                    if failed_follow:
                        failed_metric_observation = self.task.get("follow_metric_observation")
                        self.executor.abort(str(exc), rospy.Time.now().to_sec())
                        self.pending_replan_at = None
                        self.search_target = None
                    self.task = None
            if cancelled:
                return
            if failed_follow:
                self.stop_pub.publish(Empty())
            self._log_event(
                "cycle_error",
                task_id,
                error_type=type(exc).__name__,
                reason=str(exc),
                grounding_frame_stamp=(
                    None
                    if failed_metric_observation is None
                    else failed_metric_observation["grounding_frame_stamp"]
                ),
                metric_frame_stamp=(
                    None
                    if failed_metric_observation is None
                    else failed_metric_observation["metric_frame_stamp"]
                ),
                metric_odom_stamp=(
                    None
                    if failed_metric_observation is None
                    else failed_metric_observation["metric_odom_stamp"]
                ),
                metric_rgbd_odom_skew_sec=(
                    None
                    if failed_metric_observation is None
                    else failed_metric_observation["metric_rgbd_odom_skew_sec"]
                ),
                relocalized_after_grounding=(failed_metric_observation is not None),
            )
            self._publish_status("ERROR", str(exc))
        finally:
            with self.lock:
                self.inference_busy = False

    def _snapshot_inputs(self):
        """Read one internally consistent view of perception, pose, and calibration."""
        with self.lock:
            return (
                self.latest_frame,
                dict(self.latest_odom) if self.latest_odom else None,
                (
                    None if self.body_from_infra1 is None else self.body_from_infra1.copy(),
                    None if self.infra1_from_depth is None else self.infra1_from_depth.copy(),
                    None if self.color_from_depth is None else self.color_from_depth.copy(),
                ),
            )

    def _require_cycle_snapshot(self, frame, odom, transforms):
        if frame is None:
            raise RuntimeError("no synchronized color/aligned-depth/CameraInfo frame")
        if odom is None:
            raise RuntimeError("no VINS odometry")
        if any(transform is None for transform in transforms):
            with self.lock:
                calibration_errors = tuple(self.calibration_errors.values())
            detail = "; ".join(calibration_errors) or "calibration topic unavailable"
            raise RuntimeError("VINS or RealSense extrinsics are unavailable: %s" % detail)
        stamp = frame.stamp
        color = frame.color
        depth = frame.depth
        intrinsics = frame.intrinsics
        if rospy.Time.now().to_sec() - stamp > self.frame_timeout_sec:
            raise RuntimeError("synchronized RGB-D frame is stale")
        if color.shape[:2] != (intrinsics.height, intrinsics.width):
            raise RuntimeError("CameraInfo dimensions do not match RGB-D frame")
        return stamp, color, depth, intrinsics

    def _run_cycle_impl(self, task_id):
        cycle_started = time.monotonic()
        with self.lock:
            if self.task is None or self.task["task_id"] != task_id:
                return
            task = dict(self.task)
        frame, odom, transforms = self._snapshot_inputs()
        stamp, color, depth, intrinsics = self._require_cycle_snapshot(
            frame,
            odom,
            transforms,
        )
        now = rospy.Time.now().to_sec()

        stage_decomposition_latency = 0.0
        stage_model_calls = 0
        stage_instruction = task["instruction"]
        if task["mode"] == "long_horizon":
            if task["stages"] is None:
                pre_decomposition_frame_stamp = stamp
                model_config = self._model_config()
                if self.stage_client is None:
                    self.stage_client = TaskStageClient(
                        api_key=model_config["llm_api_key"],
                        base_url=model_config["llm_base_url"],
                        model_id=self.llm_model_id,
                        reasoning_effort=self.llm_reasoning_effort,
                    )
                stage_started = time.monotonic()
                try:
                    decomposition = self.stage_client.decompose(task["instruction"])
                finally:
                    self._archive_model_responses(task_id, "stage_llm", self.stage_client)
                stage_decomposition_latency = time.monotonic() - stage_started
                stage_model_calls = 1
                if len(decomposition.stages) < 2:
                    raise RuntimeError("long_horizon mode requires at least two ordered target stages")
                stages = tuple(stage.instruction for stage in decomposition.stages)
                with self.lock:
                    if self.task is None or self.task["task_id"] != task_id:
                        return
                    self.task["stages"] = stages
                    self.task["stage_index"] = 0
                task["stages"] = stages
                task["stage_index"] = 0
                self._log_event("task_decomposed", task_id, stages=stages)
                frame, odom, transforms = self._snapshot_inputs()
                stamp, color, depth, intrinsics = self._require_cycle_snapshot(
                    frame,
                    odom,
                    transforms,
                )
                if stamp <= pre_decomposition_frame_stamp:
                    raise RuntimeError(
                        "long-horizon grounding frame did not advance after stage decomposition"
                    )
                self._log_event(
                    "stage_grounding_resnapshot",
                    task_id,
                    pre_decomposition_frame_stamp=pre_decomposition_frame_stamp,
                    grounding_frame_stamp=stamp,
                    grounding_frame_age_sec=max(0.0, rospy.Time.now().to_sec() - stamp),
                )
            stage_instruction = task["stages"][task["stage_index"]]

        if self.detector is None:
            model_config = self._model_config()
            self.detector = VisionDetectorClient(
                api_key=model_config["vlm_api_key"],
                base_url=model_config["vlm_base_url"],
                model_id=self.vlm_model_id,
            )
        grounding_stamp = stamp
        grounding_age_at_detection_start_sec = max(
            0.0,
            rospy.Time.now().to_sec() - grounding_stamp,
        )
        detection_started = time.monotonic()
        try:
            if task["mode"] == "follow":
                detection = self.detector.detect(color, stage_instruction)
                scene_obstacles = ()
            else:
                scene = self.detector.detect_scene(color, stage_instruction)
                detection = scene.target
                scene_obstacles = scene.obstacles
        finally:
            self._archive_model_responses(task_id, "vlm", self.detector)
        detection_latency = time.monotonic() - detection_started
        detection_attempts = self.detector.last_attempts
        vlm_annotations = []
        if detection is not None:
            vlm_annotations.append(
                {
                    "label": "VLM target: %s" % detection.label,
                    "bbox_yxyx": detection.pixel_bbox(color.shape),
                    "color_bgr": (40, 220, 40),
                }
            )
        for item in scene_obstacles:
            vlm_annotations.append(
                {
                    "label": "VLM obstacle: %s" % item.label,
                    "bbox_yxyx": item.pixel_bbox(color.shape),
                    "color_bgr": (0, 180, 255),
                }
            )
        self._publish_debug_image(
            task_id,
            "vlm_grounding",
            color,
            vlm_annotations,
            grounding_stamp,
        )
        self._publish_debug_depth(
            task_id,
            "vlm_grounding",
            depth,
            vlm_annotations,
            grounding_stamp,
        )
        if detection is None:
            if task["mode"] == "follow" and task["follow_final_observation"]:
                with self.lock:
                    if self.task is not None and self.task["task_id"] == task_id:
                        self.task = None
                self.stop_pub.publish(Empty())
                self._log_event(
                    "terminal",
                    task_id,
                    state="TIMEOUT",
                    success=False,
                    reason="final follow observation did not detect the target",
                )
                self._publish_status("TIMEOUT", "final follow observation did not detect the target")
                return
            if task["mode"] == "follow":
                raise RuntimeError("Follow grounding VLM did not identify a visible target label")
            self._log_event(
                "target_not_visible",
                task_id,
                mode=task["mode"],
                detection_latency_sec=detection_latency,
            )
            self._target_missing(task, odom)
            return
        if task["mode"] == "search":
            with self.lock:
                self.task = None
                self.search_target = None
            self._log_event(
                "terminal",
                task_id,
                state="SUCCESS",
                success=True,
                reason="search target detected",
                target_label=detection.label,
                detection_latency_sec=detection_latency,
            )
            self._publish_status("SUCCESS", "search target detected: %s" % detection.label)
            return

        body_models = []
        sam_latency = 0.0
        sam_bbox_fallbacks = []
        sam_annotations = []
        metric_observation = None
        if task["mode"] == "follow":
            frame, odom, transforms = self._snapshot_inputs()
            stamp, color, depth, intrinsics = self._require_cycle_snapshot(
                frame,
                odom,
                transforms,
            )
            metric_timing = validate_follow_metric_timing(
                grounding_stamp,
                stamp,
                odom["stamp"],
                rospy.Time.now().to_sec(),
                self.follow_metric_frame_max_age_sec,
                self.follow_metric_odom_skew_sec,
                color_stamp=frame.color_stamp,
                depth_stamp=frame.depth_stamp,
            )
            metric_observation = {
                "grounding_frame_stamp": grounding_stamp,
                "grounding_frame_age_at_detection_start_sec": (
                    grounding_age_at_detection_start_sec
                ),
                "grounding_frame_age_after_vlm_sec": max(
                    0.0,
                    rospy.Time.now().to_sec() - grounding_stamp,
                ),
                "metric_frame_stamp": stamp,
                "metric_color_stamp": frame.color_stamp,
                "metric_depth_stamp": frame.depth_stamp,
                "metric_odom_stamp": odom["stamp"],
                "metric_frame_age_at_snapshot_sec": metric_timing["frame_age_sec"],
                "metric_rgbd_odom_skew_sec": metric_timing["rgbd_odom_skew_sec"],
                "metric_color_odom_skew_sec": metric_timing["color_odom_skew_sec"],
                "metric_depth_odom_skew_sec": metric_timing["depth_odom_skew_sec"],
                "relocalized": True,
            }
            with self.lock:
                if self.task is None or self.task["task_id"] != task_id:
                    return
                self.task["follow_metric_observation"] = dict(metric_observation)
            remaining_freshness_sec = (
                self.follow_metric_frame_max_age_sec - metric_timing["frame_age_sec"]
            )
            sam_timeout_sec = min(self.follow_sam_timeout_sec, remaining_freshness_sec)
            if sam_timeout_sec <= 0.0:
                raise RuntimeError("Follow metric RGB-D freshness budget expired before SAM")
            body_from_color = body_from_color_via_infra1(
                transforms[0],
                transforms[1],
                transforms[2],
            )
            sam_started = time.monotonic()
            try:
                prediction = self.sam.predict(
                    color,
                    detection.label,
                    timeout_sec=sam_timeout_sec,
                )
            except SamClientError as exc:
                raise RuntimeError(
                    "SAM failed while relocalizing Follow target %s: %s"
                    % (detection.label, exc)
                ) from exc
            finally:
                sam_latency += time.monotonic() - sam_started
            mask = select_follow_sam_mask(prediction)
            metric_observation["sam_mask_count"] = prediction.mask_count
            metric_observation["sam_mask_selection"] = "largest_area"
            metric_observation["sam_selected_mask_area_px"] = mask.area_px
            sam_points = [mask.centroid_uv]
            sam_points.extend(mask.sample_points_uv)
            sam_annotations.append(
                {
                    "label": "SAM target: %s" % detection.label,
                    "bbox_yxyx": mask.bbox_yxyx,
                    "centroid_uv": mask.centroid_uv,
                    "color_bgr": (255, 80, 220),
                }
            )
            self._publish_debug_image(
                task_id,
                "sam_metric",
                color,
                sam_annotations,
                stamp,
            )
            self._publish_debug_depth(
                task_id,
                "sam_metric",
                depth,
                sam_annotations,
                stamp,
            )
            sphere, estimate = sphere_from_aligned_bbox(
                detection.label,
                mask.bbox_yxyx,
                depth,
                intrinsics,
                body_from_color,
                sample_points_uv=sam_points,
                safety_margin_m=self.sphere_margin_m,
                confidence=1.0,
                frame_id="body_flu",
                source="vlm_label_full_frame_sam_relocalized_aligned_rgbd",
            )
            body_models.append((sphere, estimate))
            metric_timing = validate_follow_metric_timing(
                grounding_stamp,
                stamp,
                odom["stamp"],
                rospy.Time.now().to_sec(),
                self.follow_metric_frame_max_age_sec,
                self.follow_metric_odom_skew_sec,
                color_stamp=frame.color_stamp,
                depth_stamp=frame.depth_stamp,
            )
            metric_observation["metric_frame_age_after_sam_sec"] = metric_timing[
                "frame_age_sec"
            ]
            metric_observation["sam_timeout_sec"] = sam_timeout_sec
        else:
            body_from_color = body_from_color_via_infra1(
                transforms[0],
                transforms[1],
                transforms[2],
            )
            detections = (detection,) + scene_obstacles
            for index, item in enumerate(detections):
                detected_bbox = item.pixel_bbox(color.shape)
                roi, offset = self._expanded_roi(color, detected_bbox, expansion=0.20)
                sam_started = time.monotonic()
                try:
                    prediction = self.sam.predict(roi, item.label)
                except SamClientError as exc:
                    if index == 0:
                        raise RuntimeError(
                            "SAM failed for target %s: %s" % (item.label, exc)
                        ) from exc
                    mask = None
                    fallback_reason = type(exc).__name__
                else:
                    mask = prediction.best_mask
                    fallback_reason = "empty_mask"
                finally:
                    sam_latency += time.monotonic() - sam_started

                if mask is None and index == 0:
                    raise RuntimeError("SAM did not segment target: %s" % item.label)
                if mask is None:
                    sam_bbox = detected_bbox
                    sam_points = None
                    sam_centroid = None
                    geometry_source = "vlm_bbox_aligned_rgbd_fallback"
                    sam_bbox_fallbacks.append({"label": item.label, "reason": fallback_reason})
                else:
                    sam_bbox = self._offset_bbox(mask.bbox_yxyx, offset)
                    sam_centroid = self._offset_point(mask.centroid_uv, offset)
                    sam_points = [sam_centroid]
                    sam_points.extend(
                        self._offset_point(point, offset) for point in mask.sample_points_uv
                    )
                    geometry_source = "vlm_sam_aligned_rgbd"
                sam_annotations.append(
                    {
                        "label": "%s %s: %s"
                        % ("SAM" if mask is not None else "VLM fallback", "target" if index == 0 else "obstacle", item.label),
                        "bbox_yxyx": sam_bbox,
                        "centroid_uv": sam_centroid,
                        "color_bgr": (255, 80, 220) if index == 0 else (255, 180, 40),
                    }
                )
                sphere, estimate = sphere_from_aligned_bbox(
                    item.label,
                    sam_bbox,
                    depth,
                    intrinsics,
                    body_from_color,
                    sample_points_uv=sam_points,
                    safety_margin_m=self.sphere_margin_m,
                    confidence=item.confidence,
                    frame_id="body_flu",
                    source=geometry_source,
                )
                body_models.append((sphere, estimate))
            self._publish_debug_image(
                task_id,
                "vlm_sam_geometry",
                color,
                vlm_annotations + sam_annotations,
                stamp,
            )
            self._publish_debug_depth(
                task_id,
                "vlm_sam_geometry",
                depth,
                vlm_annotations + sam_annotations,
                stamp,
            )

        sphere_body, depth_estimate = body_models[0]

        world_from_body = _pose_transform(odom["msg"].pose.pose)
        body_from_world = invert_rigid_transform(world_from_body)
        world_models = []
        for sphere, estimate in body_models:
            world_center = transform_points(world_from_body, sphere.center)
            world_models.append(
                (
                    ObjectSphere(
                        sphere.label,
                        tuple(float(value) for value in world_center),
                        sphere.radius,
                        confidence=sphere.confidence,
                        frame_id="world",
                        source=sphere.source,
                    ),
                    estimate,
                )
            )
        observed_target_world, depth_estimate = world_models[0]
        repeated_target_id = None
        with self.lock:
            if self.task is None or self.task["task_id"] != task_id:
                return
            identity = self.task["target_identity"]
            dynamic_target = task["mode"] == "follow"
            target_gate = self.dynamic_association_distance_m if dynamic_target else None
            reject_completed = (
                self.completed_target_exclusion_enabled
                and task["mode"] == "long_horizon"
                and task["stage_index"] > 0
            )
            observed_entries = []
            try:
                target_entry, identity = associate_target_observation(
                    self.memory,
                    observed_target_world,
                    stamp,
                    identity,
                    reject_completed=reject_completed,
                    dynamic=dynamic_target,
                    max_distance_m=target_gate,
                )
            except CompletedTargetError as exc:
                repeated_target_id = exc.object_id
            if repeated_target_id is None:
                self.task["target_identity"] = identity
                observed_entries.append(target_entry)
                for sphere, _estimate in world_models[1:]:
                    observed_entries.append(self.memory.update(sphere, stamp))
                memory_entries = tuple(self.memory.snapshot_entries(stamp))
        if repeated_target_id is not None:
            self._log_event(
                "target_identity_rejected",
                task_id,
                stage_index=task["stage_index"],
                object_id=repeated_target_id,
                reason="object was completed by an earlier long-horizon stage",
            )
            self._target_missing(
                task,
                odom,
                reason="completed target %s was observed again" % repeated_target_id,
            )
            return

        target_entry = observed_entries[0]
        target_memory_body = ObjectSphere(
            target_entry.label,
            tuple(float(value) for value in transform_points(body_from_world, target_entry.center)),
            target_entry.radius,
            confidence=target_entry.confidence,
            frame_id="body_flu",
            source=target_entry.source,
        )
        target_memory_world = target_entry.as_sphere()
        self._archive_sphere_models(
            task_id,
            stamp,
            observed_entries,
            body_models,
            world_models,
        )
        try:
            corridor_goal = approach_goal_for_sphere(
                target_memory_body,
                clearance_margin_m=self.path_margin_m,
                standoff_m=self.fallback_standoff_m,
            )
        except VisibilityGraphError:
            corridor_goal = target_memory_body.center

        memory_obstacle_records = []
        for entry in memory_entries:
            if entry.object_id == target_entry.object_id:
                continue
            obstacle_sphere_world = entry.as_sphere()
            obstacle_sphere_body = ObjectSphere(
                obstacle_sphere_world.label,
                tuple(
                    float(value)
                    for value in transform_points(body_from_world, obstacle_sphere_world.center)
                ),
                obstacle_sphere_world.radius,
                confidence=obstacle_sphere_world.confidence,
                frame_id="body_flu",
                source=obstacle_sphere_world.source,
            )
            memory_obstacle_records.append((entry, obstacle_sphere_world, obstacle_sphere_body))
        memory_assessments = assess_corridor_obstacles(
            (0.0, 0.0, 0.0),
            corridor_goal,
            (record[2] for record in memory_obstacle_records),
            corridor_margin_m=self.corridor_obstacle_margin_m,
        )
        relevant_memory_records = [
            record
            for record, assessment in zip(memory_obstacle_records, memory_assessments)
            if assessment.relevant or not self.corridor_obstacle_filter_enabled
        ]
        memory_world = (target_memory_world,) + tuple(
            record[1] for record in relevant_memory_records
        )
        memory_body = (target_memory_body,) + tuple(
            record[2] for record in relevant_memory_records
        )
        observed_obstacle_assessments = assess_corridor_obstacles(
            (0.0, 0.0, 0.0),
            corridor_goal,
            (sphere for sphere, _estimate in body_models[1:]),
            corridor_margin_m=self.corridor_obstacle_margin_m,
        )

        current_target_center_distance = float(
            np.linalg.norm(np.asarray(target_memory_body.center))
        )
        current_target_surface_distance = (
            current_target_center_distance - target_memory_body.radius
        )
        if task["mode"] == "follow":
            decision_timing = self._validate_follow_metric_observation(metric_observation)
            metric_observation["metric_frame_age_at_standoff_decision_sec"] = decision_timing[
                "frame_age_sec"
            ]
            follow_decision = evaluate_follow_surface_standoff(
                current_target_surface_distance,
                desired_standoff_m=self.follow_target_surface_standoff_m,
                tolerance_m=self.follow_target_surface_tolerance_m,
                minimum_safe_surface_distance_m=self.path_margin_m,
                final_observation=task["follow_final_observation"],
            )
            if follow_decision in {FOLLOW_SUCCESS, FOLLOW_TIMEOUT, FOLLOW_UNSAFE}:
                state = {
                    FOLLOW_SUCCESS: "SUCCESS",
                    FOLLOW_TIMEOUT: "TIMEOUT",
                    FOLLOW_UNSAFE: "ERROR",
                }[follow_decision]
                reason = {
                    FOLLOW_SUCCESS: "follow target is visible at the requested sphere-surface standoff",
                    FOLLOW_TIMEOUT: "follow cycle budget exhausted outside the standoff tolerance",
                    FOLLOW_UNSAFE: "current pose is inside the target safety clearance",
                }[follow_decision]
                self._complete_follow_decision(task_id, follow_decision)
                self._log_event(
                    "terminal",
                    task_id,
                    state=state,
                    success=state == "SUCCESS",
                    reason=reason,
                    target_center_distance_m=current_target_center_distance,
                    target_surface_distance_m=current_target_surface_distance,
                    grounding_frame_stamp=metric_observation["grounding_frame_stamp"],
                    metric_frame_stamp=metric_observation["metric_frame_stamp"],
                    metric_frame_age_sec=decision_timing["frame_age_sec"],
                    metric_odom_stamp=metric_observation["metric_odom_stamp"],
                    metric_rgbd_odom_skew_sec=metric_observation[
                        "metric_rgbd_odom_skew_sec"
                    ],
                    relocalized_after_grounding=True,
                )
                self._publish_status(state, reason)
                return
        cycle_local_bounds = dict(self.local_bounds)
        cycle_local_bounds["z_min"] = max(
            cycle_local_bounds["z_min"],
            self.world_bounds["z_min"] - odom["z"] + 0.05,
        )
        fallback_goals = ()
        follow_goal = None
        follow_goal_latency = 0.0
        planner_latency = 0.0
        if task["mode"] == "follow":
            follow_goal_started = time.monotonic()
            follow_goal = select_follow_goal(
                target_memory_body,
                memory_body,
                bounds=cycle_local_bounds,
                clearance_margin_m=self.path_margin_m,
                surface_standoff_m=self.follow_target_surface_standoff_m,
                max_step_m=(
                    self.follow_max_step_m if self.follow_step_limit_enabled else None
                ),
            )
            body_goal_validation = validate_follow_goal_point(
                follow_goal.goal,
                memory_body,
                bounds=cycle_local_bounds,
                clearance_margin_m=self.path_margin_m,
            )
            if not body_goal_validation.valid:
                kinds = sorted({issue.kind for issue in body_goal_validation.issues})
                raise RuntimeError("body-frame Follow goal rejected: %s" % ",".join(kinds))
            plan = GuidepointPlan(
                ((0.0, 0.0, 0.0), follow_goal.goal),
                "VLM/SAM target geometry produced one free 3-D target-sphere standoff goal; EGO owns trajectory generation.",
                body_goal_validation,
                attempts=0,
                planner_source="direct_3d_follow_goal",
                target_surface_distance_m=follow_goal.target_surface_distance_m,
                target_progress_m=(
                    current_target_center_distance - follow_goal.target_center_distance_m
                ),
                target_visible=follow_goal.target_visible,
            )
            follow_goal_latency = time.monotonic() - follow_goal_started
        else:
            if self.planner is None:
                model_config = self._model_config()
                self.planner = ModelPlannerClient(
                    api_key=model_config["llm_api_key"],
                    base_url=model_config["llm_base_url"],
                    model_id=self.llm_model_id,
                    reasoning_effort=self.llm_reasoning_effort,
                )
            try:
                fallback_goals = approach_goal_candidates_for_sphere(
                    target_memory_body,
                    clearance_margin_m=self.path_margin_m,
                    standoff_m=self.fallback_standoff_m,
                    bounds=cycle_local_bounds,
                )
            except VisibilityGraphError:
                fallback_goals = ()
            request = PlanningRequest(
                stage_instruction,
                memory_body,
                bounds_flu_m=cycle_local_bounds,
                clearance_margin_m=self.path_margin_m,
                fallback_goals_flu_m=fallback_goals,
                target_sphere=(
                    target_memory_body if self.goal_condition_validation_enabled else None
                ),
                min_target_standoff_m=self.min_target_standoff_m,
                max_target_standoff_m=self.max_target_standoff_m,
                min_target_progress_m=self.min_target_progress_m,
                require_target_visibility=self.require_target_visibility,
            )
            planner_started = time.monotonic()
            try:
                plan = self.planner.plan(
                    request,
                    max_attempts=2,
                    enable_deterministic_fallback=self.deterministic_fallback_enabled,
                )
            finally:
                self._archive_model_responses(task_id, "planning_llm", self.planner)
            planner_latency = time.monotonic() - planner_started
        world_points_array = transform_points(world_from_body, plan.guidepoints_m)
        world_points = tuple(tuple(float(value) for value in point) for point in world_points_array)
        if task["mode"] == "follow":
            world_validation = validate_follow_goal_point(
                world_points[-1],
                memory_world,
                bounds=self.world_bounds,
                clearance_margin_m=self.path_margin_m,
            )
            target_center_world = np.asarray(target_memory_world.center, dtype=float)
            start_target_distance = float(
                np.linalg.norm(np.asarray(world_points[0], dtype=float) - target_center_world)
            )
            goal_target_distance = float(
                np.linalg.norm(np.asarray(world_points[-1], dtype=float) - target_center_world)
            )
            world_goal_validation = GoalValidationResult(
                world_validation,
                target_surface_distance_m=goal_target_distance - target_memory_world.radius,
                target_progress_m=start_target_distance - goal_target_distance,
                target_visible=follow_goal.target_visible,
            )
        else:
            world_goal_validation = validate_goal_conditioned_polyline(
                world_points,
                memory_world,
                target_sphere=(
                    target_memory_world if self.goal_condition_validation_enabled else None
                ),
                bounds=self.world_bounds,
                clearance_margin_m=self.path_margin_m,
                min_target_standoff_m=self.min_target_standoff_m,
                max_target_standoff_m=self.max_target_standoff_m,
                min_target_progress_m=self.min_target_progress_m,
                require_target_visibility=self.require_target_visibility,
                bounds_start_index=1,
            )
            world_validation = world_goal_validation.validation
        if not world_validation.valid:
            kinds = sorted({issue.kind for issue in world_validation.issues})
            raise RuntimeError("world-frame plan rejected after transform: %s" % ",".join(kinds))
        planned_target_yaws = target_facing_yaws(
            world_points[1:],
            target_entry.center,
            fallback_yaw=odom["yaw"],
        )
        path_length = sum(
            float(np.linalg.norm(world_points_array[index + 1] - world_points_array[index]))
            for index in range(len(world_points_array) - 1)
        )
        observation_age_at_plan_sec = max(0.0, rospy.Time.now().to_sec() - stamp)
        execution_points = world_points[1:]
        execution_yaws = planned_target_yaws
        follow_step = None
        if task["mode"] == "follow":
            desired_standoff_world = tuple(
                float(value)
                for value in transform_points(
                    world_from_body,
                    (follow_goal.desired_standoff_goal,),
                )[0]
            )
            execution_points = (world_points[-1],)
            execution_yaws = planned_target_yaws[-1:]
            follow_step = {
                "limit_enabled": self.follow_step_limit_enabled,
                "max_step_m": self.follow_max_step_m,
                "distance_m": follow_goal.distance_m,
                "distance_to_standoff_m": follow_goal.distance_to_standoff_m,
                "full_path_length_m": follow_goal.distance_to_standoff_m,
                "target_center_distance_m": follow_goal.target_center_distance_m,
                "requested_surface_standoff_m": follow_goal.requested_surface_standoff_m,
                "safety_limited": follow_goal.safety_limited,
                "target_visible": follow_goal.target_visible,
                "goal_world_m": world_points[-1],
                "desired_standoff_goal_world_m": desired_standoff_world,
                "waypoints_world_m": execution_points,
                "source_segment_index": 0,
                "candidate_index": follow_goal.candidate_index,
                "candidate_count": follow_goal.candidate_count,
                "clipped": follow_goal.clipped,
                "trajectory_owner": "ego",
                "llm_planning_skipped": True,
            }
            if len(execution_points) != 1:
                raise RuntimeError("Follow must submit exactly one tracking goal to EGO")
            completed_timing = self._validate_follow_metric_observation(metric_observation)
            metric_observation["metric_frame_age_at_goal_completion_sec"] = completed_timing[
                "frame_age_sec"
            ]
            observation_age_at_plan_sec = completed_timing["frame_age_sec"]

        dry_run = {
            "schema": "gameuav.smpf.dry_run.v1",
            "task_id": task_id,
            "mode": task["mode"],
            "models": {"vlm": self.vlm_model_id, "llm": self.llm_model_id},
            "llm_reasoning_effort": self.llm_reasoning_effort,
            "stage_index": task["stage_index"],
            "stage_count": 0 if task["stages"] is None else len(task["stages"]),
            "stage_instruction": stage_instruction,
            "frame": "world",
            "goal_representation": (
                "single_3d_tracking_point"
                if task["mode"] == "follow"
                else "verified_guidepoint_path"
            ),
            "trajectory_owner": "ego",
            "artifacts": {
                "task_dir": str(self.artifact_writer.task_directory(task_id)),
                "annotated_image_topic": self.debug_image_topic,
                "depth_image_topic": self.debug_depth_topic,
                "object_spheres_topic": self.debug_spheres_topic,
            },
            "guidepoints_m": () if task["mode"] == "follow" else world_points,
            "tracking_goal_world_m": (
                world_points[-1] if task["mode"] == "follow" else None
            ),
            "target_facing_yaw_deg": [math.degrees(yaw) for yaw in planned_target_yaws],
            "observation_age_at_plan_sec": observation_age_at_plan_sec,
            "grounding_observation": {
                "frame_stamp": grounding_stamp,
                "frame_age_at_detection_start_sec": (
                    grounding_age_at_detection_start_sec
                ),
            },
            "metric_observation": metric_observation,
            "relocalized_after_grounding": (
                False if metric_observation is None else metric_observation["relocalized"]
            ),
            "follow_goal": follow_step,
            "follow_step": follow_step,
            "modeled_object_count": len(memory_world),
            "target": {
                "object_id": target_entry.object_id,
                "label": target_memory_world.label,
                "center_world_m": target_memory_world.center,
                "safety_radius_m": target_memory_world.radius,
                "depth_m": depth_estimate.value_m,
                "depth_std_m": depth_estimate.std_m,
            },
            "explicit_obstacles": [
                {
                    "object_id": entry.object_id,
                    "label": sphere.label,
                    "center_world_m": sphere.center,
                    "safety_radius_m": sphere.radius,
                    "geometry_source": sphere.source,
                    "sam_bbox_fallback": sphere.source == "vlm_bbox_aligned_rgbd_fallback",
                    "corridor_surface_clearance_m": assessment.centerline_clearance_m,
                    "corridor_relevant": assessment.relevant,
                    "included_in_planning": (
                        assessment.relevant or not self.corridor_obstacle_filter_enabled
                    ),
                }
                for entry, (sphere, _estimate), assessment in zip(
                    observed_entries[1:],
                    world_models[1:],
                    observed_obstacle_assessments,
                )
            ],
            "obstacle_filter": {
                "enabled": self.corridor_obstacle_filter_enabled,
                "corridor_margin_m": self.corridor_obstacle_margin_m,
                "memory_candidate_count": len(memory_obstacle_records),
                "planning_obstacle_count": len(relevant_memory_records),
                "filtered_memory_obstacle_count": (
                    len(memory_obstacle_records) - len(relevant_memory_records)
                ),
            },
            "planner_attempts": plan.attempts,
            "vlm_attempts": detection_attempts,
            "target_approach_candidate_count": (
                follow_goal.candidate_count
                if task["mode"] == "follow"
                else len(fallback_goals)
            ),
            "sam_bbox_fallbacks": sam_bbox_fallbacks,
            "planner_source": plan.planner_source,
            "fallback_trigger": plan.fallback_trigger,
            "visibility_graph": {
                "candidate_count": plan.graph_candidate_count,
                "expanded_nodes": plan.graph_expanded_nodes,
            },
            "target_terminal": {
                "surface_distance_m": world_goal_validation.target_surface_distance_m,
                "center_distance_m": (
                    follow_goal.target_center_distance_m
                    if task["mode"] == "follow"
                    else None
                ),
                "requested_surface_standoff_m": (
                    self.follow_target_surface_standoff_m
                    if task["mode"] == "follow"
                    else None
                ),
                "surface_standoff_tolerance_m": (
                    self.follow_target_surface_tolerance_m
                    if task["mode"] == "follow"
                    else None
                ),
                "safety_limited": (
                    follow_goal.safety_limited if task["mode"] == "follow" else None
                ),
                "progress_m": world_goal_validation.target_progress_m,
                "visible": world_goal_validation.target_visible,
                "current_observation_visible": True,
                "min_surface_standoff_m": self.min_target_standoff_m,
                "max_surface_standoff_m": self.max_target_standoff_m,
            },
            "ablation": {
                "deterministic_fallback_enabled": self.deterministic_fallback_enabled,
                "completed_target_exclusion_enabled": self.completed_target_exclusion_enabled,
                "goal_condition_validation_enabled": self.goal_condition_validation_enabled,
                "corridor_obstacle_filter_enabled": self.corridor_obstacle_filter_enabled,
                "follow_step_limit_enabled": self.follow_step_limit_enabled,
            },
            "reasoning": plan.reasoning,
            "execution_gate_open": self._execution_open(),
            "execution_requested_at_submit": task["execution_requested_at_submit"],
        }
        self.dry_run_pub.publish(String(data=json.dumps(dry_run, ensure_ascii=False, sort_keys=True)))
        self._log_event(
            "plan_verified",
            task_id,
            mode=task["mode"],
            models={"vlm": self.vlm_model_id, "llm": self.llm_model_id},
            llm_reasoning_effort=self.llm_reasoning_effort,
            stage_index=task["stage_index"],
            stage_count=0 if task["stages"] is None else len(task["stages"]),
            stage_instruction=stage_instruction,
            target_label=target_memory_world.label,
            target_object_id=target_entry.object_id,
            target_depth_m=depth_estimate.value_m,
            target_depth_std_m=depth_estimate.std_m,
            vlm_attempts=detection_attempts,
            safety_radius_m=target_memory_world.radius,
            modeled_object_count=len(memory_world),
            explicit_obstacle_count=len(world_models) - 1,
            sam_bbox_fallback_count=len(sam_bbox_fallbacks),
            sam_bbox_fallback_labels=[item["label"] for item in sam_bbox_fallbacks],
            memory_obstacle_candidate_count=len(memory_obstacle_records),
            planning_obstacle_count=len(relevant_memory_records),
            filtered_memory_obstacle_count=(
                len(memory_obstacle_records) - len(relevant_memory_records)
            ),
            minimum_corridor_surface_clearance_m=(
                min(
                    assessment.centerline_clearance_m
                    for assessment in memory_assessments
                )
                if memory_assessments
                else None
            ),
            guidepoint_count=0 if task["mode"] == "follow" else len(world_points),
            tracking_goal_count=1 if task["mode"] == "follow" else 0,
            tracking_goal_distance_m=(
                follow_goal.distance_m if task["mode"] == "follow" else None
            ),
            path_length_m=None if task["mode"] == "follow" else path_length,
            goal_representation=(
                "single_3d_tracking_point"
                if task["mode"] == "follow"
                else "verified_guidepoint_path"
            ),
            trajectory_owner="ego",
            planner_source=plan.planner_source,
            deterministic_fallback_used=plan.planner_source == "visibility_graph_fallback",
            fallback_trigger=plan.fallback_trigger,
            graph_candidate_count=plan.graph_candidate_count,
            graph_expanded_nodes=plan.graph_expanded_nodes,
            target_approach_candidate_count=(
                follow_goal.candidate_count
                if task["mode"] == "follow"
                else len(fallback_goals)
            ),
            target_facing_yaw_deg=[math.degrees(yaw) for yaw in planned_target_yaws],
            observation_age_at_plan_sec=observation_age_at_plan_sec,
            grounding_frame_stamp=grounding_stamp,
            grounding_frame_age_at_detection_start_sec=(
                grounding_age_at_detection_start_sec
            ),
            metric_frame_stamp=(
                None if metric_observation is None else metric_observation["metric_frame_stamp"]
            ),
            metric_frame_age_sec=(
                None
                if metric_observation is None
                else metric_observation["metric_frame_age_at_goal_completion_sec"]
            ),
            metric_odom_stamp=(
                None if metric_observation is None else metric_observation["metric_odom_stamp"]
            ),
            metric_rgbd_odom_skew_sec=(
                None
                if metric_observation is None
                else metric_observation["metric_rgbd_odom_skew_sec"]
            ),
            sam_mask_count=(
                None if metric_observation is None else metric_observation.get("sam_mask_count")
            ),
            sam_mask_selection=(
                None
                if metric_observation is None
                else metric_observation.get("sam_mask_selection")
            ),
            sam_selected_mask_area_px=(
                None
                if metric_observation is None
                else metric_observation.get("sam_selected_mask_area_px")
            ),
            relocalized_after_grounding=(metric_observation is not None),
            follow_step_limit_enabled=self.follow_step_limit_enabled,
            follow_step_distance_m=(None if follow_step is None else follow_step["distance_m"]),
            follow_step_clipped=(None if follow_step is None else follow_step["clipped"]),
            follow_goal_distance_m=(None if follow_step is None else follow_step["distance_m"]),
            follow_goal_clipped=(None if follow_step is None else follow_step["clipped"]),
            follow_goal_candidate_index=(
                None if follow_step is None else follow_step["candidate_index"]
            ),
            follow_goal_candidate_count=(
                None if follow_step is None else follow_step["candidate_count"]
            ),
            follow_execution_waypoint_count=(
                None if follow_step is None else len(follow_step["waypoints_world_m"])
            ),
            ablation={
                "deterministic_fallback_enabled": self.deterministic_fallback_enabled,
                "completed_target_exclusion_enabled": self.completed_target_exclusion_enabled,
                "goal_condition_validation_enabled": self.goal_condition_validation_enabled,
                "corridor_obstacle_filter_enabled": self.corridor_obstacle_filter_enabled,
                "follow_step_limit_enabled": self.follow_step_limit_enabled,
            },
            minimum_clearance_body_m=plan.validation.minimum_clearance_m,
            minimum_clearance_world_m=world_validation.minimum_clearance_m,
            target_surface_distance_m=world_goal_validation.target_surface_distance_m,
            target_center_distance_m=(
                follow_goal.target_center_distance_m
                if task["mode"] == "follow"
                else None
            ),
            requested_target_surface_standoff_m=(
                self.follow_target_surface_standoff_m
                if task["mode"] == "follow"
                else None
            ),
            target_progress_m=world_goal_validation.target_progress_m,
            target_visible=world_goal_validation.target_visible,
            model_calls={
                "stage_llm": stage_model_calls,
                "vlm": detection_attempts,
                "sam": len(body_models),
                "llm": plan.attempts,
            },
            latency_sec={
                "stage_llm": stage_decomposition_latency,
                "vlm": detection_latency,
                "sam": sam_latency,
                "llm": planner_latency,
                "follow_goal": follow_goal_latency,
                "cycle_total": time.monotonic() - cycle_started,
            },
            execution_gate_open=self._execution_open(),
            execution_requested_at_submit=task["execution_requested_at_submit"],
        )

        with self.lock:
            if self.task is None or self.task["task_id"] != task_id:
                return
            self.task["cycles"] += 1
            if task["mode"] == "follow":
                self.task["follow_metric_observation"] = dict(metric_observation)
            if not task["execution_requested_at_submit"] or not self._execution_open():
                self.task = None
                self._log_event("terminal", task_id, state="DRY_RUN", success=None)
                reason = (
                    "execution was not requested when the task was submitted"
                    if not task["execution_requested_at_submit"]
                    else "execution gate is closed"
                )
                self._publish_status("DRY_RUN", "verified plan published; %s" % reason)
                return
            events = self.executor.start(
                task_id,
                execution_points,
                rospy.Time.now().to_sec(),
                waypoint_yaws=execution_yaws,
            )
        self._handle_execution_events(events)
        self._refresh_target_yaw(rospy.Time.now().to_sec())
        self._publish_status("EXECUTING", "verified world-frame waypoints accepted")

    def _target_missing(self, task, odom, reason="target not visible"):
        if task["mode"] not in {"search", "follow", "long_horizon"}:
            raise RuntimeError("target is not visible; use search mode rather than guessing")
        if not task.get("execution_requested_at_submit", False):
            self._publish_status(
                "SEARCH_REQUIRED",
                "%s; execution was not requested when the task was submitted" % reason,
            )
            return
        if not self._execution_open():
            self._publish_status("SEARCH_REQUIRED", "%s; execution gate is closed" % reason)
            return
        offsets_deg = (45, -45, 90, -90, 135, -135, 180)
        with self.lock:
            if self.task is None or self.task["task_id"] != task["task_id"]:
                return
            index = self.task["search_index"]
            if index >= len(offsets_deg):
                self.task = None
                self._publish_status("TIMEOUT", "search sweep exhausted without detecting target")
                return
            if self.task["search_base_yaw"] is None:
                self.task["search_base_yaw"] = odom["yaw"]
            yaw = self.task["search_base_yaw"] + math.radians(offsets_deg[index])
            self.task["search_index"] += 1
            self.search_target = {
                "yaw": yaw,
                "deadline": rospy.Time.now().to_sec() + float(rospy.get_param("~search_view_timeout_sec", 5.0)),
                "settled_since": None,
            }
        self._publish_pose_goal((odom["x"], odom["y"], odom["z"]), yaw)
        self._publish_status(
            "SEARCHING",
            "%s; commanded gated in-place search view %d" % (reason, index + 1),
        )

    def _model_config(self):
        llm_key = os.environ.get("SMPF_LLM_API_KEY", "").strip()
        llm_url = os.environ.get("SMPF_LLM_BASE_URL", "").strip()
        vlm_key = os.environ.get("SMPF_VLM_API_KEY", "").strip() or llm_key
        vlm_url = os.environ.get("SMPF_VLM_BASE_URL", "").strip() or llm_url
        if llm_key and llm_url and vlm_key and vlm_url:
            self.model_config_source = "environment"
            return {
                "llm_api_key": llm_key,
                "llm_base_url": llm_url,
                "vlm_api_key": vlm_key,
                "vlm_base_url": vlm_url,
            }

        legacy_path = (
            REPO_ROOT
            / "strategy"
            / "smpf"
            / "upstream"
            / "src"
            / "guide_module"
            / "config"
            / "config.json"
        )
        try:
            with legacy_path.open("r", encoding="utf-8") as stream:
                section = json.load(stream).get("modelinfo", {})
        except Exception as exc:
            raise RuntimeError("SMPF model environment is unset and legacy config cannot be read") from exc
        fallback_key = str(section.get("OPENAIKEY") or section.get("api_key") or "").strip()
        fallback_url = str(section.get("BASE_URL") or section.get("base_url") or "").strip()
        llm_key, llm_url = llm_key or fallback_key, llm_url or fallback_url
        vlm_key, vlm_url = vlm_key or fallback_key, vlm_url or fallback_url
        if not llm_key or not llm_url or not vlm_key or not vlm_url:
            raise RuntimeError("SMPF model credentials are not configured")
        self.model_config_source = "legacy_local_config"
        return {
            "llm_api_key": llm_key,
            "llm_base_url": llm_url,
            "vlm_api_key": vlm_key,
            "vlm_base_url": vlm_url,
        }

    @staticmethod
    def _expanded_roi(image, bbox_yxyx, expansion):
        ymin, xmin, ymax, xmax = bbox_yxyx
        height, width = image.shape[:2]
        pad_y = int(round((ymax - ymin + 1) * expansion))
        pad_x = int(round((xmax - xmin + 1) * expansion))
        y0, x0 = max(0, ymin - pad_y), max(0, xmin - pad_x)
        y1, x1 = min(height - 1, ymax + pad_y), min(width - 1, xmax + pad_x)
        if y1 <= y0 or x1 <= x0:
            raise RuntimeError("detected target ROI has no usable area")
        return image[y0 : y1 + 1, x0 : x1 + 1], (x0, y0)

    @staticmethod
    def _offset_point(point_uv, offset_xy):
        return (float(point_uv[0]) + offset_xy[0], float(point_uv[1]) + offset_xy[1])

    @staticmethod
    def _offset_bbox(bbox_yxyx, offset_xy):
        ymin, xmin, ymax, xmax = bbox_yxyx
        return (ymin + offset_xy[1], xmin + offset_xy[0], ymax + offset_xy[1], xmax + offset_xy[0])

    def _timer_cb(self, _event):
        now = rospy.Time.now().to_sec()
        with self.lock:
            odom = dict(self.latest_odom) if self.latest_odom else None
            events = self.executor.tick(now, odom)
            search_target = dict(self.search_target) if self.search_target else None
            pending_replan = self.pending_replan_at
        self._handle_execution_events(events)

        if search_target and odom:
            ready = _angle_error(search_target["yaw"], odom["yaw"]) <= math.radians(10.0)
            ready = ready and odom["speed"] <= 0.25
            with self.lock:
                if self.search_target:
                    if ready and self.search_target["settled_since"] is None:
                        self.search_target["settled_since"] = now
                    settled = (
                        self.search_target["settled_since"] is not None
                        and now - self.search_target["settled_since"] >= 0.5
                    )
                    timed_out = now >= self.search_target["deadline"]
                    if settled or timed_out:
                        self.search_target = None
                        self.pending_replan_at = now
                        pending_replan = now
        if pending_replan is not None and now >= pending_replan:
            with self.lock:
                self.pending_replan_at = None
            self._start_cycle()

    def _validate_follow_metric_observation(self, observation, now=None):
        if not isinstance(observation, dict) or not observation.get("relocalized"):
            raise RuntimeError("Follow metric observation metadata is unavailable")
        required = {
            "grounding_frame_stamp",
            "metric_frame_stamp",
            "metric_color_stamp",
            "metric_depth_stamp",
            "metric_odom_stamp",
        }
        if not required.issubset(observation):
            raise RuntimeError("Follow metric observation metadata is unavailable")
        return validate_follow_metric_timing(
            observation["grounding_frame_stamp"],
            observation["metric_frame_stamp"],
            observation["metric_odom_stamp"],
            rospy.Time.now().to_sec() if now is None else now,
            self.follow_metric_frame_max_age_sec,
            self.follow_metric_odom_skew_sec,
            color_stamp=observation["metric_color_stamp"],
            depth_stamp=observation["metric_depth_stamp"],
        )

    def _stop_failed_follow_task(self, task_id):
        should_stop = False
        with self.lock:
            if (
                self.task is not None
                and self.task["task_id"] == task_id
                and self.task["mode"] == "follow"
            ):
                self.task = None
                self.pending_replan_at = None
                self.search_target = None
                should_stop = True
        if should_stop:
            self.stop_pub.publish(Empty())
        return should_stop

    def _complete_follow_decision(self, task_id, decision):
        if decision in {FOLLOW_TIMEOUT, FOLLOW_UNSAFE}:
            return self._stop_failed_follow_task(task_id)
        if decision != FOLLOW_SUCCESS:
            raise ValueError("unsupported terminal Follow decision: %s" % decision)
        with self.lock:
            if self.task is not None and self.task["task_id"] == task_id:
                self.task = None
                return True
        return False

    def _handle_execution_events(self, events):
        for event in events:
            if event["type"] == "publish_goal":
                with self.lock:
                    task = dict(self.task) if self.task else None
                    latest_yaw = self.latest_odom["yaw"] if self.latest_odom else 0.0
                yaw = event.get("yaw")
                if yaw is None:
                    yaw = latest_yaw
                metric_observation = None
                is_follow = task is not None and task["mode"] == "follow"
                if is_follow:
                    metric_observation = task.get("follow_metric_observation")
                try:
                    if is_follow and metric_observation is None:
                        raise RuntimeError("Follow metric observation metadata is unavailable")
                    publish_timing = self._publish_pose_goal(
                        event["goal"],
                        yaw,
                        metric_observation=metric_observation,
                    )
                except RuntimeError as exc:
                    if is_follow:
                        with self.lock:
                            if self.task is not None and self.task["task_id"] == task["task_id"]:
                                self.executor.abort(str(exc), rospy.Time.now().to_sec())
                                self.task = None
                                self.pending_replan_at = None
                                self.search_target = None
                        self.stop_pub.publish(Empty())
                        self._log_event(
                            "follow_goal_publish_rejected",
                            task["task_id"],
                            reason=str(exc),
                            grounding_frame_stamp=(
                                None
                                if metric_observation is None
                                else metric_observation["grounding_frame_stamp"]
                            ),
                            metric_frame_stamp=(
                                None
                                if metric_observation is None
                                else metric_observation["metric_frame_stamp"]
                            ),
                            metric_odom_stamp=(
                                None
                                if metric_observation is None
                                else metric_observation["metric_odom_stamp"]
                            ),
                            metric_rgbd_odom_skew_sec=(
                                None
                                if metric_observation is None
                                else metric_observation["metric_rgbd_odom_skew_sec"]
                            ),
                            relocalized_after_grounding=(metric_observation is not None),
                        )
                    raise
                self._log_event(
                    "goal_published",
                    task["task_id"] if task else None,
                    waypoint_index=event["index"],
                    goal_world_m=event["goal"],
                    target_facing_yaw_rad=yaw,
                    metric_frame_stamp=(
                        None
                        if metric_observation is None
                        else metric_observation["metric_frame_stamp"]
                    ),
                    metric_frame_age_at_publish_sec=(
                        None if publish_timing is None else publish_timing["frame_age_sec"]
                    ),
                    metric_rgbd_odom_skew_sec=(
                        None
                        if metric_observation is None
                        else metric_observation["metric_rgbd_odom_skew_sec"]
                    ),
                    relocalized_after_grounding=(metric_observation is not None),
                )
            elif event["type"] == "terminal":
                failed_follow_task_id = None
                with self.lock:
                    task = dict(self.task) if self.task else None
                    if event["state"] == "SUCCESS" and task and task["mode"] == "follow":
                        final_observation = next_follow_observation_is_final(
                            task["cycles"],
                            task["max_cycles"],
                        )
                        self.task["follow_final_observation"] = final_observation
                        self.pending_replan_at = rospy.Time.now().to_sec() + 0.5
                        state = "FINAL_REOBSERVATION" if final_observation else "REOBSERVING"
                        reason = (
                            "follow cycle budget reached; scheduling terminal observation"
                            if final_observation
                            else "follow waypoint reached; scheduling new observation"
                        )
                        self._publish_status(state, reason)
                        continue
                    if event["state"] == "SUCCESS" and task and task["mode"] == "long_horizon":
                        identity = self.task["target_identity"].complete_current()
                        self.task["target_identity"] = identity
                        completed_target_id = identity.completed_object_ids[-1]
                        next_stage = task["stage_index"] + 1
                        if task["stages"] is not None and next_stage < len(task["stages"]):
                            self.task["stage_index"] = next_stage
                            self.task["search_index"] = 0
                            self.task["search_base_yaw"] = None
                            self.pending_replan_at = rospy.Time.now().to_sec() + 0.5
                            self._log_event(
                                "stage_completed",
                                task["task_id"],
                                stage_index=task["stage_index"],
                                next_stage_index=next_stage,
                                completed_target_object_id=completed_target_id,
                            )
                            self._publish_status(
                                "NEXT_STAGE",
                                "long-horizon stage completed; scheduling next target",
                            )
                            continue
                    if (
                        task
                        and task["mode"] == "follow"
                        and event["state"] in {"TIMEOUT", "ERROR"}
                    ):
                        failed_follow_task_id = task["task_id"]
                    else:
                        self.task = None
                if failed_follow_task_id is not None:
                    self._stop_failed_follow_task(failed_follow_task_id)
                self._log_event(
                    "terminal",
                    task["task_id"] if task else None,
                    state=event["state"],
                    success=event["state"] == "SUCCESS",
                    reason=event["reason"],
                    final_yaw_error_rad=self.executor.yaw_error,
                )
                self._publish_status(event["state"], event["reason"])

    def _refresh_target_yaw(self, now):
        if self.yaw_refresh_hz <= 0.0 or not self._execution_open():
            return
        with self.lock:
            if self.executor.state != "WAITING_ARRIVAL":
                return
            yaw = self.executor.current_goal_yaw
            period = 1.0 / self.yaw_refresh_hz
            if yaw is None:
                return
            if self.last_yaw_refresh_at is not None and now - self.last_yaw_refresh_at < period:
                return
            self.last_yaw_refresh_at = now
        self.planning_yaw_pub.publish(Float64(data=math.degrees(yaw)))

    def _publish_pose_goal(self, point, yaw, metric_observation=None):
        if not self._execution_open():
            raise RuntimeError("attempted goal publication while execution gate is closed")
        publish_timing = None
        if metric_observation is not None:
            publish_timing = self._validate_follow_metric_observation(metric_observation)
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "world"
        goal.pose.position.x = float(point[0])
        goal.pose.position.y = float(point[1])
        goal.pose.position.z = float(point[2])
        _set_yaw(goal.pose.orientation, float(yaw))
        self.goal_pub.publish(goal)
        self.last_yaw_refresh_at = rospy.Time.now().to_sec()
        return publish_timing

    def _execution_open(self):
        ready, _reason = self._execution_preconditions(check_speed=False)
        return self.allow_execution and self.executor.enabled and ready

    def _execution_preconditions(self, check_speed):
        now = rospy.Time.now().to_sec()
        with self.lock:
            state = self.vehicle_state
            odom = self.latest_odom
            if self.require_armed and (state is None or not state.connected or not state.armed):
                return False, "PX4 must be connected and armed"
            if odom is None or now - odom["stamp"] > self.executor.odom_timeout_sec:
                return False, "VINS odometry must be fresh"
            if self.min_execution_z > 0.0 and odom["z"] < self.min_execution_z:
                return False, "VINS start altitude is below min_execution_z"
            if (
                check_speed
                and self.enable_max_speed > 0.0
                and odom["speed"] > self.enable_max_speed
            ):
                return False, "vehicle speed is above enable_max_speed"
        return True, "ready"

    def _log_event(self, event, task_id, **fields):
        try:
            record = self.experiment_logger.log(event, task_id, **fields)
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "SMPF experiment log failed: %s", exc)
            return
        try:
            self.artifact_writer.record_event(record)
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "SMPF trial summary failed: %s", exc)

    def _archive_model_responses(self, task_id, kind, client):
        responses = tuple(getattr(client, "raw_responses", ()))
        for response in responses:
            try:
                self.artifact_writer.write_response(task_id, kind, response)
            except Exception as exc:
                rospy.logwarn_throttle(5.0, "SMPF model response archive failed: %s", exc)

    def _publish_debug_image(self, task_id, phase, image, annotations, stamp):
        try:
            annotated, _path = self.artifact_writer.write_annotated_image(
                task_id,
                phase,
                image,
                annotations,
            )
            message = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            message.header.stamp = rospy.Time.from_sec(float(stamp))
            message.header.frame_id = "camera_color_optical_frame"
            self.debug_image_pub.publish(message)
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "SMPF annotated image archive failed: %s", exc)

    def _publish_debug_depth(self, task_id, phase, depth, annotations, stamp):
        try:
            annotated, _image_path, _raw_path = self.artifact_writer.write_depth_image(
                task_id,
                phase,
                depth,
                annotations,
            )
            message = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            message.header.stamp = rospy.Time.from_sec(float(stamp))
            message.header.frame_id = "camera_color_optical_frame"
            self.debug_depth_pub.publish(message)
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "SMPF depth image archive failed: %s", exc)

    def _archive_sphere_models(
        self,
        task_id,
        stamp,
        observed_entries,
        body_models,
        world_models,
    ):
        objects = []
        for index, (entry, body_model, world_model) in enumerate(
            zip(observed_entries, body_models, world_models)
        ):
            body_sphere, estimate = body_model
            world_sphere, _world_estimate = world_model
            objects.append(
                {
                    "role": "target" if index == 0 else "obstacle",
                    "object_id": entry.object_id,
                    "label": body_sphere.label,
                    "confidence": body_sphere.confidence,
                    "source": body_sphere.source,
                    "safety_radius_m": body_sphere.radius,
                    "body_flu": {"center_m": body_sphere.center},
                    "world": {"center_m": world_sphere.center},
                    "depth": {
                        "value_m": estimate.value_m,
                        "std_m": estimate.std_m,
                        "sample_count": estimate.sample_count,
                        "minimum_m": estimate.minimum_m,
                        "maximum_m": estimate.maximum_m,
                    },
                }
            )
        payload = {
            "schema": "gameuav.smpf.sphere_models.v1",
            "task_id": task_id,
            "frame_stamp": stamp,
            "sphere_safety_margin_m": self.sphere_margin_m,
            "objects": objects,
        }
        try:
            self.artifact_writer.write_geometry(task_id, "sphere_models", payload)
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "SMPF sphere model archive failed: %s", exc)
        self._publish_sphere_markers(stamp, world_models)

    def _publish_sphere_markers(self, stamp, world_models):
        marker_array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)
        for index, (sphere, _estimate) in enumerate(world_models):
            color = (0.2, 0.9, 0.2) if index == 0 else (1.0, 0.55, 0.1)
            marker = Marker()
            marker.header.stamp = rospy.Time.from_sec(float(stamp))
            marker.header.frame_id = "world"
            marker.ns = "smpf_object_spheres"
            marker.id = index * 2
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = sphere.center[0]
            marker.pose.position.y = sphere.center[1]
            marker.pose.position.z = sphere.center[2]
            marker.pose.orientation.w = 1.0
            marker.scale.x = sphere.radius * 2.0
            marker.scale.y = sphere.radius * 2.0
            marker.scale.z = sphere.radius * 2.0
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 0.28
            marker_array.markers.append(marker)

            label = Marker()
            label.header = marker.header
            label.ns = "smpf_object_sphere_labels"
            label.id = index * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = sphere.center[0]
            label.pose.position.y = sphere.center[1]
            label.pose.position.z = sphere.center[2] + sphere.radius + 0.12
            label.pose.orientation.w = 1.0
            label.scale.z = 0.16
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = "%s r=%.2fm" % (sphere.label, sphere.radius)
            marker_array.markers.append(label)
        self.debug_spheres_pub.publish(marker_array)

    def _status_timer_cb(self, _event):
        self._publish_status("ALIVE", "periodic health")

    def _publish_status(self, state, reason):
        with self.lock:
            now = rospy.Time.now().to_sec()
            frame_age = (
                None
                if self.latest_frame is None
                else max(0.0, now - self.latest_frame.stamp)
            )
            odom_age = None if self.latest_odom is None else max(0.0, now - self.latest_odom["stamp"])
            task = None
            if self.task:
                task = {
                    key: self.task[key]
                    for key in (
                        "task_id",
                        "mode",
                        "cycles",
                        "max_cycles",
                        "search_index",
                        "stage_index",
                        "execution_requested_at_submit",
                    )
                }
                task["stage_count"] = 0 if self.task["stages"] is None else len(self.task["stages"])
                identity = self.task["target_identity"]
                task["current_target_object_id"] = identity.current_object_id
                task["completed_target_object_ids"] = identity.completed_object_ids
            payload = {
                "schema": "gameuav.smpf.status.v1",
                "stamp": now,
                "state": state,
                "reason": str(reason),
                "execution_allowed_at_launch": self.allow_execution,
                "runtime_execution_enabled": self.runtime_execution_enabled,
                "execution_gate_open": self._execution_open(),
                "vehicle_connected": None if self.vehicle_state is None else self.vehicle_state.connected,
                "vehicle_armed": None if self.vehicle_state is None else self.vehicle_state.armed,
                "inference_busy": self.inference_busy,
                "task": task,
                "executor": self.executor.status(),
                "frame_age_sec": frame_age,
                "odom_age_sec": odom_age,
                "calibration_ready": all(
                    value is not None
                    for value in (self.body_from_infra1, self.infra1_from_depth, self.color_from_depth)
                ),
                "calibration_errors": dict(self.calibration_errors),
                "memory_object_count": len(self.memory.snapshot(now)),
                "last_error": self.last_error,
                "sam_endpoint": self.sam.endpoint,
                "artifact_root": str(self.artifact_writer.root),
                "debug_topics": {
                    "annotated_image": self.debug_image_topic,
                    "depth_image": self.debug_depth_topic,
                    "object_spheres": self.debug_spheres_topic,
                },
                "model_config_source": self.model_config_source,
                "models": {"vlm": self.vlm_model_id, "llm": self.llm_model_id},
                "llm_reasoning_effort": self.llm_reasoning_effort,
                "follow_policy": {
                    "goal_representation": "single_3d_tracking_point",
                    "trajectory_owner": "ego",
                    "relocalization": "one_full_frame_sam_after_vlm_label_grounding",
                    "sam_mask_min_count_required": 1,
                    "sam_mask_selection": "largest_area",
                    "metric_frame_max_age_sec": self.follow_metric_frame_max_age_sec,
                    "metric_rgbd_odom_skew_sec": self.follow_metric_odom_skew_sec,
                    "sam_timeout_sec": self.follow_sam_timeout_sec,
                    "target_surface_standoff_m": self.follow_target_surface_standoff_m,
                    "target_surface_tolerance_m": self.follow_target_surface_tolerance_m,
                    "step_limit_enabled": self.follow_step_limit_enabled,
                    "max_step_m": self.follow_max_step_m,
                    "llm_planning": False,
                },
                "ablation": {
                    "deterministic_fallback_enabled": self.deterministic_fallback_enabled,
                    "completed_target_exclusion_enabled": self.completed_target_exclusion_enabled,
                    "goal_condition_validation_enabled": self.goal_condition_validation_enabled,
                    "corridor_obstacle_filter_enabled": self.corridor_obstacle_filter_enabled,
                    "follow_step_limit_enabled": self.follow_step_limit_enabled,
                },
            }
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False, sort_keys=True)))


def main():
    rospy.init_node("smpf_bridge")
    SmpfBridgeNode()
    rospy.spin()


if __name__ == "__main__":
    main()

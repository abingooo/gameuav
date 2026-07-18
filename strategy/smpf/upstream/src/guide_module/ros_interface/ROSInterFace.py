#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import threading
import sys
from collections import deque
from typing import Optional, Tuple

import numpy as np
import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None  # type: ignore


class CameraReader:
    """
    CameraReader 订阅 RGB 与深度话题，提供最新帧的 numpy 数组
    """
    def __init__(
        self,
        color_topic: str = "/camera/color/image_raw",
        depth_topic: str = "/camera/aligned_depth_to_color/image_raw",
        queue_size: int = 10,
    ) -> None:
        self.color_topic = color_topic
        self.depth_topic = depth_topic
        self.bridge = CvBridge() if CvBridge else None
        self.bridge_active = self.bridge is not None

        self._lock = threading.Lock()
        self._color_array: Optional[np.ndarray] = None
        self._depth_array: Optional[np.ndarray] = None
        self.have_color = False
        self.have_depth = False

        self.color_sub = rospy.Subscriber(
            self.color_topic, Image, self._color_cb, queue_size=queue_size
        )
        self.depth_sub = rospy.Subscriber(
            self.depth_topic, Image, self._depth_cb, queue_size=queue_size
        )
        rospy.loginfo(
            "CameraReader subscribed to %s and %s", self.color_topic, self.depth_topic
        )

    # --- Public API ---
    def latest(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """返回最新的 (彩色, 深度) 数组，任一未收到则为 None。"""
        with self._lock:
            return self._color_array, self._depth_array

    def latest_color(self) -> Optional[np.ndarray]:
        with self._lock:
            return self._color_array

    def latest_depth(self) -> Optional[np.ndarray]:
        with self._lock:
            return self._depth_array

    # --- Callbacks ---
    def _color_cb(self, msg: Image) -> None:
        array = self._decode_color(msg)
        with self._lock:
            self._color_array = array
            self.have_color = array is not None

    def _depth_cb(self, msg: Image) -> None:
        array = self._decode_depth(msg)
        with self._lock:
            self._depth_array = array
            self.have_depth = array is not None

    # --- Decoding helpers ---
    def _decode_color(self, msg: Image) -> Optional[np.ndarray]:
        if self.bridge_active and self.bridge is not None:
            try:
                return self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception as err:
                rospy.logwarn_throttle(5.0, "cv_bridge failed for color image: %s", err)
                self.bridge_active = False
        return self._decode_color_manual(msg)

    def _decode_depth(self, msg: Image) -> Optional[np.ndarray]:
        if self.bridge_active and self.bridge is not None:
            try:
                return self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            except Exception as err:
                rospy.logwarn_throttle(5.0, "cv_bridge failed for depth image: %s", err)
                self.bridge_active = False
        return self._decode_depth_manual(msg)

    @staticmethod
    def _decode_color_manual(msg: Image) -> Optional[np.ndarray]:
        encoding = (msg.encoding or "").lower()
        mapping = {
            "bgr8": (np.uint8, 3),
            "rgb8": (np.uint8, 3),
            "mono8": (np.uint8, 1),
            "mono16": (np.uint16, 1),
        }
        if encoding not in mapping:
            rospy.logwarn_throttle(5.0, "Unsupported color encoding: %s", encoding)
            return None
        dtype, channels = mapping[encoding]
        expected = msg.height * msg.width * channels
        flat = np.frombuffer(msg.data, dtype=dtype, count=expected)
        if flat.size != expected:
            rospy.logwarn_throttle(
                5.0,
                "Color image size mismatch (encoding %s, got %d, expected %d)",
                encoding,
                flat.size,
                expected,
            )
            return None
        if channels == 1:
            image = flat.reshape((msg.height, msg.width))
            return image
        image = flat.reshape((msg.height, msg.width, channels))
        if encoding == "rgb8":
            # 手动将 RGB 顺序转换为 BGR
            return image[:, :, ::-1]
        return image

    @staticmethod
    def _decode_depth_manual(msg: Image) -> Optional[np.ndarray]:
        encoding = (msg.encoding or "").lower()
        mapping = {
            "32fc1": np.float32,
            "16uc1": np.uint16,
            "16sc1": np.int16,
        }
        if encoding not in mapping:
            rospy.logwarn_throttle(5.0, "Unsupported depth encoding: %s", encoding)
            return None
        dtype = mapping[encoding]
        expected = msg.height * msg.width
        flat = np.frombuffer(msg.data, dtype=dtype, count=expected)
        if flat.size != expected:
            rospy.logwarn_throttle(
                5.0,
                "Depth image size mismatch (encoding %s, got %d, expected %d)",
                encoding,
                flat.size,
                expected,
            )
            return None
        depth = flat.reshape((msg.height, msg.width))
        if dtype in (np.uint16, np.int16):
            return depth.astype(np.float32) / 1000.0  # convert mm to meters
        return depth.astype(np.float32)



class OdomReader:
    """
    OdomReader 订阅里程计话题，缓存最新的 nav_msgs/Odometry 消息，便于查询位置和姿态。
    """
    def __init__(self, odom_topic: str = "/vins_fusion/imu_propagate", queue_size: int = 10) -> None:
        self.odom_topic = odom_topic
        self._lock = threading.Lock()
        self._odom_msg: Optional[Odometry] = None
        self.have_odom = False

        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self._odom_cb, queue_size=queue_size
        )
        rospy.loginfo("OdomReader subscribed to %s", self.odom_topic)

    # --- Public API ---
    def latest(self) -> Optional[Odometry]:
        """返回最新的 Odometry 消息，尚未收到则为 None。"""
        with self._lock:
            return self._odom_msg

    def latest_pose(self) -> Tuple[Optional[Tuple[float, float, float]], Optional[Tuple[float, float, float, float]]]:
        """返回 (位置xyz, 姿态xyzw) 元组，若未收到则为 (None, None)。"""
        with self._lock:
            if self._odom_msg is None:
                return None, None
            p = self._odom_msg.pose.pose.position
            q = self._odom_msg.pose.pose.orientation
            return (round(p.x,2), round(p.y,2), round(p.z,2)), (round(q.x,2), round(q.y,2), round(q.z,2), round(q.w,2))

    # --- Callbacks ---
    def _odom_cb(self, msg: Odometry) -> None:
        with self._lock:
            self._odom_msg = msg
            self.have_odom = True


class ManageCmdBridge:
    """
    精简的 manage_cmd 收发桥。
      - publish(cmd_type: str, context: str)
      - receive(timeout: Optional[float]) -> Tuple[Optional[str], Optional[str]]
        每条消息只消费一次；timeout=None 阻塞等待，0 为非阻塞，其它为超时秒数。
    """

    def __init__(self, topic: str = "/manage_cmd", queue_size: int = 10, latch: bool = False) -> None:
        self.topic = topic
        self._queue = deque()
        self._cond = threading.Condition()
        self._pub = rospy.Publisher(self.topic, String, queue_size=queue_size, latch=latch)
        self._sub = rospy.Subscriber(self.topic, String, self._cb, queue_size=queue_size)
        rospy.loginfo("ManageCmdBridge ready on %s", self.topic)

    def publish(self, cmd_type: str, context: str) -> None:
        """发布一条管理指令。"""
        payload = {"type": cmd_type, "context": context}
        msg = String(data=json.dumps(payload, ensure_ascii=False))
        self._pub.publish(msg)

    def receive(self, timeout: Optional[float] = None) -> Tuple[Optional[str], Optional[str]]:
        """
        取出一条未消费的管理指令。
        timeout=None 时阻塞直到有新消息；timeout=0 为非阻塞；其它为超时秒数。
        返回 (type, context)，若无消息则返回 (None, None)。
        """
        with self._cond:
            if not self._queue:
                if timeout == 0:
                    return None, None
                self._cond.wait(timeout=timeout)
            if self._queue:
                return self._queue.popleft()
            return None, None

    def wait_for_subscribers(self, timeout: Optional[float] = None) -> bool:
        """
        等待至少有一个订阅者连接到发布者；timeout=None 表示一直等。
        返回 True 表示已连接，False 表示超时未连上。
        """
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self._pub.get_num_connections() > 0:
                return True
            if timeout is not None:
                elapsed = (rospy.Time.now() - start).to_sec()
                if elapsed >= timeout:
                    return False
            rate.sleep()
        return False

    # 内部订阅回调
    def _cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            cmd_type = data.get("type")
            context = data.get("context")
        except Exception:
            cmd_type = None
            context = msg.data
        with self._cond:
            self._queue.append((cmd_type, context))
            self._cond.notify()


class PlanPointPublisher:
    """
    简易航点发布器，封装单个 PoseStamped 发布流程。

    用法:
        pub = PlanPointPublisher(topic="/toplan/single_plan_point", frame_id="world")
        pub.publish_point(1.0, 2.0, 0.5)  # 默认姿态 w=1
    """

    def __init__(self, topic: str = "/toplan/single_plan_point", frame_id: str = "world", queue_size: int = 10) -> None:
        self.topic = topic
        self.frame_id = frame_id
        self._pub = rospy.Publisher(self.topic, PoseStamped, queue_size=queue_size)
        rospy.loginfo("PlanPointPublisher ready on %s", self.topic)

    def publish_point(self, point, orientation=None, stamp=None) -> None:
        """
        发布单个航点。
        Args:
            point:(x, y, z): 位置坐标（米）
            orientation: 可选四元数 (x, y, z, w)，默认 (0,0,0,1)
            stamp: 可选 rospy.Time，默认 rospy.Time.now()
        """
        x, y, z = point
        if orientation is None:
            orientation = (0.0, 0.0, 0.0, 1.0)
        ox, oy, oz, ow = orientation
        pose = PoseStamped()
        pose.header.stamp = stamp or rospy.Time.now()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.x = float(ox)
        pose.pose.orientation.y = float(oy)
        pose.pose.orientation.z = float(oz)
        pose.pose.orientation.w = float(ow)
        self._pub.publish(pose)

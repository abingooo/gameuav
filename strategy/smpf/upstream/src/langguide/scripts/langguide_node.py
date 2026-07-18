#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 导入必要的Python库
import base64
import json
import os
import sys
import time
import threading
from pathlib import Path
# 导入图像处理库
import cv2
import numpy as np

# 导入ROS相关库
import rospkg
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import LangCommand
from geometry_msgs.msg import PoseStamped

SCRIPT_DIR = Path(__file__).resolve().parent
EGOVLM_DIR = SCRIPT_DIR / "egovlm"
if SCRIPT_DIR.exists():
    sys.path.insert(0, str(SCRIPT_DIR))
try:
    from egovlm.lib.img_utils import ImageUtils
except ImportError as exc:
    raise ImportError(f"Failed to import egovlm; ensure {EGOVLM_DIR} exists and PYTHONPATH includes {SCRIPT_DIR}") from exc
try:
    from cv_bridge import CvBridge, CvBridgeError
except ImportError:  # 当cv_bridge不可用时提供备选方案
    CvBridge = None  # type: ignore
    CvBridgeError = RuntimeError  # type: ignore


class LangGuideNode:
    """订阅 RGB-D 话题、缓存最新图像并周期发布状态的基础节点。"""

    def __init__(self):
        # 航点完成标志
        self.waypoint_completed = False
        
        # 允许通过参数调整订阅/发布话题
        self.color_topic = rospy.get_param("~color_topic", "/camera/color/image_raw")
        self.depth_topic = rospy.get_param(
            "~depth_topic", "/camera/aligned_depth_to_color/image_raw"
        )
        self.status_topic = rospy.get_param("~status_topic", "langguide/status")
        # VINS-Fusion IMU传播位姿话题
        self.odom_topic = rospy.get_param("~odom_topic", "/vins_fusion/imu_propagate")
        # 语言指令话题
        self.lang_cmd_topic = rospy.get_param("~lang_cmd_topic", "/lang_cmd")
        # 规划点发布话题
        self.plan_point_topic = rospy.get_param("~plan_point_topic", "/toplan/single_plan_point")

        status_rate = rospy.get_param("~status_rate", 10.0)
        if status_rate <= 0.0:
            rospy.logwarn("status_rate <= 0 detected, resetting to 10 Hz")
            status_rate = 10.0

        # 数据可用性标记与缓存
        self.have_color = False
        self.have_depth = False
        self.have_odometry = False
        self.have_cmd = False
        self.have_plan = False
        self.warned_missing = False
        self.last_color = None
        self.last_depth = None
        self.last_odometry = None
        self.position = None  # 存储位置信息
        self.orientation = None  # 存储姿态四元数信息
        # 添加位置历史缓冲区，用于计算平均值
        self.position_history = []
        self.position_history_size = 10

        queue_size = rospy.get_param("~queue_size", 10)
        self.bridge = CvBridge() if CvBridge else None
        self.bridge_active = self.bridge is not None
        if not self.bridge_active:
            rospy.logwarn("cv_bridge unavailable, falling back to manual decoding.")
        pkg_path = rospkg.RosPack().get_path("langguide")
        workspace_root = os.path.abspath(os.path.join(pkg_path, "..", ".."))
        default_debug_dir = os.path.join(workspace_root, "debug")
        self.debug_dir = rospy.get_param("~debug_dir", default_debug_dir)
        self._ensure_debug_dir()
        self.color_path = os.path.join(self.debug_dir, "color_latest.jpg")
        self.depth_npy_path = os.path.join(self.debug_dir, "depth_latest.npy")
        self.depth_png_path = os.path.join(self.debug_dir, "depth_latest.jpg")
        # 添加伪彩色深度图像保存路径
        self.depth_color_path = os.path.join(self.debug_dir, "depth_color_latest.png")
        
        
        # 订阅彩色与深度图像
        self.color_sub = rospy.Subscriber(
            self.color_topic, Image, self._color_cb, queue_size=queue_size
        )
        self.depth_sub = rospy.Subscriber(
            self.depth_topic, Image, self._depth_cb, queue_size=queue_size
        )
        # 订阅VINS-Fusion的IMU传播位姿话题
        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self._odom_cb, queue_size=queue_size
        )
        # 订阅语言指令话题
        self.lang_cmd_sub = rospy.Subscriber(
            self.lang_cmd_topic, LangCommand, self._lang_cmd_cb, queue_size=queue_size
        )
        # 订阅航点完成话题
        self.waypoint_done_sub = rospy.Subscriber('/toplan/waypoint_done', String, self.waypoint_done_callback)

        # 发布状态字符串供其他模块监控
        self.status_pub = rospy.Publisher(
            self.status_topic, String, queue_size=10, latch=False
        )
        self.plan_data_pub = rospy.Publisher(
            "langguide/plan_data", String, queue_size=1, latch=True
        )
        self.plan_asset_pub = rospy.Publisher(
            "langguide/plan_assets", String, queue_size=1, latch=True
        )
        self.plan_cloud_pub = rospy.Publisher(
            "langguide/plan_pointcloud", String, queue_size=1, latch=True
        )
        self.plan_asset_names = {
            "rgb_image.jpg",
            "target2d_visual_in_rgbimage.jpg",
            "vlm_detection_result.jpg",
        }
        self.asset_stage_map = {
            "rgb_image.jpg": 0,
            "vlm_detection_result.jpg": 1,
            "target2d_visual_in_rgbimage.jpg": 2,
        }
        self.asset_mtimes = {name: 0.0 for name in self.plan_asset_names}
        self._install_image_hook()
        self._start_asset_watcher()
        # 规划点发布器
        self.plan_point_pub = rospy.Publisher(self.plan_point_topic, PoseStamped, queue_size=10)
        rospy.loginfo("PlanPointPublisher ready on %s", self.plan_point_topic)
        

        self.status_srv = rospy.Service("~data_status", Trigger, self._handle_status)

        # 定时器相当于 10Hz 主循环
        self.timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / status_rate), self._timer_cb
        )

        rospy.loginfo(
            "LangGuide node subscribed to %s, %s, %s, and %s, publishing %s at %.1f Hz",
            self.color_topic,
            self.depth_topic,
            self.odom_topic,
            self.lang_cmd_topic,
            self.status_topic,
            status_rate,
        )

    def _color_cb(self, msg):
        self.last_color = msg
        self.have_color = True
        if not self._save_color_via_bridge(msg):
            array = self._decode_color_manual(msg)
            if array is not None:
                try:
                    cv2.imwrite(self.color_path, array)
                except cv2.error as err:
                    rospy.logwarn_throttle(5.0, "Failed to save color image: %s", err)
            else:
                rospy.logwarn_throttle(
                    5.0, "Unsupported color encoding: %s", msg.encoding
                )

    def _depth_cb(self, msg):
        self.last_depth = msg
        self.have_depth = True
        depth_array = self._save_depth_via_bridge(msg)
        if depth_array is None:
            depth_array = self._decode_depth_manual(msg)
        if depth_array is None:
            rospy.logwarn_throttle(5.0, "Unsupported depth encoding: %s", msg.encoding)
            return
        try:
            np.save(self.depth_npy_path, depth_array)
            depth_normalized = cv2.normalize(depth_array, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            cv2.imwrite(self.depth_png_path, depth_colored)
            
        except (ValueError, cv2.error) as err:
            rospy.logwarn_throttle(5.0, "Failed to save depth data: %s", err)
    
    def _timer_cb(self, _event):
        if not (self.have_color and self.have_depth):
            if not self.warned_missing:
                self.warned_missing = True
                self.status_pub.publish(
                    String(data="Waiting for both color and depth images...")
                )
            return

        self.warned_missing = False
        self.status_pub.publish(String(data=self._compose_status()))
    
    def _odom_cb(self, msg):
        """处理VINS-Fusion的IMU传播位姿数据"""
        self.last_odometry = msg
        self.have_odometry = True
        # 提取位置信息
        self.position = msg.pose.pose.position
        # 提取姿态四元数信息
        self.orientation = msg.pose.pose.orientation
        
        # 更新位置历史缓冲区
        position_data = {
            'x': self.position.x,
            'y': self.position.y,
            'z': self.position.z
        }
        self.position_history.append(position_data)
        # 保持缓冲区大小不超过指定值
        if len(self.position_history) > self.position_history_size:
            self.position_history.pop(0)
            
        rospy.logdebug("Received odometry data: pos=(%f, %f, %f), orient=(%f, %f, %f, %f)",
                     self.position.x, self.position.y, self.position.z,
                     self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w)

    # 在_lang_cmd_cb方法中，将原来的subprocess调用替换为函数调用
    
    def _lang_cmd_cb(self, msg):
        """处理语言指令"""
        self.last_lang_cmd = msg
        self.have_cmd = True
        rospy.loginfo("\n")
        rospy.loginfo("="*40)
        rospy.loginfo("Received command type: %s", self.last_lang_cmd.command_type)
        rospy.loginfo("Received command content: %s", self.last_lang_cmd.content)
        if self.last_lang_cmd.command_type == "plan":
            self._plan()
        elif self.last_lang_cmd.command_type == "run":
            self._control()
        else:
            rospy.logwarn("Unknown command type: %s", self.last_lang_cmd.command_type)
    



    def _handle_status(self, _req):
        resp = TriggerResponse()
        if not (self.have_color and self.have_depth):
            resp.success = False
            resp.message = "Images not received yet"
            return resp

        resp.success = True
        resp.message = self._compose_status()
        return resp

    def _compose_status(self):
        now = rospy.Time.now()
        color_age = (now - self.last_color.header.stamp).to_sec()
        depth_age = (now - self.last_depth.header.stamp).to_sec()
        
        # 基本状态信息
        msg_parts = [
            f"Color stamp: {self.last_color.header.stamp.to_sec():.6f}",
            f" ({self.last_color.width}x{self.last_color.height}), ",
            f"Depth stamp: {self.last_depth.header.stamp.to_sec():.6f}",
            f" ({self.last_depth.width}x{self.last_depth.height}), "
        ]
        
        # 如果有里程计信息，添加到状态中
        if self.have_odometry and self.last_odometry:
            odom_age = (now - self.last_odometry.header.stamp).to_sec()
            msg_parts.extend([
                f"Odom stamp: {self.last_odometry.header.stamp.to_sec():.6f}, ",
                f"pos: ({self.position.x:.3f}, {self.position.y:.3f}, {self.position.z:.3f}), ",
                f"orient: ({self.orientation.w:.3f}, {self.orientation.x:.3f}, {self.orientation.y:.3f}, {self.orientation.z:.3f}), ",
                f"odom age: {odom_age:.3f}s, "
            ])
        if self.have_cmd and self.last_lang_cmd:
            msg_parts.extend([
                f"ctrl cmd: type:{self.last_lang_cmd.command_type}, content:{self.last_lang_cmd.content}"
            ])
            self.have_cmd = False
        # 添加图像数据年龄信息
        msg_parts.extend([
            f"color age: {color_age:.3f}s, depth age: {depth_age:.3f}s"
        ])
        
        return "".join(msg_parts)

    def _ensure_debug_dir(self):
        if not os.path.exists(self.debug_dir):
            os.makedirs(self.debug_dir)
            rospy.loginfo("Created debug directory at %s", self.debug_dir)

    @staticmethod
    def _depth_to_uint16(depth_array):
        # 支持 float32/float64 等深度编码，转换为毫米的 uint16，便于重复写入 PNG
        depth = np.nan_to_num(depth_array, nan=0.0, posinf=0.0, neginf=0.0)
        if depth.dtype == np.uint16:
            return depth
        # clip to 0-65.535m before转毫米
        depth_mm = np.clip(depth * 1000.0, 0, 65535)
        return depth_mm.astype(np.uint16)

    def _save_color_via_bridge(self, msg):
        if not self.bridge_active or self.bridge is None:
            return False
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv2.imwrite(self.color_path, cv_image)
            return True
        except Exception as err:  # broad to catch linkage/runtime errors
            rospy.logwarn_throttle(5.0, "cv_bridge failed for color image: %s", err)
            self.bridge_active = False
            return False

    def _save_depth_via_bridge(self, msg):
        if not self.bridge_active or self.bridge is None:
            return None
        try:
            depth_array = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            )
            return depth_array
        except Exception as err:
            rospy.logwarn_throttle(5.0, "cv_bridge failed for depth image: %s", err)
            self.bridge_active = False
            return None

    def _decode_color_manual(self, msg):
        encoding = (msg.encoding or "").lower()
        mapping = {
            "bgr8": (np.uint8, 3),
            "rgb8": (np.uint8, 3),
            "mono8": (np.uint8, 1),
            "mono16": (np.uint16, 1),
        }
        if encoding not in mapping:
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
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return image

    def _decode_depth_manual(self, msg):
        encoding = (msg.encoding or "").lower()
        mapping = {
            "32fc1": np.float32,
            "16uc1": np.uint16,
            "16sc1": np.int16,
        }
        if encoding not in mapping:
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
        # convert integer types to float meters for consistent downstream usage
        if dtype == np.uint16:
            return depth.astype(np.float32) / 1000.0
        if dtype == np.int16:
            return depth.astype(np.float32) / 1000.0
        return depth
    def _get_average_position(self):
        """计算位置历史的平均值"""
        if not self.position_history:
            return self.position
        
        avg_x = sum(pos['x'] for pos in self.position_history) / len(self.position_history)
        avg_y = sum(pos['y'] for pos in self.position_history) / len(self.position_history)
        avg_z = sum(pos['z'] for pos in self.position_history) / len(self.position_history)
        
        # 创建一个具有x, y, z属性的对象
        class AveragePosition:
            def __init__(self, x, y, z):
                self.x = x
                self.y = y
                self.z = z
        
        return AveragePosition(avg_x, avg_y, avg_z)
    
    # 规划数据坐标转换
    def _transform_data_c2w(self, plandata_camera):
        # 创建世界坐标系下的数据结构
        plandata_world = {
            'path_points': [],
            'object': []
        }
        
        # 获取平均位置
        avg_position = self._get_average_position()
        
        # 1. 处理路径点
        path_points = plandata_camera.get('path_points', [])
        for point in path_points:
            # 坐标方向转换
            transformed_point = [point[2], -point[0], -point[1]]
            # 使用平均位置平移到世界坐标系
            world_point = [
                round(transformed_point[0] + avg_position.x, 2),
                round(transformed_point[1] + avg_position.y, 2),
                round(transformed_point[2] + avg_position.z, 2)
            ]
            plandata_world['path_points'].append(world_point)
        
        # 2. 处理物体中心
        objects = plandata_camera.get('object', [])
        for obj in objects:
            # 创建物体的副本，避免修改原始数据
            world_obj = obj.copy()
            # 假设物体有一个center字段表示中心点坐标
            if 'center' in world_obj:
                center = world_obj['center']
                # 坐标方向转换
                transformed_center = [center[2], -center[0], -center[1]]
                # 使用平均位置平移到世界坐标系
                world_center = [
                    round(transformed_center[0] + avg_position.x, 2),
                    round(transformed_center[1] + avg_position.y, 2),
                    round(transformed_center[2] + avg_position.z, 2)
                ]
                world_obj['center'] = world_center
            # 添加转换后的物体到结果中
            plandata_world['object'].append(world_obj)
        
        return plandata_world
        
    def _afterplan(self,plandata_camera):
        # 坐标转换到世界坐标系
        self.plandata_world = self._transform_data_c2w(plandata_camera)
        # 打印规划信息
        rospy.loginfo("World Objects: %s", self.plandata_world['object'])
        rospy.loginfo("World Path Points: %s", self.plandata_world['path_points'])
        self._publish_plan_data()
        self._publish_plan_assets()
        self.have_plan = True

    def _json_friendly(self, value):
        if isinstance(value, np.generic):
            return value.item()
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, dict):
            return {k: self._json_friendly(v) for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [self._json_friendly(v) for v in value]
        return value

    def _publish_plan_data(self):
        payload = {
            "object": self.plandata_world.get("object", []),
            "path_points": self.plandata_world.get("path_points", []),
        }
        try:
            safe_payload = self._json_friendly(payload)
            msg = String(data=json.dumps(safe_payload, ensure_ascii=True))
            self.plan_data_pub.publish(msg)
        except Exception as exc:
            rospy.logerr("Failed to publish plan data: %s", exc)

    def _read_image_b64(self, path):
        try:
            with open(path, "rb") as f:
                data = f.read()
            return base64.b64encode(data).decode("ascii")
        except FileNotFoundError:
            return None
        except Exception as exc:
            rospy.logwarn("Failed to read image %s: %s", path, exc)
            return None

    def _publish_plan_assets(self):
        log_dir = os.path.join(os.path.dirname(__file__), "egovlm", "log")
        for name in self.plan_asset_names:
            path = os.path.join(log_dir, name)
            self._publish_single_asset(name, path)

    def _publish_single_asset(self, name, path):
        b64 = self._read_image_b64(path)
        if b64 is None:
            return
        stage = self.asset_stage_map.get(name, 0)
        payload = {"stage": stage, "asset": {"name": name, "mime": "image/jpeg", "data": b64}}
        try:
            msg = String(data=json.dumps(payload, ensure_ascii=True))
            self.plan_asset_pub.publish(msg)
        except Exception as exc:
            rospy.logerr("Failed to publish plan asset %s: %s", name, exc)

    def _install_image_hook(self):
        """Monkey-patch ImageUtils.save_image to publish tracked images when saved."""
        original_save = ImageUtils.save_image
        node = self

        def hooked_save(image, save_path):
            success = original_save(image, save_path)
            if success:
                basename = os.path.basename(save_path)
                if basename in node.plan_asset_names:
                    node._publish_single_asset(basename, save_path)
            return success

        ImageUtils.save_image = staticmethod(hooked_save)

    def _start_asset_watcher(self):
        """Poll the log dir for new images and publish when mtime changes."""
        log_dir = os.path.join(os.path.dirname(__file__), "egovlm", "log")

        def loop():
            rate = rospy.Rate(5)  # 5 Hz polling
            while not rospy.is_shutdown():
                for name in self.plan_asset_names:
                    path = os.path.join(log_dir, name)
                    try:
                        mtime = os.path.getmtime(path)
                    except OSError:
                        continue
                    if mtime > self.asset_mtimes.get(name, 0):
                        self.asset_mtimes[name] = mtime
                        self._publish_single_asset(name, path)
                rate.sleep()

        t = threading.Thread(target=loop, daemon=True)
        t.start()

    def _plan(self):
        # 调用egovlm的模块函数处理语言指令
        try:
            import sys
            import os
            # 添加egovlm目录到Python路径
            egovlm_path = os.path.join(os.path.dirname(__file__), 'egovlm')
            if egovlm_path not in sys.path:
                sys.path.append(egovlm_path)
            
            # 直接导入并调用egovlm_module中的process_command函数
            # 使用绝对导入路径来避免模块导入问题
            import sys
            import os
            
            # 获取当前脚本所在目录
            current_script_dir = os.path.dirname(os.path.abspath(__file__))
            # 添加egovlm目录到Python路径
            egovlm_dir = os.path.join(current_script_dir, 'egovlm')
            if egovlm_dir not in sys.path:
                sys.path.append(egovlm_dir)
            
            # 现在尝试导入egovlm_module
            try:
                from egovlm_module import process_command
            except ImportError:
                # 如果直接导入失败，尝试使用__import__函数
                import importlib.util
                spec = importlib.util.spec_from_file_location("egovlm_module", os.path.join(egovlm_dir, "egovlm_module.py"))
                egovlm_module = importlib.util.module_from_spec(spec)
                sys.modules["egovlm_module"] = egovlm_module
                spec.loader.exec_module(egovlm_module)
                process_command = egovlm_module.navigate_process
                
            
            # 在后台线程中执行，避免阻塞ROS节点
            from threading import Thread
            def run_egovlm():
                result = {}
                try:
                    # 切换工作目录到egovlm目录
                    original_dir = os.getcwd()
                    os.chdir(egovlm_path)
                    
                    # 将ROS Image消息转换为numpy数组
                    try:
                        # 转换彩色图像
                        if self.bridge_active and self.last_color is not None:
                            color_array = self.bridge.imgmsg_to_cv2(self.last_color, "bgr8")
                        else:
                            color_array = self._decode_color_manual(self.last_color)
                            
                        # 转换深度图像
                        if self.bridge_active and self.last_depth is not None:
                            depth_array = self.bridge.imgmsg_to_cv2(self.last_depth, "32FC1")
                        else:
                            depth_array = self._decode_depth_manual(self.last_depth)
                        
                        # 检查转换结果
                        if color_array is None or depth_array is None:
                            rospy.logerr("Failed to convert image data to numpy arrays")
                            return
                            
                        # rospy.loginfo("Image data converted successfully")
                        rospy.loginfo("Received Color image and depth data")
                        # 调用处理函数
                        if self.have_plan == False:
                            rospy.loginfo("Egovlm processing...")
                            times = time.time()
                            # 任务类型判定
                            from egovlm.lib.vls import getPrompt
                            tasktype_prompt = getPrompt(self.last_lang_cmd.content,"tasktype")
                            start_time = time.time()
                            from egovlm.egovlm_module import log_message
                            from egovlm.lib.vls import initModel
                            vlm, _ = initModel()
                            log_message(f"{vlm.model_id}开始任务类型判定")
                            tasktype_result =vlm.analyze_image(color_array, tasktype_prompt)
                            end_time = time.time()
                            log_message(f"{vlm.model_id} 任务类型判定耗时: {round(end_time - start_time, 1)}")
                            log_message(f"{vlm.model_id} 任务类型判定结果: {tasktype_result}")

                            if tasktype_result not in ["control","navigate","follow","search"]:
                                result['message'] = "task type error"
                        
                                return result
                            
                            elif tasktype_result == "navigate":
                                pass
                                
                            elif tasktype_result == "follow":
                                pass
                            elif tasktype_result == "search":
                                pass
                            elif tasktype_result == "control":
                                pass
                            result = process_command(self.last_lang_cmd.content, color_array, depth_array,debug=True)
                        else:
                            rospy.loginfo("have plan,egovlm cancel")

                    except Exception as e:
                        rospy.logerr("Error converting image data: %s", str(e))
                        return
                    
                    # 处理结果
                    if result['success']:
                        # 计算处理耗时
                        end_time = time.time()
                        processing_time = end_time - times
                        rospy.loginfo(f"Egovlm processing successful, time cost: {processing_time:.4f} seconds")    
                        self._afterplan(result)
                    else:
                        rospy.logerr("Egovlm processing failed: %s", result['message'])
                        
                except ImportError as e:
                    rospy.logerr("Failed to import egovlm_module: %s", str(e))
                except Exception as e:
                    rospy.logerr("Error running egovlm: %s", str(e))
                finally:
                    # 恢复原来的工作目录
                    os.chdir(original_dir)
            
            # 启动后台线程执行egovlm
            thread = Thread(target=run_egovlm)
            thread.daemon = True  # 设为守护线程，ROS节点关闭时自动终止
            thread.start()
            
        except Exception as e:
            rospy.logerr("Failed to call egovlm module: %s", str(e))

    
    def waypoint_done_callback(self, msg):
        """处理航点完成消息"""
        rospy.loginfo("Received waypoint done notification: %s", msg.data)
        self.waypoint_completed = True
    
    def _control(self):
        """处理控制指令"""
        rospy.loginfo("Received control command: %s", self.last_lang_cmd.content)
        # if self.have_plan == False:
        #     rospy.logerr("No plan available, please plan first")
        #     return
        if self.last_lang_cmd.content == "go":
            # 先丢掉第一个点，因为一般是无人机当前的位置
            self.plandata_world['path_points'].pop(0)
            targets = self.plandata_world['path_points']
            # targets = [[1.0,0.0,0.4],[3.0,1.5,0.4]]
            rospy.loginfo(f"targets: {targets}")
            rospy.loginfo(f"Starting control with {len(targets)} targets")
            for i, target in enumerate(targets):
                rospy.logdebug(f"Flying to target {i+1}/{len(targets)}: {target}")
                
                # 重置完成标志
                self.waypoint_completed = False
                
                # 发布航点
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id ="world"
                pose.pose.position.x = target[0]
                pose.pose.position.y = target[1]
                pose.pose.position.z = target[2]
                pose.pose.orientation.w = 1.0
                self.plan_point_pub.publish(pose)
                rospy.loginfo(
                            "Published single plan point: (%.2f, %.2f, %.2f)",
                            pose.pose.position.x,
                            pose.pose.position.y,
                            pose.pose.position.z,
                        )
                # 等待到达目标点
                rospy.loginfo("Waiting to reach target...")
                
                start = rospy.Time.now()
                rate = rospy.Rate(10)
                is_last_target = i == len(targets) - 1
                while (
                    not self.waypoint_completed
                    and not rospy.is_shutdown()
                ):
                    # 如果当前位置距离目标位置在0.5m半径内，则认为到达
                    if self.position is not None:
                        dx = self.position.x - target[0]
                        dy = self.position.y - target[1]
                        dz = self.position.z - target[2]
                        distance = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5
                        if is_last_target:
                            arrived_distance = 0.3
                        else:
                            arrived_distance = 0.5
                        if distance <= arrived_distance:
                            rospy.loginfo(
                                "Within %.2f m of target (distance=%.2f m), marking reached",
                                arrived_distance,distance
                            )
                            self.waypoint_completed = True
                            break

                    rate.sleep()
                rospy.loginfo(f"Reached target {i+1}/{len(targets)}")
            
            rospy.loginfo("All targets successfully!")
            # 开启规划使能
            self.have_plan = False
        elif self.last_lang_cmd.content == "cancel":
            # 清空self.plandata_world
            self.plandata_world = {}
            rospy.loginfo("cancel plan")
            # 开启规划使能
            self.have_plan = False
        else:
            rospy.logerr("Unknown control command: %s", self.last_lang_cmd.content)
        
        
        

def main():
    rospy.init_node("langguide_node")
    LangGuideNode()
    rospy.spin()


if __name__ == "__main__":
    main()

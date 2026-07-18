#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
GuideVLP: 简单的封装，初始化 LLM、VLM 和 PromptLoader。
"""

import os
import sys
import time
import rospy
from pathlib import Path
import json
import re
import math

# 确保可以找到包内模块
PKG_ROOT = Path(__file__).resolve().parents[1]
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

from ros_interface.ROSInterFace import CameraReader, OdomReader,PlanPointPublisher
from guide_vlm.lib.MLM import LLM, VLM
from guide_vlm.lib.Prompts import PromptLoader
from guide_vlm.lib.Solve3D import Solve3D
from utils.Logging import Logging
from utils.Image import ImageProcess,ImageSave
from guide_vlm.lib.SAM import SAMClient


log = Logging(also_stdout=True, use_color=True, level="debug")
Solve3d = Solve3D()

class GuideVLP:
    def __init__(
        self,
        camera_reader: CameraReader,
        odom_reader: OdomReader,
        llm: LLM = None,
        vlm: VLM = None,
        prompts: PromptLoader = None,
        samclient = None,
        rate=None,
        task_config_path: str = None,
        map_config_path: str = None,
    ):
        # 依赖由外部创建并注入，便于统一管理生命周期
        self.camera_reader = camera_reader
        self.odom_reader = odom_reader
        self.point_pub = PlanPointPublisher()
        self.llm = llm or LLM()
        self.vlm = vlm or VLM(model_id="qwen3.5-plus")
        self.prompts = prompts or PromptLoader()
        # self.samclient = samclient or SAMClient(server_ip="47.108.251.163", server_port=5000, timeout=10)
        self.samclient = samclient or SAMClient(timeout=10)
        # 初始化循环频率，若外部未传入则默认 10Hz
        self.rate = rate or rospy.Rate(10)
        # 任务参数配置
        default_task_config = task_config_path or str(PKG_ROOT / "config" / "config.json")
        self.task_params = self._load_task_params(default_task_config)
        # 地图边界配置
        default_map_config = map_config_path or str(PKG_ROOT / "config" / "config.json")
        self.map_limits = self._load_map_limits(default_map_config)

        self.lastpos2d = [0,0]
        self.disp2d = [0,0]

    @staticmethod
    def _load_task_params(path: str):
        config_dir = PKG_ROOT / "config"
        candidates = []
        if path:
            candidates.append(Path(path))
        candidates.append(config_dir / "config.json")
        candidates.append(config_dir / "taskparam.json")

        seen = set()
        for candidate in candidates:
            cpath = Path(candidate)
            if cpath in seen:
                continue
            seen.add(cpath)
            try:
                with cpath.open("r", encoding="utf-8") as f:
                    data = json.load(f)
            except FileNotFoundError:
                continue
            except Exception as exc:
                log.error(f"读取任务参数失败 {cpath}: {exc}", color="red")
                continue

            if isinstance(data, dict) and "taskparam" in data:
                data = data["taskparam"]
            if isinstance(data, dict):
                return data
            log.error(f"任务参数格式无效: {cpath}", color="red")

        log.error("未找到可用的任务参数配置", color="red")
        return {}

    def process(self, cmd: str):
        """执行一次任务类型判断，返回提示词和 VLM 结果。"""
        image_np = self.camera_reader.latest_color()
        if image_np is None:
            raise RuntimeError("未获取到彩色图像")
        type_result = self.vlm.analyze(image_np, self.prompts.get("type", instruction=cmd),repeat=1)
        log.info(f"任务类型检测结果:{type_result}",color="orange_bold")
        self.task_run(type_result,cmd)

    def task_run(self,type_result,cmd):
        if type_result["content"] == "control":
            self.control_task(cmd)
        elif type_result["content"] == "navigate":
            self.navigate_task(cmd)
        elif type_result["content"] == "follow":
            self.follow_task(cmd)
        elif type_result["content"] == "search":
            self.search_task(cmd)

    def control_task(self,cmd):
        # 这是一个单次任务，只需要调用LLM大模型生成控制指令，然后发布即可（没有参照物，比较危险）
        
        # 获取当前vins绝对位置
        now_pos,now_ort = self.get_now_odom()
        log.info(f"当前位置：{now_pos},当前朝向:{now_ort}")
        # 调用llm生成控制航点
        
        # 全局仲裁(航点障碍判断、地图边界)

        # 发布目标航点序列

        pass

    def navigate_task(self,cmd):
        # 这是一个单次任务，先用VLM识别目标物体位置，然后调用LLM生成导航航点，最后发布航点、
        base_log = "/home/uav/lab/muav/src/guide_module/log/navigate/"
        # 获取当前vins绝对位置和当前视野
        now_pos,now_ort = self.get_now_odom()
        color, depth = self.get_now_imgdep()
        ImageSave.save_numpy_image(color,base_log+"1vlminput.jpg")
        log.info(f"当前位置：{now_pos},当前朝向:{now_ort}")
        # 使用vlm计算物体在视野中的位置bbox
        detect_result = self.vlm.analyze(color, self.prompts.get("detect", instruction=cmd),repeat=3)
        log.info(f"VLM识别结果:{detect_result}")
        # 若视野丢失，则调用搜索任务，并continue（设定计时，超时则返回跟随失败）
        if not detect_result["content"][0].get("box_2d"):
            # 调用搜索任务
            log.info("目标丢失，启动搜索任务",color="deep_blue_bold")
            detect_result = self.search_task(cmd)

        detection = detect_result["content"][0]

        # 坐标反归一化
        label = detection.get("label", "unknown")
        box_2d = ImageProcess.denormalize_bbox_1000(detection.get("box_2d"), color.shape)
        ImageSave.save_with_bbox_and_text(color,box_2d,label,base_log+"2vlmoutput.jpg")
        log.debug(f"锁定目标：{label}({box_2d})")
        # 视野外扩
        res = ImageProcess.extract_target_region(color, box_2d)
        ImageSave.save_numpy_image(res["imageROI"],base_log+"3saminput.jpg")

        # 调用sam，获得分割中心点和随机序列
        sam_result = self.samclient.predict(res["imageROI"], label)
        # 点还原 
        sam_points_dict = ImageProcess.samresult_adjust(sam_result,box_2d)
        ImageSave.save_sam_result(color,sam_points_dict,label,base_log+"4samoutput.jpg")
        # log.debug(f"sam result:{sam_points_dict}")
        # 移动方向计算，便于丢失后搜索
        self.record_2d_displacement(sam_points_dict["center"])
        self.lastpos2d = sam_points_dict["center"]
        # 对物体进行3D球体建模
        sphere_model = Solve3d.build_sphere_model_from_sam(
            depth,
            sam_points_dict,
            min_radius=self.task_params.get("navigate", {}).get("min_radius", 0.3),
            radius_scale=0.56,
            depth_mode="median_clipped",
            prefer_axis=2,
            prefer_max=True,
            precision=3,
        )
        log.info(
            f"目标球体建模(JSON): {json.dumps(sphere_model, ensure_ascii=False)}",
            color="green_bold",
        )
        # 调用llm生成导航航点序列
        plan_result = self.llm.chat_text(self.prompts.get("plan", instruction=cmd,objects_json=json.dumps(sphere_model, ensure_ascii=False)))
        points = plan_result["guidepoints"]
        points = self.add_vectors([now_pos]*len(points), points)  # 加上位移向量，补偿目标移动
        log.info(f"LLM规划结果(绝对坐标): {points}", color="green_bold")
        # 全局仲裁(航点障碍判断、地图边界)
        self.clamp_to_map(points)
        # 发布目标航点序列
        for idx, point in enumerate(points):
            log.info(f"发布航点 {idx+1}/{len(points)}: {point}")
            # self.point_pub.publish_point(point)
            # 阻塞等待
            self.waitfor_arrived(point,0.5,-1)
        log.info("导航任务完成", color="green_bold")


    def follow_task(self,cmd):
        # 这是一个迭代任务，一直迭代发布航点，直到飞到目标范围内，
        # 中间如果遇到目标丢失，则调用搜索任务，如果丢失超时或者
        # 已经到达目标范围内，则视为完成任务
        base_log = "/home/uav/lab/muav/src/guide_module/log/follow/"
        self.disp2d = [0,0]  
        while not rospy.is_shutdown():
            self.rate.sleep()
            # 获取当前vins绝对位置和当前视野
            now_pos,now_ort = self.get_now_odom()
            color, depth = self.get_now_imgdep()
            ImageSave.save_numpy_image(color,base_log+"1vlminput.jpg")
            log.info(f"当前位置：{now_pos},当前朝向:{now_ort}")
            # 使用vlm计算物体在视野中的位置bbox
            detect_result = self.vlm.analyze(color, self.prompts.get("detect", instruction=cmd),repeat=3)
            log.info(f"VLM识别结果:{detect_result}")
            # 若视野丢失，则调用搜索任务，并continue（设定计时，超时则返回跟随失败）
            if not detect_result["content"][0].get("box_2d"):
                # 调用搜索任务
                log.info("目标丢失，启动搜索任务",color="deep_blue_bold")
                detect_result = self.search_task(cmd)
            # 坐标反归一化
            detection = detect_result["content"][0]
            label = detection.get("label", "unknown")
            box_2d = ImageProcess.denormalize_bbox_1000(detection.get("box_2d"), color.shape)
            ImageSave.save_with_bbox_and_text(color,box_2d,label,base_log+"2vlmoutput.jpg")
            log.debug(f"锁定目标：{label}({box_2d})")
            # 视野外扩
            res = ImageProcess.extract_target_region(color, box_2d)
            ImageSave.save_numpy_image(res["imageROI"],base_log+"3saminput.jpg")
            # 调用sam，获得分割中心点和随机序列
            sam_result = self.samclient.predict(res["imageROI"], label)
            # 点还原 
            sam_points_dict = ImageProcess.samresult_adjust(sam_result,box_2d)
            ImageSave.save_sam_result(color,sam_points_dict,label,base_log+"4samoutput.jpg")
            # log.debug(f"sam result:{sam_points_dict}")
            # 解算3d位置
            points_for_depth = [sam_points_dict["center"], *sam_points_dict["rdmpoints"]]
            # 移动方向计算，便于丢失后搜索
            self.record_2d_displacement(sam_points_dict["center"])
            self.lastpos2d = sam_points_dict["center"]
            objpos = Solve3d.calculate_3d_position(sam_points_dict["center"],Solve3d.depth_stats(depth, points_for_depth, mode="min_clipped"))
            # 进行任务到达裁定
            dt = self.task_params.get("follow", {}).get("arrived_meter")
            flag,ds = self.arrived_check(now_pos,objpos,dt)
            if flag:
                log.info(f"arrived with distance {ds}",color="red")
                continue
            log.info(f"now distance {ds}",color="red")
            # 设置目标航点
            rlt_tgt3dp = Solve3d.point_at_distance_from_b((0,0,0),objpos,dt*0.8,2)
            abs_tgt3dp = Solve3d.camera2world_transform(now_pos, now_ort, rlt_tgt3dp, precision=2)
            # 全局仲裁(航点地图边界)
            self.clamp_to_map(abs_tgt3dp)
            # 发布目标航点
            self.point_pub.publish_point(abs_tgt3dp)
            # 阻塞等待
            self.waitfor_arrived(abs_tgt3dp,0.3,5)
                

    def search_task(self,cmd):
        # 这是一个迭代任务，一直迭代发布航点，直到找到目标物体位置，
        # 中间如果遇到目标物体，则视为完成任务
        base_log = "/home/uav/lab/muav/src/guide_module/log/search/"
        while not rospy.is_shutdown():
            self.rate.sleep()
            # 获取当前vins绝对位置和当前视野
            now_pos,now_ort = self.get_now_odom()
            color, depth = self.get_now_imgdep()
            ImageSave.save_numpy_image(color,base_log+"1vlminput.jpg")
            log.info(f"当前位置：{now_pos},当前朝向:{now_ort}")
            # 使用vlm计算物体在视野中的位置bbox
            detect_result = self.vlm.analyze(color, self.prompts.get("detect", instruction=cmd),repeat=3)
            log.info(f"VLM识别结果:{detect_result}")
            # 若视野找到目标物体，则返回成功
            if detect_result["content"] and detect_result["content"][0].get("box_2d"):
                log.info("搜索任务完成，已找到目标物体",color="green_bold")
                detection = detect_result["content"][0]
                label = detection.get("label", "unknown")
                box_2d = ImageProcess.denormalize_bbox_1000(detection.get("box_2d"), color.shape)
                ImageSave.save_with_bbox_and_text(color,box_2d,label,base_log+"2vlmoutput.jpg")
                # 返回目标信息
                return detect_result
            log.info("搜索任务进行中...",color="orange_bold")

            # 无人机进行旋转寻找目标物体或者为了获得更大视野进行平移（实际上是一个导航任务，物标为更空旷的区域）
            
                # 飞机进行偏航旋转，记录角度和视野照片

                # 多线程VLM分析多角度图像

                # 汇总分析结果：
                    # A.得到更大视野区域所在的方向

                        # SAM获得分割中心点和随机序列

                        # 解算3d位置

                        # 设置目标航点

                        # 全局仲裁(航点障碍判断、地图边界)

                        # 发布目标航点

                    # B.找到了目标物体位置，更新cmd调用follow或者navigate任务
                    
    @staticmethod
    def _sample_to_center(sample):
        """
        将一次采样转换为中心点坐标。
        支持格式：
        - dict: 优先使用 box_2d，其次 center
        - list/tuple 长度4: box_2d [ymin, xmin, ymax, xmax]
        - list/tuple 长度>=2: 点坐标 [x, y]
        """
        if sample is None:
            return None
        if isinstance(sample, dict):
            if "box_2d" in sample:
                return GuideVLP._sample_to_center(sample.get("box_2d"))
            if "center" in sample:
                return GuideVLP._sample_to_center(sample.get("center"))
        if isinstance(sample, (list, tuple)):
            if len(sample) >= 4:
                ymin, xmin, ymax, xmax = sample[:4]
                cx = (float(xmin) + float(xmax)) / 2.0
                cy = (float(ymin) + float(ymax)) / 2.0
                return (cx, cy)
            if len(sample) >= 2:
                return (float(sample[0]), float(sample[1]))
        return None

    def record_2d_displacement(self, nowpos2d):
        """
        根据两次采样记录物体在图像平面的位移向量 (dx, dy)，单位为像素。
        支持输入 box_2d/center 字段的 dict，或 [ymin, xmin, ymax, xmax] 框，或 [x, y] 点。
        """
        p2 = self._sample_to_center(nowpos2d)
        p1 = self.lastpos2d
        if p1 is None or p2 is None:
            raise ValueError(f"无法解析采样点: {self.lastpos2d} / {nowpos2d}")
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        self.disp2d = [dx, dy]
        self.lastpos2d = nowpos2d
        log.debug(f"2D 位移向量: dx={dx:.2f}, dy={dy:.2f}")

    def get_now_odom(self):
        while not rospy.is_shutdown():
            now_pos, now_ort = self.odom_reader.latest_pose()
            if now_pos is None or now_ort is None:
                rospy.loginfo_throttle(5.0, "等待里程计数据...")
                self.rate.sleep()
                continue
            return now_pos,now_ort

    def get_now_imgdep(self):
        while not rospy.is_shutdown():
            color, depth = self.camera_reader.latest()
            if color is None or depth is None:
                rospy.loginfo_throttle(5.0, "等待彩色图像和深度数据...")
                self.rate.sleep()
                continue
            return color, depth
    
    def arrived_check(self,npos,tpos,distance):
        """
        判断当前是否到达目标点。
        Args:
            npos: 当前坐标 (x,y,z) 或长度>=3 的可迭代。
            tpos: 目标坐标 (x,y,z) 或长度>=3 的可迭代。
            distance: 判定阈值，米。若 None/<=0 则使用 task_params 中 follow.arrived_meter，默认 1.0。
        Returns:
            bool 是否到达；float 当前距离。
        """
        if npos is None or tpos is None:
            return False, None
        try:
            cx, cy, cz = float(npos[0]), float(npos[1]), float(npos[2])
            tx, ty, tz = float(tpos[0]), float(tpos[1]), float(tpos[2])
        except Exception:
            return False, None

        if distance is None or distance <= 0:
            distance = 1.0

        dist = math.sqrt((cx - tx) ** 2 + (cy - ty) ** 2 + (cz - tz) ** 2)
        return dist <= distance, dist


    def clamp_to_map(self, point):
        """
        全局仲裁：若目标点超出地图边界，则裁剪到边界。
        Args:
            point: 可迭代 (x, y, z)
        Returns:
            tuple (x, y, z) 经过裁剪后的坐标
        """
        if point is None:
            return None
        try:
            x, y, z = float(point[0]), float(point[1]), float(point[2])
        except Exception as exc:
            log.error(f"目标点格式错误: {point}", color="red")
            raise exc

        limits = self.map_limits or {}
        x_min = limits.get("min_x", x)
        x_max = limits.get("max_x", x)
        y_min = limits.get("min_y", y)
        y_max = limits.get("max_y", y)
        z_min = limits.get("min_z", z)
        z_max = limits.get("max_z", z)

        clamped = (
            min(max(x, x_min), x_max),
            min(max(y, y_min), y_max),
            min(max(z, z_min), z_max),
        )
        if clamped != (x, y, z):
            log.info(f"目标点超出地图边界，已裁剪到 {clamped}", color="orange_bold")
        return clamped
    
    def waitfor_arrived(self, abs_tgt3dp, ds, timeout_sec: float = 10.0):
        """
        等待到达目标点，支持超时。
        Args:
            abs_tgt3dp: 目标点 (x, y, z)
            ds: 预期距离（未使用，仅兼容原签名）
            timeout_sec: 超时时间秒，<0 则一直等待 =0 不等待 >0 等待timeout_sec秒
        Returns:
            bool 是否到达(超时或中断返回 False)
        """
        if timeout_sec == 0:
            return 1
        rate = rospy.Rate(5)
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            rate.sleep()
            crt_pos, _ = self.odom_reader.latest_pose()
            flag, ds_now = self.arrived_check(crt_pos, abs_tgt3dp, ds)
            if flag:
                log.info(f"arrived pos({abs_tgt3dp}) with distance:{ds_now}")
                return 2
            if timeout_sec is not None and timeout_sec > 0:
                if (rospy.Time.now() - start).to_sec() >= timeout_sec:
                    log.warn(f"waitfor_arrived timeout after {timeout_sec}s, last distance {ds_now}")
                    return 3
        return 0

    @staticmethod
    def _load_map_limits(path: str):
        """
        读取地图边界配置，优先 config.json 的 mapsize 段，回退 mapsize.json。
        """
        config_dir = PKG_ROOT / "config"
        candidates = []
        if path:
            candidates.append(Path(path))
        candidates.append(config_dir / "config.json")
        candidates.append(config_dir / "mapsize.json")

        seen = set()
        for candidate in candidates:
            cpath = Path(candidate)
            if cpath in seen:
                continue
            seen.add(cpath)
            try:
                with cpath.open("r", encoding="utf-8") as f:
                    data = json.load(f)
            except FileNotFoundError:
                continue
            except Exception as exc:
                log.error(f"读取地图边界失败 {cpath}: {exc}", color="red")
                continue

            if isinstance(data, dict) and "mapsize" in data:
                data = data["mapsize"]
            if isinstance(data, dict):
                return data
            log.error(f"地图边界配置格式无效: {cpath}", color="red")

        log.error("未找到可用的地图边界配置", color="red")
        return {}

class Controller:
    pass

class follower:
    pass

class Navigator:
    pass

class Searcher:
    pass




if __name__ == "__main__":
    try:
        cam = CameraReader()
        odom = OdomReader()
        guide = GuideVLP(cam, odom)
        print("GuideVLP initialized. LLM:", guide.llm.model_id, "VLM:", guide.vlm.model_id)
    except Exception as exc:
        print("GuideVLP init failed:", exc)
    # 可选：在正式调用前等待收到图像
    # print(guide.process("飞到树的前面"))

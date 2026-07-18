#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import rospy
from std_msgs.msg import String

# 确保当前文件所在目录及其子目录加入模块搜索路径
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from ros_interface.ROSInterFace import CameraReader, OdomReader, ManageCmdBridge
from guide_vlm.GuideVLP import GuideVLP
from utils.Logging import Logging

log = Logging(also_stdout=True, use_color=True, level="debug")

def main():
    rospy.init_node("guide_node")
    # 初始化基础资源
    camera_reader = CameraReader()
    odom_reader = OdomReader()
    guide = GuideVLP(camera_reader, odom_reader)
    # 订阅管理端指令
    cmd_bridge = ManageCmdBridge(topic="/manage_cmd")

    rate = rospy.Rate(1.0)
    try:
        while not rospy.is_shutdown():
            rate.sleep()
            log.info_every(5, "等待指令...", counter_name="wait_cmd")
            cmd_type, cmd_ctx = cmd_bridge.receive(0)
            if cmd_type is None or cmd_ctx is None:
                continue
            log.debug("收到 manage_cmd: type=%s, context=%s", cmd_type, cmd_ctx)
            if cmd_type == "plan":
                # 简单等待获取到一帧图像
                if not camera_reader.have_color:
                    log.warn("等待彩色图像...")
                    continue
                try:
                    result = guide.process(cmd_ctx)
                    log.info("GuideVLP 结果: %s", result)
                except Exception as exc:
                    log.error("GuideVLP 处理失败: %s", exc)
            elif cmd_type == "ctrl":
                # 进行系统控制
                pass

            else:
                log.error("cmd type error")

    except KeyboardInterrupt:
        rospy.loginfo("GuideNode 停止（Ctrl+C）")


if __name__ == "__main__":
    main()

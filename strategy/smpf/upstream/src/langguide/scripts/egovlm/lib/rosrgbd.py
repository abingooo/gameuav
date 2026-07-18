# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import os
from datetime import datetime
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ROSRGBD:
    def __init__(self, node_name='realsense_image_processor', log_enabled=True, log_level='quiet', timeout=5.0):
        """
        初始化ROSRGBD类
        
        Args:
            node_name (str): ROS节点名称
            log_enabled (bool): 是否启用日志记录
            log_level (str): 日志详细程度级别 ('quiet', 'error', 'warn', 'info', 'debug')
            timeout (float): 等待图像数据的超时时间（秒）
        """
        self.log_enabled = log_enabled
        self.log_level = log_level
        self.timeout = timeout
        
        # 初始化图像数据变量
        self.color_image = None
        self.depth_image = None
        self.b_color_received = False
        self.b_depth_received = False
        
        # 创建CvBridge实例
        self.bridge = CvBridge()
        
        # 初始化ROS节点
        try:
            rospy.init_node(node_name, anonymous=True)
            self._log("Realsense图像处理器节点已启动")
            
            # 订阅图像话题
            self._subscribe_topics()
            
            # 等待图像数据
            self._wait_for_images()
            
        except Exception as e:
            self._log(f"初始化过程中发生错误: {str(e)}", level='error')
            raise
    
    def _log(self, message, level='info'):
        """
        统一的日志记录方法
        
        Args:
            message (str): 日志消息
            level (str): 日志级别 ('info', 'warn', 'error', 'fatal')
        """
        if not self.log_enabled:
            return
            
        # 根据日志级别控制输出
        level_priority = {
            'quiet': 0,
            'error': 1,
            'warn': 2,
            'info': 3,
            'debug': 4
        }
        
        # 计算当前消息的优先级
        message_priority = 0
        if level == 'error' or level == 'fatal':
            message_priority = 1
        elif level == 'warn':
            message_priority = 2
        elif level == 'info':
            message_priority = 3
        else:  # debug或其他
            message_priority = 4
        
        # 只有当消息优先级小于等于设置的日志级别优先级时才输出
        if message_priority <= level_priority.get(self.log_level, 3):
            if level == 'info':
                rospy.loginfo(message)
            elif level == 'warn':
                rospy.logwarn(message)
            elif level == 'error':
                rospy.logerr(message)
            elif level == 'fatal':
                rospy.logfatal(message)
    
    def _color_image_callback(self, data):
        """彩色图像回调函数"""
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.b_color_received = True
            # self._log(f"彩色图像已接收，分辨率: {self.color_image.shape[1]}x{self.color_image.shape[0]}")
        except Exception as e:
            self._log(f"彩色图像转换错误: {str(e)}", level='error')
    
    def _depth_image_callback(self, data):
        """深度图像回调函数"""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
            self.b_depth_received = True
            # self._log(f"深度图像已接收，分辨率: {self.depth_image.shape[1]}x{self.depth_image.shape[0]}")
        except Exception as e:
            self._log(f"深度图像转换错误: {str(e)}", level='error')
    
    def _subscribe_topics(self):
        """订阅图像话题"""
        # 订阅彩色图像话题
        self.color_sub = rospy.Subscriber("/camera/color/image_raw", Image, self._color_image_callback)
        if self.color_sub:
            self._log("已成功订阅彩色图像话题: /camera/color/image_raw")
        else:
            self._log("订阅彩色图像话题失败: /camera/color/image_raw", level='error')
        
        # 订阅深度图像话题
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self._depth_image_callback)
        if self.depth_sub:
            self._log("已成功订阅深度图像话题: /camera/aligned_depth_to_color/image_raw")
        else:
            self._log("订阅深度图像话题失败: /camera/aligned_depth_to_color/image_raw", level='error')
        
        # 检查是否至少有一个话题订阅成功
        if not self.color_sub and not self.depth_sub:
            error_msg = "所有图像话题订阅失败，请检查话题名称和Realsense节点是否正常运行！"
            self._log(error_msg, level='fatal')
            rospy.signal_shutdown("话题订阅失败")
            raise RuntimeError(error_msg)
    
    def _wait_for_images(self):
        """等待图像数据接收"""
        # 等待图像数据接收，设置超时时间
        timeout_duration = rospy.Duration(self.timeout)
        start_time = rospy.Time.now()
        
        # 等待直到收到至少一个图像或超时
        while not rospy.is_shutdown() and not (self.b_color_received or self.b_depth_received):
            if rospy.Time.now() - start_time > timeout_duration:
                # 根据订阅状态给出更详细的超时信息
                if self.color_sub and not self.b_color_received:
                    self._log("超时警告: 已订阅彩色图像话题但未收到数据", level='error')
                if self.depth_sub and not self.b_depth_received:
                    self._log("超时警告: 已订阅深度图像话题但未收到数据", level='error')
                
                # 如果没有收到任何图像数据，给出错误提示
                if not self.b_color_received and not self.b_depth_received:
                    self._log("等待图像超时，未收到任何图像数据，请检查Realsense相机是否正常工作以及话题是否正确发布！", level='error')
                break
            self._log("等待图像数据中...")
            rospy.sleep(0.5)  # 每0.5秒检查一次
    
    def getRGBD(self):
        """
        获取当前的RGB图像、深度图像和状态
        
        Returns:
            tuple: (color_image, depth_image, status)
                - color_image: 彩色图像 (numpy数组)，如果未收到则为None
                - depth_image: 深度图像 (numpy数组)，如果未收到则为None
                - status: 状态字典，包含彩色图像和深度图像的接收状态
        """
        status = {
            'color_received': self.b_color_received,
            'depth_received': self.b_depth_received,
            'both_received': self.b_color_received and self.b_depth_received
        }
        
        self._log(f"获取RGBD数据: 彩色图像{'已' if self.b_color_received else '未'}接收, 深度图像{'已' if self.b_depth_received else '未'}接收")
        
        return self.color_image, self.depth_image, status
    
    def save_images(self):
        """
        保存图像和深度数据到日志目录
        
        Returns:
            bool: 保存是否成功
        """
        try:
            # 创建基于时间戳的保存目录
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            base_dir = "./log"
            save_dir = os.path.join(base_dir, timestamp)
            
            # 确保基础目录存在
            if not os.path.exists(base_dir):
                os.makedirs(base_dir)
                self._log(f"已创建基础目录: {base_dir}")
            
            # 创建时间戳子文件夹
            if not os.path.exists(save_dir):
                os.makedirs(save_dir)
                self._log(f"已创建时间戳保存目录: {save_dir}")
            
            # 创建日志文件
            log_file_path = os.path.join(save_dir, "capture_log.txt")
            log_lines = [f"Capture Timestamp: {timestamp}", ""]
            
            # 保存彩色图像
            if self.b_color_received:
                color_path = os.path.join(save_dir, "color_image.jpg")
                success = cv2.imwrite(color_path, self.color_image)
                if success:
                    self._log(f"彩色图像已成功保存: {color_path}")
                    log_lines.append(f"Color Image: {os.path.basename(color_path)} - Saved Successfully")
                else:
                    self._log(f"彩色图像保存失败: {color_path}", level='error')
                    log_lines.append(f"Color Image: {os.path.basename(color_path)} - Save Failed")
            else:
                self._log("未收到彩色图像，跳过保存", level='warn')
                log_lines.append("Color Image: Not Received")
            
            # 保存深度图像和原始数据
            if self.b_depth_received:
                log_lines.append(f"Depth Image Shape: {self.depth_image.shape}")
                
                # 保存深度图像的可视化版本
                depth_normalized = cv2.normalize(self.depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
                
                depth_jpg_path = os.path.join(save_dir, "depth_image.jpg")
                success = cv2.imwrite(depth_jpg_path, depth_colored)
                if success:
                    self._log(f"深度图像可视化版本已成功保存: {depth_jpg_path}")
                    log_lines.append(f"Depth Visualization: {os.path.basename(depth_jpg_path)} - Saved Successfully")
                else:
                    self._log(f"深度图像可视化版本保存失败: {depth_jpg_path}", level='error')
                    log_lines.append(f"Depth Visualization: {os.path.basename(depth_jpg_path)} - Save Failed")
                
                # 保存深度图像的原始数据为.npy文件
                depth_npy_path = os.path.join(save_dir, "depth_image.npy")
                try:
                    np.save(depth_npy_path, self.depth_image)
                    self._log(f"深度图像原始数据已成功保存: {depth_npy_path}")
                    log_lines.append(f"Raw Depth Data: {os.path.basename(depth_npy_path)} - Saved Successfully")
                except Exception as e:
                    error_msg = f"深度图像原始数据保存失败: {str(e)}"
                    self._log(error_msg, level='error')
                    log_lines.append(f"Raw Depth Data: Save Failed - {str(e)}")
            else:
                self._log("未收到深度图像，跳过保存", level='warn')
                log_lines.append("Depth Image: Not Received")
            
            # 写入日志文件
            try:
                with open(log_file_path, 'w') as log_file:
                    log_file.write('\n'.join(log_lines))
                self._log(f"捕获日志已保存: {log_file_path}")
            except Exception as e:
                self._log(f"写入日志文件失败: {str(e)}", level='error')
                
            return True
        except Exception as e:
            self._log(f"保存图像过程中发生错误: {str(e)}", level='error')
            return False
    
    def shutdown(self):
        """关闭ROS节点"""
        self._log("图像处理器节点已关闭")
        rospy.signal_shutdown("正常关闭")

# 提供简单的使用示例
if __name__ == '__main__':
    try:
        # 创建ROSRGBD实例
        rgbd_processor = ROSRGBD(log_enabled=True)
        
        # 获取RGBD数据
        color_img, depth_img, status = rgbd_processor.getRGBD()
        
        # 保存图像（可选）
        rgbd_processor.save_images()
        
        # 显示等待用户按键
        print("按任意键退出...")
        cv2.waitKey(0)
        
        # 关闭
        rgbd_processor.shutdown()
        
    except KeyboardInterrupt:
        print("用户中断")
    except Exception as e:
        print(f"发生错误: {str(e)}")
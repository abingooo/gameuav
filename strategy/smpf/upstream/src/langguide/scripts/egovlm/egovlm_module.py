#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import json
import os
import time
import matplotlib.pyplot as plt
from lib.img_utils import ImageUtils
from lib.vls import *
from lib.cloud_utils import PointCloudUtils

# 尝试导入rospy，支持ROS环境
use_ros = False
try:
    import rospy
    use_ros = True
except ImportError:
    pass

def log_message(message,debug=True):
    """统一日志输出函数，支持ROS和非ROS环境"""
    if debug == False:
        return None
    if use_ros:
        rospy.loginfo(message)
    else:
        print(message)

def navigate_pipline(vlm, llm, ctrl_cmd, rgb_image, depth_data, log_dir, debug=True):
    """执行检测、3D建模和路径规划的完整流水线，返回阶段性结果"""
    detect_prompt = getPrompt(ctrl_cmd)
    
    # 第一阶段：vlm根据自然语言控制指令进行目标检测
    start_time = time.time()
    log_message(f"{vlm.model_id}开始VLM检测",debug)
    detect_result = getDetectBBox(vlm, rgb_image, detect_prompt)
    end_time = time.time()
    log_message(f"{vlm.model_id} VLM检测耗时: {round(end_time - start_time, 1)}",debug)
    log_message(f"{vlm.model_id} VLM检测结果: {detect_result}",debug)
    
    # 使用ImageUtils绘制检测结果并保存
    try:
        # 确保rgb_image是numpy数组格式用于绘制
        if not isinstance(rgb_image, np.ndarray):
            log_message(f"警告: 尝试在非numpy数组格式的图像上绘制边界框",debug)
            rgb_image = np.zeros((480, 640, 3), dtype=np.uint8)
            
        image_with_boxes = ImageUtils.draw_bounding_boxes(rgb_image, detect_result)
        detection_result_path = os.path.join(log_dir, 'vlm_detection_result.jpg')
        save_success = ImageUtils.save_image(image_with_boxes, detection_result_path)
        if save_success:
            log_message(f"{vlm.model_id} VLM检测可视化已保存: {detection_result_path}",debug)
        else:
            log_message(f"{vlm.model_id} VLM检测可视化存储失败",debug)
    except Exception as e:
        log_message(f"{vlm.model_id} 绘制和保存检测结果时出错: {e}",debug)
    
    # 第二阶段：lsam服务器进行目标分割，进行目标3D建模
    log_message(f"{vlm.model_id} 开始3D目标建模...",debug)
    target3dmodel = get3DTargetModel(rgb_image, detect_result, depth_data, safe_distance=0.3)
    
    # 确保target3dmodel不为空且有有效数据
    if not target3dmodel or len(target3dmodel) == 0:
        raise ValueError(f"警告: 3D目标建模结果为空")
    else:
        # 确保center不为空列表
        for target in target3dmodel:
            if 'center' not in target or not target['center']:
               raise ValueError(f"警告: 目标{target.get('label', 'unknown')}的center为空")
                
    
    pcu = PointCloudUtils()
    point_cloud_path = os.path.join(log_dir, 'point_cloud_sphere.ply')
    try:
        pcu.process_point_cloud(
            depth_data=depth_data,
            rgb_image=rgb_image,
            annotation_data=target3dmodel,
            extend_distance=0.3,
            modeling_type="sphere",
            output_ply_path=point_cloud_path,
            show_visualization=False
        )
    except Exception as e:
        log_message(f"处理点云时出错: {str(e)}",debug)
        # 创建一个空的点云文件作为替代
        import open3d as o3d
        empty_pcd = o3d.geometry.PointCloud()
        o3d.io.write_point_cloud(point_cloud_path, empty_pcd)
    log_message(f"点云文件已生成: {point_cloud_path}",debug)
    
    # 第三阶段：llm根据目标3d模型进行规划
    log_message("开始路径规划...",debug)
    plan_prompt = getPrompt(ctrl_cmd, task="plan", objects_json=target3dmodel)
    start_time = time.time()
    log_message(f"{llm.model_id}开始LLM规划",debug)
    log_message(f'输入自然语言控制指令: {ctrl_cmd}',debug)
    log_message(f'输入的场景json: {str(target3dmodel)}',debug)
    
    path_points = getPlan(llm, plan_prompt)
    end_time = time.time()
    log_message(f"LLM规划耗时: {round(end_time - start_time, 1)}",debug)
    log_message(f"规划路径点: {path_points}",debug)
    
    # 可视化路径
    path_ply_path = os.path.join(log_dir, 'point_cloud_sphere_path.ply')
    
    # 检查点云文件是否存在
    if os.path.exists(point_cloud_path):
        log_message(f"加载点云文件: {point_cloud_path}",debug)
        pcu.process_point_cloud(
            input_ply_path=point_cloud_path,
            annotation_data=path_points,
            modeling_type="path",
            output_ply_path=path_ply_path,
            show_visualization=False,
            radius=0.1
        )
        log_message(f"路径可视化已保存: {path_ply_path}",debug)   
    else:
        log_message(f"警告: 点云文件不存在: {point_cloud_path}",debug)
        # 即使点云文件不存在，我们仍然可以返回路径点
    
    return {
        'detect_result': detect_result,
        'target3dmodel': target3dmodel,
        'object': [{'label': target['label'], 'center': target['center']} for target in target3dmodel],
        'path_points': path_points
    }

def navigate_process(ctrl_cmd, rgb_image=None, depth_data=None,debug=True):
    """
    navigate_process
    
    Args:
        ctrl_cmd: 自然语言控制指令
        rgb_image: RGB图像数据（可选，如果为None则调用getCameraData获取）
        depth_data: 深度数据（可选，如果为None则调用getCameraData获取）
    
    Returns:
        dict: 包含处理结果的字典
    """
    result = {
        'success': False,
        'message': '',
        'detect_result': None,
        'target3dmodel': None,
        'path_points': None
    }
    
    try:
        log_message(f"开始处理控制指令: {ctrl_cmd}",debug)
        
        # 获取当前文件所在目录，使用绝对路径
        current_dir = os.path.dirname(os.path.abspath(__file__))
        log_dir = os.path.join(current_dir, 'log')
        
        # 确保log目录存在
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
            log_message(f"创建log目录: {log_dir}",debug)
        
        # VLM和LLM初始化
        vlm, llm = initModel()
        
        # 获取RGBD数据
        if rgb_image is None or depth_data is None:
            if debug == True:
                log_message("获取相机数据...",debug)
            rgb_image, depth_data = getCameraData()
        
        # 处理ROS Image消息对象转换为numpy数组
        try:
            # 检查是否为ROS Image消息对象（具有header、width、height等属性）
            if hasattr(rgb_image, 'header') and hasattr(rgb_image, 'width') and hasattr(rgb_image, 'height'):
                log_message("检测到ROS Image消息对象，尝试转换为numpy数组",debug)
                # 尝试导入cv_bridge进行转换
                try:
                    from cv_bridge import CvBridge
                    bridge = CvBridge()
                    rgb_image = bridge.imgmsg_to_cv2(rgb_image, "bgr8")
                    log_message("成功将ROS Image消息转换为numpy数组",debug)
                except ImportError:
                    log_message("cv_bridge未找到，尝试手动解码",debug)
                    # 手动解码逻辑
                    import numpy as np
                    if rgb_image.encoding == 'bgr8':
                        rgb_image = np.frombuffer(rgb_image.data, dtype=np.uint8).reshape(rgb_image.height, rgb_image.width, 3)
                    elif rgb_image.encoding == 'rgb8':
                        rgb_data = np.frombuffer(rgb_image.data, dtype=np.uint8).reshape(rgb_image.height, rgb_image.width, 3)
                        rgb_image = cv2.cvtColor(rgb_data, cv2.COLOR_RGB2BGR)
                    else:
                        log_message(f"不支持的图像编码格式: {rgb_image.encoding}",debug)
                        return result
        except Exception as e:
            log_message(f"转换图像时出错: {e}",debug)
            # 继续执行，让save_image尝试处理原始输入
        
        # 使用ImageUtils保存RGB图像
        rgb_image_path = os.path.join(log_dir, 'rgb_image.jpg')
        save_success = ImageUtils.save_image(rgb_image, rgb_image_path)
        if save_success:
            log_message(f"初始图像已保存: {rgb_image_path}",debug)
        else:
            log_message("初始图像存储失败",debug)

        pipeline_result = navigate_pipline(
                    vlm=vlm,
                    llm=llm,
                    ctrl_cmd=ctrl_cmd,
                    rgb_image=rgb_image,
                    depth_data=depth_data,
                    log_dir=log_dir,
                    debug=debug
                )
        # 更新结果
        result.update(pipeline_result)
        result['success'] = True
        result['message'] = "处理成功"
        
    except Exception as e:
        error_msg = f"处理失败: {str(e)}"
        log_message(error_msg,debug)
        result['message'] = error_msg
    
    return result

# 如果直接运行此模块，提供简单的测试功能
if __name__ == '__main__':
    # 读取commend文件作为测试输入
    if os.path.exists('commend'):
        with open('commend', 'r') as f:
            test_cmd = f.read().strip()
    else:
        test_cmd = "Fly to the back of the tree"
    
    # 运行处理函数
    result = process_command(test_cmd)
    print(f"测试结果: {json.dumps(result, indent=2)}")

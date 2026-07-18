import cv2
import numpy as np
import json
import os
import time
import matplotlib.pyplot as plt
from lib.img_utils import ImageUtils
from lib.vls import *
from lib.cloud_utils import PointCloudUtils

if __name__ == '__main__':
    # VLM和LLM初始化
    vlm, llm = initModel()
    # 设定控制命令
    ctrl_cmd = getCtrlCmd()
    # 获取RGBD数据
    rgb_image, depth_data = getCameraData()
    
    # 使用ImageUtils保存RGB图像
    _ = ImageUtils.save_image(rgb_image, os.path.join('log', 'rgb_image.jpg'))
    if _:
        print("初始图像：", os.path.join('log', 'rgb_image.jpg'))
    else:
        print("初始图像存储失败")

    # 获得检测提示词和规划提示词
    detect_prompt = getPrompt(ctrl_cmd)
    # 第一阶段：vlm根据自然语言控制指令进行目标检测
    start_time = time.time()
    print(vlm.model_id+"开始VLM检测")
    detect_result = getDetectBBox(vlm, rgb_image, detect_prompt)
    end_time = time.time()
    print("VLM检测耗时:", round(end_time - start_time, 1))

    print("VLM检测结果:", detect_result)
    # 使用ImageUtils绘制检测结果并保存
    image_with_boxes = ImageUtils.draw_bounding_boxes(rgb_image, detect_result)
    _ = ImageUtils.save_image(image_with_boxes, os.path.join('log', 'vlm_detection_result.jpg'))
    if _:
        print("VLM检测可视化：", os.path.join('log', 'vlm_detection_result.jpg'))
    else:
        print("VLM检测可视化存储失败")
    
    # 第二阶段：lsam服务器进行目标分割，进行目标3D建模
    target3dmodel = get3DTargetModel(rgb_image, detect_result, depth_data, safe_distance=0.3)
    pcu = PointCloudUtils()
    pcu.process_point_cloud( depth_data=depth_data, 
                            rgb_image=rgb_image,
                            annotation_data=target3dmodel[0],
                            extend_distance=0.3, 
                            modeling_type="sphere", 
                            output_ply_path="./log/point_cloud_sphere.ply", 
                            show_visualization=False
                            )
    # 第三阶段：llm根据目标3d模型进行规划 
    plan_prompt = getPrompt(ctrl_cmd, task="plan", objects_json=target3dmodel)
    start_time = time.time()
    print(llm.model_id+"开始LLM规划")
    print('输入自然语言控制指令:'+ctrl_cmd)
    print('输入的场景json:'+str(target3dmodel))
    path_points = getPlan(llm, plan_prompt)
    end_time = time.time()
    print("LLM规划耗时:", round(end_time - start_time, 1))
    print(path_points)
    pcu = PointCloudUtils()
    pcu.process_point_cloud(
                            input_ply_path="./log/point_cloud_sphere.ply",
                            annotation_data=path_points, 
                            modeling_type="path", 
                            output_ply_path="./log/point_cloud_sphere_path.ply", 
                            show_visualization=True, 
                            radius=0.1
                            )
# vls.py - Vision Language System 封装文件
# 封装VLM和LLM模型的初始化和使用函数

import cv2
import json
import numpy as np
from lib.mlm import VLM, LLM
from lib.cloud_utils import PointCloudUtils
from lib.prompt_utils import format_prompt

# 在文件顶部添加导入语句
from lib.solve import Target3DProcessor

from lib.log_utils import create_reader
from lib.lsamclient import getLSamResult
from lib.img_utils import ImageUtils
# 环境配置
REALDEVICE_ENV = False  # 是否为真机环境
if REALDEVICE_ENV:
    from lib.rosrgbd import ROSRGBD


def initModel():
    """
    初始化VLM和LLM模型
    
    Returns:
        tuple: (vlm, llm) - VLM和LLM模型实例
    """
    vlm = VLM()
    llm = LLM('gpt-5.2-chat-latest',base_url='https://api.zhizengzeng.com/v1')
    # llm = LLM()
    return vlm, llm


def getCtrlCmd():
    """
    获取控制命令
    
    Returns:
        str: 控制命令字符串
    """
    try:
        with open('commend', 'r', encoding='utf-8') as f:
            first_line = f.readline().strip()
            print(f"控制命令：{first_line}")
            return first_line
    except Exception as e:
        print(f"读取commend.txt失败: {e}")
        return ""


def getCameraData():
    """
    获取RGBD相机数据
    
    Returns:
        tuple: (rgb_image, depth_data) - RGB图像和深度数据
    """
    if REALDEVICE_ENV:
        # 真机环境下，使用ROSRGBD获取数据
        rgbd = ROSRGBD(log_level='error')
        color_image, depth_image, status = rgbd.getRGBD()
        rgbd.shutdown()
        return color_image, depth_image
    else:
        reader = create_reader('./oldlog')
        # 读取最新的日志数据
        latest_data = reader.read_log_data('20251103_104756')
        # 获取RGB图像
        rgb_image = latest_data['color_image']
        # 获取深度数据
        dep_data = latest_data['depth_data']

        return rgb_image, dep_data


def getPrompt(instruction, task="detect", objects_json=""):
    """
    根据控制命令生成检测提示词和规划提示词
    
    Args:
        instruction: 控制命令字符串
        task: 任务类型，默认"detect"
        objects_json: 场景中对象的JSON字符串
    
    Returns:
        str: 提示词字符串
    """
    if task == "tasktype":
        # 指令任务分类提示词模版
        prompt = format_prompt(
            "tasktype_prompt.txt",
            instruction=instruction
        )
        return prompt

    elif task == "detect":
        # 加载检测提示词模板
        prompt = format_prompt(
            "detect_prompt.txt",
            instruction=instruction
        )
        return prompt
    
    elif task == "plan" or task == "plan1":
        # 加载规划提示词模板（v1版本）
        prompt = format_prompt(
            "plan_promptv1.txt",
            instruction=instruction,
            objects_json=objects_json
        )
        return prompt

    elif task == "plan2":
        # 加载规划提示词模板（v2版本）
        prompt = format_prompt(
            "plan_promptv2.txt",
            instruction=instruction,
            objects_json=objects_json
        )
        return prompt
    else:
        raise ValueError("Invalid task. Supported tasks are 'detect', 'plan', 'plan1' and 'plan2'.")

def getDetectBBox(vlm, rgb_image, detect_prompt):
    """
    使用VLM根据自然语言控制指令进行目标检测，并将归一化坐标转换为实际像素坐标
    
    Args:
        vlm: VLM模型实例
        rgb_image: RGB格式的图像数据
        detect_prompt: 检测提示词
    
    Returns:
        list: 检测结果列表，包含实际像素坐标
    """
    # 调用VLM进行图像分析，获取归一化坐标的检测结果
    response_text = vlm.generate_content(rgb_image, prompt=detect_prompt)
    try:
            normalized_result = vlm.parse_json_response(response_text)
    except json.JSONDecodeError:
        print("VLM未检测到目标对象")
        exit(0) 
    
    # 获取图像形状
    image_shape = rgb_image.shape[:2]  # (height, width)
    
    # 使用反归一化函数将坐标转换为实际像素坐标
    pixel_coordinates_result = vlm.denormalize_coordinates(
        normalized_result, 
        image_shape,
        verbose=False
    )
    
    return pixel_coordinates_result

def get3DTargetModel(rgb_image, detect_result, depth_data, safe_distance=0.3):
    """
    从RGB图像中剪切检测结果对应的区域，并构建指定格式的字典
    
    Args:
        rgb_image: RGB格式的图像数据
        detect_result: 检测结果列表，包含边界框信息
        depth_data: 深度图像数据
    
    Returns:
        list: 包含剪切区域信息的字典列表
    """
    
    processor = Target3DProcessor()
    return  processor.process_targets(rgb_image, detect_result, depth_data, safe_distance)



def getPlan(llm, plan_prompt):
    """
    使用LLM根据目标3D模型进行规划
    
    Args:
        llm: LLM模型实例
        target3dmodel: 目标3D模型字典
        plan_prompt: 规划提示词
    
    Returns:
        list: 规划结果列表，包含实际像素坐标
    """
    # 调用LLM进行规划
    response_text = llm.generate_text(
        prompt=plan_prompt
    )
    # print(response_text)
    return llm.parse_json_response(response_text)["guidepoints"]



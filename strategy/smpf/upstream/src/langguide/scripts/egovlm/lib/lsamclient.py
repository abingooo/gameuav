import requests
import json
import base64
from PIL import Image
import io
import os

def getLSamResult(image_input, text_prompt, server_ip="127.0.0.1", server_port=5002):
    """
    调用服务器进行图像分割和分析
    
    参数:
    - image_input: 图像文件路径(字符串)或图像数据(NumPy数组)
    - text_prompt: 文本提示
    - server_ip: 服务器IP地址
    - server_port: 服务器端口
    
    返回:
    - 包含分割结果的JSON字典
    """
    try:
        # 根据输入类型处理图像数据
        if isinstance(image_input, str):
            # 输入是文件路径
            if not os.path.exists(image_input):
                raise FileNotFoundError(f"图像文件不存在: {image_input}")
            
            # 读取图像并转换为Base64
            with open(image_input, "rb") as img_file:
                img_base64 = base64.b64encode(img_file.read()).decode('utf-8')
        else:
            # 检查是否为NumPy数组或类似图像数据结构
            import numpy as np
            if hasattr(image_input, 'shape') and hasattr(image_input, 'dtype'):
                # 输入是NumPy数组（图像数据）
                import cv2
                # 确保图像格式正确（BGR）
                if len(image_input.shape) == 3 and image_input.shape[2] == 3:
                    # 如果是RGB格式，转换为BGR
                    if image_input.dtype == np.uint8:
                        # 已经是uint8格式，直接处理
                        image_bgr = image_input.copy()
                        if not np.array_equal(image_bgr[:,:,0], image_bgr[:,:,1]) or not np.array_equal(image_bgr[:,:,1], image_bgr[:,:,2]):
                            # 不是灰度图, 直接赋值过去
                            pass
                            
                else:
                    # 如果图像格式不正确，进行转换
                    image_bgr = np.array(image_input, dtype=np.uint8)
                    if len(image_bgr.shape) == 2:
                        image_bgr = cv2.cvtColor(image_bgr, cv2.COLOR_GRAY2BGR)
                    elif len(image_bgr.shape) != 3 or image_bgr.shape[2] != 3:
                        raise ValueError(f"不支持的图像形状: {image_input.shape}")
                
                # 将图像编码为JPEG格式
                success, img_encoded = cv2.imencode('.jpg', image_bgr)
                if not success:
                    raise Exception("图像编码失败")
                
                # 转换为Base64
                img_base64 = base64.b64encode(img_encoded).decode('utf-8')
            else:
                raise TypeError(f"不支持的图像输入类型: {type(image_input).__name__}. 请提供文件路径或NumPy数组")
        
        # 构建请求URL
        url = f"http://{server_ip}:{server_port}/predict"
        
        # 构建请求数据
        data = {
            "image": img_base64,
            "text": text_prompt
        }
        
        # 发送请求
        # print(f"正在向LSAM服务器发送请求: {url}")
        response = requests.post(url, json=data)
        
        # 检查响应状态
        if response.status_code == 200:
            return response.json()
        else:
            print(f"服务器返回错误状态码: {response.status_code}")
            try:
                return response.json()  # 尝试返回错误信息
            except:
                return {"error": response.text}
                
    except Exception as e:
        print(f"请求过程中发生错误: {str(e)}")
        print(f"错误详情: {str(e)}")
        print("VLM检测错误，LSAM无法分割")
        exit(0)

def visualize_result_local(image_path, result_json, save_path=None):
    """
    在本地进行结果可视化
    
    参数:
    - image_path: 原始图像文件路径
    - result_json: 分割结果的JSON数据
    - save_path: 保存可视化结果的路径，如果为None则不保存
    
    返回:
    - 可视化后的PIL图像对象
    """
    try:
        # 检查图像文件是否存在
        if not os.path.exists(image_path):
            raise FileNotFoundError(f"图像文件不存在: {image_path}")
        
        # 加载图像
        image_pil = Image.open(image_path).convert("RGB")
        
        # 进行可视化
        if result_json["mask_count"] > 0:
            import cv2
            import numpy as np
            
            image_array = np.array(image_pil)
            
            # 定义颜色 (BGR格式)
            centroid_color = (0, 0, 255)  # 红色 - 质心
            box_color = (0, 255, 0)       # 绿色 - 检测框
            random_color = (255, 165, 0)  # 橙色 - 随机点
            
            # 创建可视化图像
            visualized = image_array.copy()
            
            # 遍历每个掩码并绘制
            for mask_data in result_json["masks"]:
                # 绘制边界框
                bbox = mask_data["bounding_box"]
                cv2.rectangle(visualized, 
                            (bbox["x1"], bbox["y1"]), 
                            (bbox["x2"], bbox["y2"]), 
                            box_color, 2)
                
                # 绘制质心
                centroid = mask_data["centroid"]
                cv2.circle(visualized, (centroid[0], centroid[1]), 5, centroid_color, -1)
                cv2.putText(visualized, 'Centroid', (centroid[0] + 10, centroid[1] - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, centroid_color, 1)
                
                # 绘制随机点
                for idx, point in enumerate(mask_data["random_points"]):
                    cv2.circle(visualized, (point[0], point[1]), 4, random_color, -1)
                    cv2.putText(visualized, f'P{idx+1}', (point[0] + 8, point[1] + 8), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, random_color, 1)
            
            # 转换回PIL图像
            output_image = Image.fromarray(np.uint8(visualized)).convert("RGB")
        else:
            # 如果没有检测结果，返回原图
            output_image = image_pil
            print("未检测到对象，返回原图")
        
        # 保存结果
        if save_path:
            output_image.save(save_path)
            # print(f"可视化结果已保存到: {save_path}")
        
        return output_image
        
    except Exception as e:
        print(f"本地可视化过程中发生错误: {str(e)}")
        return None

def visualize_result(image_path, result_json, server_ip="127.0.0.1", server_port=5000, save_path=None, use_local=True):
    """
    结果可视化（默认使用本地可视化）
    
    参数:
    - image_path: 原始图像文件路径
    - result_json: 分割结果的JSON数据
    - server_ip: 服务器IP地址（使用服务器可视化时需要）
    - server_port: 服务器端口（使用服务器可视化时需要）
    - save_path: 保存可视化结果的路径，如果为None则不保存
    - use_local: 是否使用本地可视化，默认为True
    
    返回:
    - 可视化后的PIL图像对象
    """
    if use_local:
        # 使用本地可视化
        print("使用本地可视化...")
        return visualize_result_local(image_path, result_json, save_path)
    else:
        # 使用服务器可视化（保留原有功能）
        try:
            # 检查图像文件是否存在
            if not os.path.exists(image_path):
                raise FileNotFoundError(f"图像文件不存在: {image_path}")
            
            # 读取图像并转换为Base64
            with open(image_path, "rb") as img_file:
                img_base64 = base64.b64encode(img_file.read()).decode('utf-8')
            
            # 构建请求URL
            url = f"http://{server_ip}:{server_port}/visualize"
            
            # 构建请求数据
            data = {
                "image": img_base64,
                "result_json": result_json
            }
            
            # 发送请求
            print(f"正在向服务器发送可视化请求: {url}")
            response = requests.post(url, json=data)
            
            # 检查响应状态
            if response.status_code == 200:
                response_data = response.json()
                
                # 解码Base64图像数据
                img_data = base64.b64decode(response_data["visualized_image"])
                image = Image.open(io.BytesIO(img_data))
                
                # 保存结果
                if save_path:
                    image.save(save_path)
                    print(f"可视化结果已保存到: {save_path}")
                
                # 如果有消息，打印出来
                if "message" in response_data:
                    print(f"消息: {response_data['message']}")
                
                return image
            else:
                print(f"服务器返回错误状态码: {response.status_code}")
                try:
                    error_data = response.json()
                    print(f"错误信息: {error_data.get('error', '未知错误')}")
                except:
                    print(f"错误信息: {response.text}")
                return None
                
        except Exception as e:
            print(f"服务器可视化过程中发生错误: {str(e)}")
            return None

# 示例用法
if __name__ == "__main__":
    # 设置服务器地址和端口
    SERVER_IP = "172.16.1.61"
    SERVER_PORT = 5002
    
    # 设置图像路径和文本提示
    image_path = "/home/uav/DataDisk/PAB/egovlm/log/rgb_image.jpg"  # 替换为实际图像路径
    text_prompt = "tree"  # 替换为实际文本提示
    
    # 执行分割
    print(f"\n正在分割图像: {image_path}")
    print(f"文本提示: '{text_prompt}'")
    
    result = getLSamResult(image_path, text_prompt, server_ip=SERVER_IP, server_port=SERVER_PORT)
    
    if "error" not in result or "mask_count" in result:
        # 打印结果
        print("\n分割结果:")
        print(f"检测到的掩码数量: {result.get('mask_count', 0)}")
        
        if result.get('mask_count', 0) > 0:
            print("\n掩码详情:")
            for mask in result['masks']:
                print(f"掩码ID: {mask['mask_id']}")
                print(f"  面积: {mask['area']}")
                print(f"  质心: {mask['centroid']}")
                print(f"  边界框: {mask['bounding_box']}")
                print(f"  随机点数量: {len(mask['random_points'])}")
        
        # 可视化结果
        print("\n正在生成可视化结果...")
        output_image_path = "./visualized_result.png"
        image = visualize_result(image_path, result, SERVER_IP, SERVER_PORT, output_image_path)
        
        if image:
            print(f"可视化结果已生成并保存到 {output_image_path}")
            # 如果需要显示图像，可以取消下面的注释
            # image.show()
    else:
        print(f"分割失败: {result.get('error', '未知错误')}")
import cv2
import json
import time
import base64
import requests


class LLM:
    """
    语言模型类，用于文本生成和理解
    使用POST请求方式调用API
    """
    
    def __init__(self, model_id="gpt-5.2", api_key="sk-zk26f90a8ef46c6589207af1a58b11c4e4a68eca448256d6",
                 base_url="https://api.zhizengzeng.com/v1/chat/completions"):
        """
        初始化语言模型
        
        Args:
            model_id: 使用的模型ID
            api_key: API密钥
            base_url: API基础URL
        """
        self.model_id = model_id
        self.api_key = api_key
        self.base_url = base_url
    
    def generate_text(self, prompt, temperature=0.5, verbose=False):
        """
        生成文本响应
        
        Args:
            prompt: 输入提示文本
            temperature: 生成内容的温度参数
            verbose: 是否打印详细信息
            
        Returns:
            生成的文本
        """
        if verbose:
            print("开始文本生成")
            start_time = time.time()
        
        try:
            # 构建请求头
            headers = {
                "Content-Type": "application/json",
                "Authorization": f"Bearer {self.api_key}"
            }
            
            # 构建请求数据
            data = {
                "model": self.model_id,
                "messages": [
                    {
                    "role": "system",
                    "content": "You are a helpful assistant. Before answering, think step by step with deep reasoning, then provide the final concise answer."
                    },
                    {

                        "role": "user",
                        "content": prompt
                    }
                ],
                "temperature": temperature
            }
            
            # 确保URL完整
            url = self.base_url
            if not url.endswith('/chat/completions') and '/v1' in url:
                url = url.rstrip('/') + '/chat/completions'
            
            # 发送POST请求
            response = requests.post(
                url,
                headers=headers,
                data=json.dumps(data)
            )
            
            # 检查响应状态
            if response.status_code == 200:
                response_data = response.json()
                # 解析响应获取内容
                if 'choices' in response_data and len(response_data['choices']) > 0:
                    result = response_data['choices'][0]['message']['content']
                else:
                    raise ValueError("响应格式不正确: 未找到choices字段")
            else:
                raise Exception(f"API请求失败: {response.status_code} - {response.text}")
                
        except Exception as e:
            if verbose:
                print(f"请求异常: {str(e)}")
            raise
        
        if verbose:
            print(f"生成时间: {time.time() - start_time:.2f} 秒")
        
        return result

    def parse_json_response(self, response_text, verbose=False):
        """
        解析JSON格式的响应文本
        
        Args:
            response_text: 模型返回的文本
            verbose: 是否打印解析结果
            
        Returns:
            解析后的JSON数据
        """
        # 清理文本，移除可能的代码块标记
        cleaned_text = response_text.strip()
        if cleaned_text.startswith('```json'):
            cleaned_text = cleaned_text[7:]
        if cleaned_text.endswith('```'):
            cleaned_text = cleaned_text[:-3]
        cleaned_text = cleaned_text.strip()
        # 解析JSON
        result = json.loads(cleaned_text)
        
        if verbose:
            print(result)
        
        return result


class VLM:
    """
    视觉语言模型类，用于图像处理和理解
    使用POST请求方式调用API
    """
    
    def __init__(self, model_id="gemini-2.5-flash", api_key="sk-zk26f90a8ef46c6589207af1a58b11c4e4a68eca448256d6",
                 base_url="https://api.zhizengzeng.com/google/v1beta/models/"):
        """
        初始化视觉语言模型
        
        Args:
            model_id: 使用的模型ID
            api_key: API密钥
            base_url: API基础URL
        """
        self.model_id = model_id
        self.api_key = api_key
        self.base_url = f"{base_url}{model_id}:generateContent?key={api_key}"
    
    def prepare_image(self, rgb_image, jpeg_quality=60):
        """
        准备图像数据，将RGB图像编码为JPEG格式并转换为base64
        
        Args:
            rgb_image: RGB格式的图像数据
            jpeg_quality: JPEG编码质量
            
        Returns:
            base64编码的图像字符串
        """
        # 确保图像是BGR格式（OpenCV的标准格式）
        if len(rgb_image.shape) == 3 and rgb_image.shape[2] == 3:
            # 如果是RGB格式，转换为BGR
            bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        else:
            bgr_image = rgb_image
        
        _, img_encoded = cv2.imencode('.jpg', bgr_image, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
        img_bytes = img_encoded.tobytes()
        # 转换为base64编码
        return base64.b64encode(img_bytes).decode('utf-8')
    
    def analyze_image(self, rgb_image, prompt, temperature=0.5, jpeg_quality=60, verbose=False):
        """
        分析图像并根据提示生成响应
        
        Args:
            rgb_image: RGB格式的图像数据
            prompt: 分析提示文本
            temperature: 生成内容的温度参数
            jpeg_quality: JPEG编码质量
            verbose: 是否打印详细信息
            
        Returns:
            模型生成的响应文本
        """
        if verbose:
            print("开始图像分析")
            start_time = time.time()
        
        try:
            # 准备图像数据（base64编码）
            base64_image = self.prepare_image(rgb_image, jpeg_quality)
            
            # 构建请求头
            headers = {
                'Content-Type': 'application/json'
            }
            
            # 构建请求数据
            data = {
                "contents": [{
                    "parts": [
                        {
                            "inline_data": {
                                "mime_type": "image/jpeg",
                                "data": base64_image
                            }
                        },
                        {"text": prompt},
                    ]
                }],
                "generationConfig": {
                    "temperature": temperature
                }
            }
            
            # 发送POST请求
            response = requests.post(
                self.base_url,
                headers=headers,
                data=json.dumps(data)
            )
            
            # 检查响应状态
            if response.status_code == 200:
                response_data = response.json()
                # 解析响应获取内容
                if 'candidates' in response_data and len(response_data['candidates']) > 0:
                    if 'content' in response_data['candidates'][0] and \
                       'parts' in response_data['candidates'][0]['content'] and \
                       len(response_data['candidates'][0]['content']['parts']) > 0:
                        result = response_data['candidates'][0]['content']['parts'][0].get('text', '')
                    else:
                        raise ValueError("响应格式不正确: 未找到预期的content结构")
                else:
                    raise ValueError("响应格式不正确: 未找到candidates字段")
            else:
                raise Exception(f"API请求失败: {response.status_code} - {response.text}")
                
        except Exception as e:
            if verbose:
                print(f"请求异常: {str(e)}")
            raise
        
        if verbose:
            print(f"分析时间: {time.time() - start_time:.2f} 秒")
        
        return result
    
    def generate_content(self, rgb_image, prompt=None, **kwargs):
        """
        兼容vls.py中的调用方式
        
        Args:
            rgb_image: RGB格式的图像数据
            prompt: 分析提示文本
            **kwargs: 其他参数
            
        Returns:
            模型生成的响应文本
        """
        # 直接调用analyze_image方法，确保接口兼容
        return self.analyze_image(rgb_image, prompt, **kwargs)
    
    def parse_json_response(self, response_text, verbose=False):
        """
        解析JSON格式的响应文本
        
        Args:
            response_text: 模型返回的文本
            verbose: 是否打印解析结果
            
        Returns:
            解析后的JSON数据
        """
        # 清理文本，移除可能的代码块标记
        cleaned_text = response_text.strip()
        if cleaned_text.startswith('```json'):
            cleaned_text = cleaned_text[7:]
        if cleaned_text.endswith('```'):
            cleaned_text = cleaned_text[:-3]
        cleaned_text = cleaned_text.strip()
        
        try:
            # 解析JSON
            result = json.loads(cleaned_text)
            
            if verbose:
                print(result)
            
            return result
        except json.JSONDecodeError as e:
            if verbose:
                print(f"JSON解析错误: {str(e)}")
            raise ValueError(f"无法解析JSON响应: {str(e)}")
    
    def denormalize_coordinates(self, detection_result, image_shape, verbose=False):
        """
        将归一化的坐标（0-1000范围）转换为实际像素坐标
        
        Args:
            detection_result: 包含归一化坐标的检测结果（JSON格式或字典列表）
            image_shape: 图像形状 (height, width)
            verbose: 是否打印转换结果
            
        Returns:
            转换后的检测结果，包含实际像素坐标
        """
        # 确保detection_result是列表格式
        if not isinstance(detection_result, list):
            # 如果传入的是JSON字符串，先解析
            if isinstance(detection_result, str):
                # 清理文本，移除可能的代码块标记
                cleaned_text = detection_result.strip()
                if cleaned_text.startswith('```json'):
                    cleaned_text = cleaned_text[7:]
                if cleaned_text.endswith('```'):
                    cleaned_text = cleaned_text[:-3]
                cleaned_text = cleaned_text.strip()
                try:
                    detection_result = json.loads(cleaned_text)
                except json.JSONDecodeError as e:
                    raise ValueError(f"无法解析JSON字符串: {str(e)}")
            else:
                raise TypeError("detection_result必须是列表或JSON字符串格式")
        
        # 获取图像高度和宽度
        height, width = image_shape
        
        # 复制结果以避免修改原始数据
        result = []
        for detection in detection_result:
            detection_copy = detection.copy()
            
            # 处理边界框坐标
            if "box_2d" in detection_copy:
                box_2d = detection_copy["box_2d"]
                # 验证box_2d格式
                if isinstance(box_2d, list) and len(box_2d) >= 4:
                    # 取前4个值作为[ymin, xmin, ymax, xmax]
                    ymin, xmin, ymax, xmax = box_2d[:4]
                    # 转换为真实像素坐标
                    x1 = int((xmin / 1000.0) * width)
                    y1 = int((ymin / 1000.0) * height)
                    x2 = int((xmax / 1000.0) * width)
                    y2 = int((ymax / 1000.0) * height)
                    # 确保坐标在有效范围内
                    x1 = max(0, min(x1, width - 1))
                    y1 = max(0, min(y1, height - 1))
                    x2 = max(0, min(x2, width - 1))
                    y2 = max(0, min(y2, height - 1))
                    detection_copy["box_2d"] = [y1, x1, y2, x2]
                else:
                    if verbose:
                        print(f"警告: box_2d格式不正确，期望包含至少4个值，但得到: {box_2d}")
                    # 如果格式不正确，跳过此检测结果
                    continue
            
            # 处理点坐标（如果存在）
            elif "point" in detection_copy:
                # 点坐标是[y, x]格式，归一化到0-1000
                y_norm, x_norm = detection_copy["point"]
                # 转换为真实像素坐标
                x = int((x_norm / 1000.0) * width)
                y = int((y_norm / 1000.0) * height)
                # 确保坐标在有效范围内
                x = max(0, min(x, width - 1))
                y = max(0, min(y, height - 1))
                detection_copy["point"] = [y, x]
            
            result.append(detection_copy)
        
        if verbose:
            print("反归一化后的坐标结果:", result)
        
        return result

# 添加通用的错误处理函数
def handle_api_error(response):
    """
    处理API响应错误
    
    Args:
        response: requests响应对象
        
    Returns:
        None
        
    Raises:
        Exception: 如果API响应包含错误
    """
    if response.status_code != 200:
        error_msg = f"API请求失败: {response.status_code}"
        try:
            # 尝试解析错误响应
            error_data = response.json()
            if 'error' in error_data:
                error_msg += f" - {error_data['error'].get('message', '未知错误')}"
        except:
            error_msg += f" - {response.text}"
        raise Exception(error_msg)


# 测试实例代码
if __name__ == "__main__":
    import os
    
    # 测试LLM类
    def test_llm():
        print("=== 测试LLM文本生成 ===")
        try:
            # 创建LLM实例
            llm = LLM(
                model_id="gpt-4o-mini",
                api_key="sk-zk26f90a8ef46c6589207af1a58b11c4e4a68eca448256d6",
                base_url="https://api.zhizengzeng.com/v1/chat/completions"
            )
            
            # 简单文本生成测试
            prompt = "请简要介绍一下人工智能的应用领域"
            print(f"发送提示: {prompt}")
            response = llm.generate_text(prompt, temperature=0.7, verbose=True)
            print(f"生成结果: {response}")
            print("\nLLM测试完成\n")
            
        except Exception as e:
            print(f"LLM测试失败: {str(e)}")
    
    # 测试VLM类
    def test_vlm():
        print("=== 测试VLM图像分析 ===")
        try:
            # 创建VLM实例
            vlm = VLM(
                model_id="gemini-2.0-flash",
                api_key="sk-zk26f90a8ef46c6589207af1a58b11c4e4a68eca448256d6",
                base_url="https://api.zhizengzeng.com/google/v1beta/models/"
            )
            
            # 检查测试图像是否存在
            test_image_path = os.path.join("..", "log", "rgb_image.jpg")
            if not os.path.exists(test_image_path):
                # 尝试使用其他可能的图像路径
                test_image_path = os.path.join("log", "rgb_image.jpg")
            
            if os.path.exists(test_image_path):
                print(f"使用测试图像: {test_image_path}")
                # 读取测试图像
                image = cv2.imread(test_image_path)
                
                if image is not None:
                    # 转换为RGB格式（因为VLM类期望RGB格式）
                    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                    
                    # 图像描述测试
                    prompt = "详细描述这张图片中有什么"
                    print(f"发送提示: {prompt}")
                    response = vlm.analyze_image(rgb_image, prompt, temperature=0.5, verbose=True)
                    print(f"分析结果: {response}")
                    
                    # 坐标检测测试（如果需要）
                    detection_prompt = "请检测图像中的所有物体，并以JSON格式返回它们的位置信息，使用box_2d字段表示边界框，格式为[ymin, xmin, ymax, xmax]，值的范围为0-1000"
                    print(f"发送检测提示: {detection_prompt}")
                    detection_response = vlm.analyze_image(rgb_image, detection_prompt, verbose=True)
                    
                    try:
                        # 尝试解析JSON响应
                        parsed_result = vlm.parse_json_response(detection_response)
                        print(f"解析的检测结果: {parsed_result}")
                        
                        # 如果有检测结果，尝试反归一化坐标
                        if parsed_result:
                            denormalized = vlm.denormalize_coordinates(
                                parsed_result, 
                                (image.shape[0], image.shape[1]), 
                                verbose=True
                            )
                            print(f"反归一化后的坐标: {denormalized}")
                    except ValueError:
                        print("检测结果不是有效的JSON格式，可能是纯文本描述")
                        print(f"原始检测响应: {detection_response}")
                else:
                    print("无法读取测试图像")
            else:
                print("未找到测试图像，跳过图像分析测试")
                print("请确保在log目录下有rgb_image.jpg文件")
            
            print("\nVLM测试完成")
            
        except Exception as e:
            print(f"VLM测试失败: {str(e)}")
    
    # 运行测试
    print("开始测试mlm.py中的模型功能...\n")
    
    # 测试LLM
    test_llm()
    
    # 测试VLM
    test_vlm()
    
    print("\n所有测试完成！")
    print("注意: 在实际应用中，请根据需要修改API密钥和URL")

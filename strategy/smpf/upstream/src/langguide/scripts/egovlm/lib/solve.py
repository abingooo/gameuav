import numpy as np
from lib.lsamclient import getLSamResult
from lib.img_utils import ImageUtils

class TargetExtractor:
    """
    从RGB图像中提取目标区域的类
    """
    
    @staticmethod
    def extract_target_region(rgb_image, detection):
        """
        从RGB图像中提取目标区域
        
        Args:
            rgb_image: RGB格式的图像数据
            detection: 单个检测结果，包含边界框信息
            
        Returns:
            dict: 包含目标区域信息的字典
        """
        # 检查是否包含box_2d信息
        if "box_2d" not in detection:
            return None
        
        # 对边界框坐标进行外扩，增大sam分割识别视野
        detection["box_2d"][0] = int(detection["box_2d"][0] * 0.8)
        detection["box_2d"][1] = int(detection["box_2d"][1] * 0.8)
        detection["box_2d"][2] = int(detection["box_2d"][2] * 1.2)
        detection["box_2d"][3] = int(detection["box_2d"][3] * 1.2)
        
        # 获取边界框坐标 [y1, x1, y2, x2]
        y1, x1, y2, x2 = detection["box_2d"]
        
        # 确保坐标在有效范围内
        height, width = rgb_image.shape[:2]
        y1 = max(0, min(y1, height - 1))
        x1 = max(0, min(x1, width - 1))
        y2 = max(0, min(y2, height - 1))
        x2 = max(0, min(x2, width - 1))
        
        # 剪切区域（注意：OpenCV使用[y:y+h, x:x+w]格式）
        image_roi = rgb_image[y1:y2+1, x1:x2+1]
        
        # 构建字典，注意bbox格式为[[x1,y1],[x2,y2]]
        return {
            "label": detection.get("label", "unknown"),
            "bbox": [[x1, y1], [x2, y2]],
            "imageROI": image_roi
        }

class LSamProcessor:
    """
    处理LSam服务器分割结果的类
    """
    
    @staticmethod
    def process_lsam_result(target_region, index, server_ip="47.108.251.163", server_port=5000):
        """
        处理LSam服务器的分割结果
        
        Args:
            target_region: 目标区域信息字典
            index: 目标索引
            server_ip: LSam服务器IP地址
            server_port: LSam服务器端口
            
        Returns:
            dict: 转换到原始RGB图像坐标系的LSam分割结果
        """
        # 调用LSam服务器进行目标分割（支持直接传入NumPy数组）
        lsam_result = getLSamResult(
            target_region['imageROI'], 
            target_region['label'], 
            server_ip=server_ip, 
            server_port=server_port
        )

        # 只选取最大面积目标
        lsam_result['masks'] = [max(lsam_result['masks'], key=lambda x: x['area'])]
        lsam_result['mask_count'] = 1
        
        # 使用新增的可视化方法保存LSAM分割结果
        ImageUtils.visualize_lsam_result(
            target_region['imageROI'], 
            lsam_result, 
            index=index, 
            label=target_region['label'], 
            save_dir="./log/roi"
        )
        
        # 将LSAM分割结果从ROI图像坐标系转换到原始RGB图像坐标系
        return ImageUtils.convert_lsam_coordinates_to_rgb(lsam_result, target_region['bbox'])

class Target3DModeler:
    """
    构建3D目标模型的类
    """
    
    def __init__(self):
        # 设置相机参数
        self.camera_params = {
            'fx': 383.19929174573906,  # 焦距x
            'fy': 384.76715878730715,  # 焦距y
            'cx': 317.944484051631,    # 光心x
            'cy': 231.71115593384292   # 光心y
        }
    
    def calculate_3d_position(self, pixel_coord, depth, precision=2):
        """
        根据像素坐标和深度计算3D坐标
        
        Args:
            pixel_coord: 像素坐标 [x, y]
            depth: 深度值
            precision: 保留小数位数
            
        Returns:
            3D坐标 [x, y, z]
        """
        # 确保坐标是整数
        x_pixel = round(pixel_coord[0])
        y_pixel = round(pixel_coord[1])
        try:
            with open('/home/uav/lab/muav/depth.txt', 'a') as f:
                f.write(f"input:{x_pixel}, {y_pixel}, {depth}\n")
        except Exception as e:  
            with open('/home/abin/lab/muav/depth.txt', 'a') as f:
                f.write(f"input:{x_pixel}, {y_pixel}, {depth}\n")
        x = ((x_pixel - self.camera_params['cx']) * depth / self.camera_params['fx']) 
        y = ((y_pixel - self.camera_params['cy']) * depth / self.camera_params['fy'])
        z = depth
        
        return [round(float(x), precision), round(float(y), precision), round(float(z), precision)]
    
    def calculate_average_depth(self, obj_data, depth_data):
        """
        计算目标的平均深度
        
        Args:
            obj_data: 目标数据字典
            depth_data: 深度图像数据
            
        Returns:
            float: 平均深度值
        """
        avg_depth = 0
        for point in obj_data['npoints'] + [obj_data['mass_center']]:
            # 对每个像素点应用7x7窗口的中值滤波
            x, y = point
            # 计算窗口边界，确保在图像范围内
            half_win = 3  # 7x7窗口的半宽
            x_min = max(0, x - half_win)
            x_max = min(depth_data.shape[1] - 1, x + half_win)
            y_min = max(0, y - half_win)
            y_max = min(depth_data.shape[0] - 1, y + half_win)
            # 获取窗口内的深度值
            window = depth_data[y_min:y_max+1, x_min:x_max+1]
            # 计算中值深度（使用Python标准库）
            window_flat = window.flatten()
            sorted_window = sorted(window_flat)
            median_index = len(sorted_window) // 2
            median_depth = sorted_window[median_index]
            avg_depth += float(median_depth)  # 转换为Python原生float类型
        avg_depth /= 10
        return round(avg_depth, 2)  # 保留2位小数
    
    def calculate_distance(self, p1, p2):
        """
        计算两点之间的欧氏距离
        
        Args:
            p1: 点1坐标 [x1, y1, z1]
            p2: 点2坐标 [x2, y2, z2]
            
        Returns:
            float: 两点之间的距离
        """
        return round(((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + (p1[2] - p2[2])**2)**0.5, 2)

    def calculate_sphere_centers(self, corners, radius, tol=1e-8):
        """
        四个共面角点 + 半径 r -> 返回两个球心解（或 0 个）
        
        参数:
            corners: [[x,y,z], [x,y,z], [x,y,z], [x,y,z]]
            radius:  float, 球半径 r

        返回:
            centers: list，长度为 0 或 2，每个元素是 [x,y,z]
        """
        P = np.array(corners, dtype=float)
        
        # 1) 平面内中心 O（四个点的平均）
        O = P.mean(axis=0)
        
        # 2) 平面内圆半径 R0（中心到任意角点的距离）
        R0 = np.linalg.norm(P[0] - O)

        # 如果给的 r 比这个圆还小，根本包不住四个点 -> 无解
        if radius < R0 - tol:
            return []  # 或者 raise ValueError("半径太小，无法成球")

        # 3) 平面法向量 n（用两条边叉乘）
        v1 = P[1] - P[0]
        v2 = P[2] - P[0]
        n = np.cross(v1, v2)
        norm_n = np.linalg.norm(n)
        if norm_n < tol:
            # 四点几乎共线或严重退化
            return []  # 或 raise

        n = n / norm_n

        # 4) 沿法线方向的偏移距离 h
        h_sq = radius**2 - R0**2
        # 数值误差可能略微 < 0，截断到 0
        if h_sq < 0:
            if h_sq > -tol:
                h_sq = 0.0
            else:
                return []  # 几何上无解
        h = np.sqrt(h_sq)

        # 5) 两个球心
        C1 = O + h * n
        C2 = O - h * n
        # 对每个坐标值进行四舍五入，保留2位小数
        C1_rounded = [round(float(coord), 2) for coord in C1]
        C2_rounded = [round(float(coord), 2) for coord in C2]
        # print(C1_rounded, C2_rounded)
        if C1_rounded[2] > C2_rounded[2]:
            return C1_rounded
        return C2_rounded

    def create_3d_model(self, rgb_coordinates_lsam_result, depth_data, index, safe_distance=0.3):
        """
        创建3D目标模型
        
        Args:
            rgb_coordinates_lsam_result: 转换后的LSam结果
            depth_data: 深度图像数据
            index: 目标索引
            safe_distance: 安全距离
            
        Returns:
            dict: 3D目标模型字典
        """
        if rgb_coordinates_lsam_result.get('mask_count', 0) <= 0 or 'masks' not in rgb_coordinates_lsam_result:
            return None
        
        first_mask = rgb_coordinates_lsam_result['masks'][0]

        # 创建对象数据字典
        obj_data = {
            'id': index,
            'label': rgb_coordinates_lsam_result.get('text_prompt', f'object_{index}'),
            'mass_center': first_mask.get('centroid', []),
        }
        
        # 边界框（转换为简单格式）
        bbox_data = first_mask.get('bounding_box', {})
        obj_data['bbox'] = [
            [bbox_data.get('x1', 0), bbox_data.get('y1', 0)],
            [bbox_data.get('x2', 0), bbox_data.get('y2', 0)]
        ]
        
        # 9个随机点
        random_points = first_mask.get('random_points', [])
        obj_data['npoints'] = random_points
        
        # 创建3D模型
        obj3d = {
            'id': obj_data['id'],
            'label': obj_data['label']
        }
        
        # 计算平均深度
        depth = self.calculate_average_depth(obj_data, depth_data)
        # 使用通用函数计算质心3D坐标（XYZ正向：右下前）
        # mass_center = self.calculate_3d_position(obj_data['mass_center'], depth)
        
        #  ============球体建模计算=============================
        # 计算四个角点3D坐标（XYZ正向：右下前）
        corners = [
            self.calculate_3d_position(obj_data['bbox'][0], depth),  # 左上前角点
            self.calculate_3d_position([obj_data['bbox'][1][0], obj_data['bbox'][0][1]], depth),  # 右上前角点
            self.calculate_3d_position([obj_data['bbox'][0][0], obj_data['bbox'][1][1]], depth),  # 左下前角点
            self.calculate_3d_position(obj_data['bbox'][1], depth)  # 右下前角点
        ]
        # with open('/home/uav/lab/muav/depth.txt', 'w') as f:
        #     f.write(str(corners))
        # 计算四个角点所在的球体球心，半径为最大边长的一半
        max_length = max(
            self.calculate_distance(corners[0], corners[3]),
            self.calculate_distance(corners[1], corners[2])
        )*0.55
        center = self.calculate_sphere_centers(corners, max_length)
        if center == []:
            center = corners[0]
            center[0] = (corners[0][0] + corners[1][0] + corners[2][0] + corners[3][0]) / 4
            center[1] = (corners[0][1] + corners[1][1] + corners[2][1] + corners[3][1]) / 4
            center[2] = (corners[0][2] + corners[1][2] + corners[2][2] + corners[3][2]) / 4
            center = [round(float(coord), 2) for coord in center]
            
        obj3d['center'] = center
        # 向外膨胀0.3m
        obj3d['safety_radius'] = round(max_length+safe_distance, 2)
        return obj3d


class Target3DProcessor:
    """
    整合所有功能的3D目标处理器类
    """
    
    def __init__(self):
        self.target_extractor = TargetExtractor()
        self.lsam_processor = LSamProcessor()
        self.target_3d_modeler = Target3DModeler()
    
    def process_targets(self, rgb_image, detect_result, depth_data, safe_distance=0.3):
        """
        处理检测结果，构建3D目标模型
        
        Args:
            rgb_image: RGB格式的图像数据
            detect_result: 检测结果列表，包含边界框信息
            depth_data: 深度图像数据
            
        Returns:
            list: 3D目标模型列表
        """
        object_dict_for3d_list = []
        
        # 遍历所有检测结果
        visual_list = []
        for index, detection in enumerate(detect_result):
            # 提取目标区域
            target_region = self.target_extractor.extract_target_region(rgb_image, detection)
            if target_region is None:
                continue
            
            # 处理LSam结果
            rgb_coordinates_lsam_result = self.lsam_processor.process_lsam_result(
                target_region, index
            )
            visual_list.append(rgb_coordinates_lsam_result)
            # import json
            # print(json.dumps(rgb_coordinates_lsam_result, indent=2, ensure_ascii=False))
            # 构建3D模型
            obj3d = self.target_3d_modeler.create_3d_model(
                rgb_coordinates_lsam_result, depth_data, index, safe_distance
            )
            if obj3d is not None:
                object_dict_for3d_list.append(obj3d)
        # 可视化lsam结果
        ImageUtils.visualize_target2d_results(rgb_image, visual_list)

        return object_dict_for3d_list

    def reshape_cubemodel_data(self,orignal3dmodel):
        cube_data = {}
        cube_data['object_name'] = orignal3dmodel['label']
        cube_data['mass_center'] = orignal3dmodel['mass_center']
        cube_data['cube_center'] = orignal3dmodel['cube_center']
        cube_data['bbox3d'] = {"front_top_left":orignal3dmodel['bbox3dfront'][0],
                                "front_top_right":orignal3dmodel['bbox3dfront'][2],
                                "front_bottom_left":orignal3dmodel['bbox3dfront'][3],
                                "front_bottom_right":orignal3dmodel['bbox3dfront'][1],
                                "back_top_left":orignal3dmodel['bbox3dback'][0],
                                "back_top_right":orignal3dmodel['bbox3dback'][2],
                                "back_bottom_left":orignal3dmodel['bbox3dback'][3],
                                "back_bottom_right":orignal3dmodel['bbox3dback'][1]}

        return cube_data


    def reshape_spheremodel_data(self,orignal3dmodel):
        sphere_data = {}
        sphere_data['object_name'] = orignal3dmodel['label']
        sphere_data['mass_center'] = orignal3dmodel['mass_center']
        sphere_data['sphere_center'] = orignal3dmodel['sphere_center']
        sphere_data['radius'] = orignal3dmodel['radius']
        
        return sphere_data

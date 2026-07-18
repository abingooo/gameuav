import numpy as np
import math
import cv2
import os
import json
try:
    import rospy  # type: ignore
except ImportError:
    rospy = None
# 添加Open3D导入（如果环境中没有安装Open3D，可以注释掉这部分）
try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    print("警告: 未安装Open3D库，无法使用可视化功能")
    HAS_OPEN3D = False

class PointCloudUtils:
    """
    点云处理工具类，提供点云读写、处理和可视化相关功能
    """
    
    # 预定义颜色常量
    RED = [255, 0, 0]      # 红色：用于起始点的球体
    BLUE = [0, 0, 255]     # 蓝色：用于所有箭头
    GREEN = [0, 255, 0]    # 绿色：用于最后一个点
    PURPLE = [128, 0, 128] # 紫色：用于只有一个点的情况
    # 新增颜色定义
    PINK = [255, 0, 255]   # 品红色：用于立方体角点
    LIGHT_BLUE = [173, 216, 230]  # 淡蓝色：用于质心点
    ORANGE = [255, 165, 0] # 橙色：用于质量中心
    CYAN = [0, 255, 255]   # 青色：用于立方体中心
    
    def __init__(self):
        """初始化点云工具类"""
        self.model_type = None
        self.cloud_pub = None
        if rospy:
            try:
                from std_msgs.msg import String  # type: ignore
                self.cloud_pub = rospy.Publisher(
                    "langguide/plan_pointcloud",
                    String,
                    queue_size=1,
                    latch=True,
                )
            except Exception:
                self.cloud_pub = None
    
    def read_ply_file(self, ply_file_path):
        """
        读取PLY文件，提取点云数据
        
        Args:
            ply_file_path: PLY文件路径
        
        Returns:
            tuple: (points, colors) 点云坐标和颜色数据
        """
        points = []
        colors = []
        
        try:
            with open(ply_file_path, 'r') as f:
                lines = f.readlines()
            
            # 跳过头部信息，找到数据开始位置
            header_end_index = 0
            vertex_count = 0
            
            for i, line in enumerate(lines):
                line = line.strip()
                if line.startswith('element vertex'):
                    vertex_count = int(line.split()[-1])
                elif line == 'end_header':
                    header_end_index = i + 1
                    break
            
            # 读取点云数据
            for line in lines[header_end_index:header_end_index + vertex_count]:
                data = list(map(float, line.strip().split()))
                if len(data) >= 3:
                    points.append(data[:3])  # x, y, z
                    if len(data) >= 6:
                        colors.append(data[3:6])  # r, g, b
                    else:
                        colors.append([0, 0, 0])
            
            return np.array(points), np.array(colors)
        except Exception as e:
            print(f"读取PLY文件时出错: {e}")
            return np.array([]), np.array([])
    
    def add_annotation_points(self, input_ply_path, output_ply_path, annotation_points, radius=0.2, points_per_arrow=1000):
        """
        添加标注点到PLY点云文件（使用箭头显示顺序）
        
        参数:
            input_ply_path: 输入PLY文件路径
            output_ply_path: 输出PLY文件路径
            annotation_points: 标注点的3D坐标列表，格式为 [[x1, y1, z1], [x2, y2, z2], ...]
            annotation_color: 标注点颜色，默认红色
            radius: 标注点大小，默认0.2
            points_per_arrow: 每个箭头生成的点数，默认1000
        
        返回:
            bool: 是否成功添加标注点
        """
        output_ply_path = self.get_next_filename(output_ply_path)
        try:
            # 读取原始点云
            original_points, original_colors = self.read_ply_file(input_ply_path)
            if len(original_points) == 0:
                return False
            
            # 将标注点扩展为球体/箭头，使其在点云中更明显
            all_points = original_points.tolist()
            all_colors = original_colors.tolist()
            
            # 添加所有标注点，使用箭头显示顺序
            for i, point in enumerate(annotation_points):
                if len(annotation_points) == 1:
                    # 只有一个点的情况：显示紫色球体
                    sphere_points, sphere_colors = self._generate_sphere_points(
                        point, radius, points_per_arrow, self.PURPLE
                    )
                    all_points.extend(sphere_points)
                    all_colors.extend(sphere_colors)
                elif i == 0:
                    # 起始点：红色球体 + 蓝色箭头
                    # 1. 添加红色球体
                    sphere_points, sphere_colors = self._generate_sphere_points(
                        point, radius, points_per_arrow, self.RED
                    )
                    all_points.extend(sphere_points)
                    all_colors.extend(sphere_colors)
                    
                    # 2. 添加指向第二个点的蓝色箭头
                    next_point = annotation_points[i + 1]
                    arrow_points, arrow_colors = self._generate_arrow_points(
                        point, next_point, radius, points_per_arrow, self.BLUE
                    )
                    all_points.extend(arrow_points)
                    all_colors.extend(arrow_colors)
                elif i < len(annotation_points) - 1:
                    # 中间点：蓝色箭头（不显示球体）
                    next_point = annotation_points[i + 1]
                    arrow_points, arrow_colors = self._generate_arrow_points(
                        point, next_point, radius, points_per_arrow, self.BLUE
                    )
                    all_points.extend(arrow_points)
                    all_colors.extend(arrow_colors)
                else:
                    # 最后一个点：绿色球体
                    sphere_points, sphere_colors = self._generate_sphere_points(
                        point, radius, points_per_arrow, self.GREEN
                    )
                    all_points.extend(sphere_points)
                    all_colors.extend(sphere_colors)
            
            # 构建新的PLY文件
            total_points = len(all_points)
            
            # 创建PLY文件头部
            header = [
                'ply',
                'format ascii 1.0',
                f'element vertex {total_points}',
                'property float x',
                'property float y',
                'property float z',
                'property uchar red',
                'property uchar green',
                'property uchar blue',
                'end_header'
            ]
            
            # 写入新的PLY文件
            with open(output_ply_path, 'w', encoding='utf-8') as f:
                f.write('\n'.join(header) + '\n')
                
                for i in range(total_points):
                    x, y, z = all_points[i]
                    r = max(0, min(255, int(all_colors[i][0])))
                    g = max(0, min(255, int(all_colors[i][1])))
                    b = max(0, min(255, int(all_colors[i][2])))
                    f.write(f'{x:.6f} {y:.6f} {z:.6f} {r} {g} {b}\n')
            
            print(f"成功添加 {len(annotation_points)} 个标注点到 {output_ply_path}，")
            print(f"使用箭头显示顺序，箭头指向后续点")
            return True
        
        except Exception as e:
            print(f"添加标注点时出错: {e}")
            return False
    
    def save_point_cloud_to_ply(self, points_3d, colors, ply_file_path):
        """
        将点云和颜色数据保存为PLY格式文件
        
        参数:
            points_3d: 3D点云数据，格式为 [[x1, y1, z1], [x2, y2, z2], ...]
            colors: 对应点的颜色数据，格式为 [[r1, g1, b1], [r2, g2, b2], ...]
            ply_file_path: 输出PLY文件路径
        
        返回:
            bool: 是否成功保存
        """
        try:
            # 确保points_3d不为空
            if len(points_3d) == 0:
                print("错误: 没有点云数据可保存")
                return False
            
            # 创建PLY文件头部
            header = [
                'ply',
                'format ascii 1.0',
                f'element vertex {len(points_3d)}',
                'property float x',
                'property float y',
                'property float z',
                'property uchar red',
                'property uchar green',
                'property uchar blue',
                'end_header'
            ]
            
            # 将点云和颜色数据组合，并确保颜色值在有效范围内
            point_cloud_data = []
            for i in range(len(points_3d)):
                x, y, z = points_3d[i]
                # 确保颜色值在0-255范围内
                if i < len(colors):
                    r = max(0, min(255, int(colors[i, 0])))
                    g = max(0, min(255, int(colors[i, 1])))
                    b = max(0, min(255, int(colors[i, 2])))
                else:
                    r, g, b = 0, 0, 0
                # 格式化为PLY文件要求的格式
                point_cloud_data.append(f'{x:.6f} {y:.6f} {z:.6f} {r} {g} {b}')
            
            # 写入PLY文件，使用二进制模式的文本写入以确保格式正确
            with open(ply_file_path, 'w', encoding='utf-8') as f:
                f.write('\n'.join(header) + '\n')
                # 逐行写入以避免大文件时的内存问题
                for line in point_cloud_data:
                    f.write(line + '\n')
            
            # print(f'点云已成功保存到: {ply_file_path}')
            # print(f'保存的点云包含 {len(points_3d)} 个点')
            return True
        except Exception as e:
            print(f"保存点云到PLY文件时出错: {e}")
            return False
    
    def depth_to_point_cloud(self, depth_data, rgb_image, fx, fy, cx, cy, depth_scale=1):
        """
        从深度图和RGB图像生成3D点云
        
        参数:
            depth_data: 深度图像数据
            rgb_image: RGB彩色图像
            fx, fy: 相机的焦距
            cx, cy: 相机的主点坐标
            depth_scale: 深度值缩放因子，将深度值转换为米
        
        返回:
            tuple: (points_3d, colors) 生成的3D点云和对应颜色
        """
        try:
            # 过滤深度值
            dep_data = depth_data.copy()
            dep_data[dep_data > 10000.0] = 0
            dep_data = dep_data * depth_scale
            # 四舍五入保留两位小数
            dep_data = dep_data.round(2)
            
            # 计算dep_data的有效值个数（大于0的值）
            valid_count = (dep_data > 0.0).sum()
            # print(f"有效值个数: {valid_count},有效率：{valid_count / dep_data.size :.2f}")
            
            # 获取有效深度值的索引
            valid_indices = np.where(dep_data > 0.0)
            valid_depths = dep_data[valid_indices]
            
            # 创建点云数据
            points_3d = []
            colors = []
            
            for i in range(len(valid_indices[0])):
                u, v = valid_indices[1][i], valid_indices[0][i]  # 像素坐标
                z = valid_depths[i]  # 深度值（米）
                
                # 计算3D坐标
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                
                # 获取RGB颜色值
                color = rgb_image[v, u].tolist()
                
                points_3d.append([x, y, z])
                colors.append(color)
            
            # 转换为numpy数组
            points_3d = np.array(points_3d)
            colors = np.array(colors)
            
            # print(f"生成的3D点云数量: {points_3d.shape[0]}")
            # if points_3d.shape[0] > 0:
            #     print(f"3D点云范围 - X: [{points_3d[:, 0].min():.2f}, {points_3d[:, 0].max():.2f}] m")
            #     print(f"3D点云范围 - Y: [{points_3d[:, 1].min():.2f}, {points_3d[:, 1].max():.2f}] m")
            #     print(f"3D点云范围 - Z: [{points_3d[:, 2].min():.2f}, {points_3d[:, 2].max():.2f}] m")
            
            return points_3d, colors
        except Exception as e:
            print(f"从深度图生成点云时出错: {e}")
            return np.array([]), np.array([])
    
    def ply_to_obj(self, ply_file_path, obj_file_path):
        """
        将PLY文件转换为OBJ文件
        
        参数:
            ply_file_path: 输入PLY文件路径
            obj_file_path: 输出OBJ文件路径
            
        返回:
            bool: 是否成功转换
        """
        try:
            # 读取PLY文件
            points, colors = self.read_ply_file(ply_file_path)
            if len(points) == 0:
                print("错误: 无法读取PLY文件或文件为空")
                return False
            
            # 创建OBJ文件
            with open(obj_file_path, 'w', encoding='utf-8') as f:
                # 写入材质库声明
                f.write("mtllib material.mtl\n")
                f.write("usemtl point_material\n")
                
                # 写入顶点信息
                print(f"正在写入{len(points)}个顶点到OBJ文件...")
                for point in points:
                    f.write(f"v {point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")
                
                # 写入纹理坐标（如果有颜色信息）
                has_colors = len(colors) > 0 and colors.shape[1] >= 3
                if has_colors:
                    for color in colors:
                        # 将RGB转换为0-1范围的浮点数
                        r = max(0, min(255, int(color[0]))) / 255.0
                        g = max(0, min(255, int(color[1]))) / 255.0
                        b = max(0, min(255, int(color[2]))) / 255.0
                        # OBJ格式中使用顶点颜色作为纹理坐标的替代方案
                        # 注意：标准OBJ不直接支持顶点颜色，这里保存为vt行供参考
                        f.write(f"vt {r:.6f} {g:.6f} {b:.6f}\n")
                
                # 写入顶点法线（简单地使用向上方向作为法线）
                # 这是一个简化处理，实际应用中可能需要计算真实法线
                for _ in range(len(points)):
                    f.write("vn 0.000000 0.000000 1.000000\n")
                
                # 写入点元素
                # OBJ格式本身不直接支持点云，但可以使用顶点列表表示
                # 注意：某些OBJ查看器可能需要特殊处理才能正确显示点云
                # 这里我们将每个点作为单独的顶点列出
                f.write("\n# Point cloud data represented as vertices\n")
                
                # 可选：创建材质文件
                material_file = os.path.join(os.path.dirname(obj_file_path), "material.mtl")
                with open(material_file, 'w', encoding='utf-8') as mtl:
                    mtl.write("newmtl point_material\n")
                    mtl.write("Ka 1.000000 1.000000 1.000000\n")  # 环境光
                    mtl.write("Kd 1.000000 1.000000 1.000000\n")  # 漫反射
                    mtl.write("Ks 0.000000 0.000000 0.000000\n")  # 镜面反射
                    mtl.write("d 1.000000\n")  # 透明度
                    mtl.write("Ns 0.000000\n")  # 高光指数
                    mtl.write("illum 1\n")  # 光照模型
            
            print(f"成功将PLY文件转换为OBJ文件: {obj_file_path}")
            print(f"转换的顶点数量: {len(points)}")
            print(f"材质文件已创建: {os.path.basename(material_file)}")
            return True
        except Exception as e:
            print(f"将PLY转换为OBJ时出错: {e}")
            return False
    
    def _generate_sphere_points(self, center, radius, num_points, color):
        """
        使用黄金螺旋算法生成均匀分布的球体点
        
        参数:
            center: 球体中心点坐标
            radius: 球体半径
            num_points: 生成的点数量
            color: 点的颜色
            
        返回:
            tuple: (points, colors) 生成的球体点和颜色列表
        """
        sphere_points = []
        sphere_colors = []
        
        # 使用黄金螺旋算法生成均匀分布的球体点
        phi = np.pi * (3 - np.sqrt(5))  # 黄金角
        
        for i in range(num_points):
            y = 1 - (i / float(num_points - 1)) * 2  # y从1到-1
            radius_at_y = np.sqrt(1 - y * y)  # 半径在该y水平上的截面
            
            theta = phi * i  # 黄金角增量旋转
            
            x = math.cos(theta) * radius_at_y
            z = math.sin(theta) * radius_at_y
            
            # 缩放并平移到指定中心
            sphere_points.append([
                center[0] + x * radius,
                center[1] + y * radius,
                center[2] + z * radius
            ])
            sphere_colors.append(color)
        
        return sphere_points, sphere_colors
    
    def _generate_arrow_points(self, start_point, end_point, radius, num_points, color):
        """
        生成从start_point指向下一个点方向的短实体箭头
        箭头由实心圆柱体（箭身）和实心圆锥体（箭头）组成
        箭头不连接到下一个点，只表示方向
        
        参数:
            start_point: 箭头起点坐标
            end_point: 箭头指向的终点坐标
            radius: 基础半径参数
            num_points: 生成的点数
            color: 箭头颜色
            
        返回:
            tuple: (points, colors) 生成的箭头点和颜色列表
        """
        points = []
        colors = []
        
        # 计算方向向量
        direction = np.array(end_point) - np.array(start_point)
        length = np.linalg.norm(direction)
        if length == 0:
            # 如果起点和终点重合，返回球体
            return self._generate_sphere_points(start_point, radius, num_points, color)
        
        direction_normalized = direction / length
        
        # 计算正交基（用于生成圆柱体表面）
        if abs(direction_normalized[2]) < 0.9:  # 如果方向不是接近z轴
            up = np.array([0, 0, 1])
        else:
            up = np.array([1, 0, 0])
        
        right = np.cross(direction_normalized, up)
        right = right / np.linalg.norm(right)
        up = np.cross(right, direction_normalized)
        up = up / np.linalg.norm(up)
        
        # 设置固定的箭头总长度，不依赖于点之间的距离
        fixed_arrow_length = radius * 3.0  # 箭头总长度为半径的3倍
        
        # 生成实心箭身（圆柱体）
        shaft_length = fixed_arrow_length * 0.5  # 箭身占总长度的50%
        shaft_radius = radius * 0.3  # 箭身半径
        
        # 增加箭身的点数和层数，使其看起来更实心
        num_circles = 15  # 层数
        points_per_radius = 3  # 径向填充层数
        
        for i in range(num_circles + 1):
            t = i / num_circles
            # 只从起点向前延伸，不连接到下一个点
            circle_center = np.array(start_point) + direction_normalized * shaft_length * t
            
            # 径向填充：从中心到外边缘生成多个同心圆
            for r_factor in range(points_per_radius):
                current_radius = shaft_radius * (r_factor + 1) / points_per_radius
                # 外层圆的点数更多，内层圆的点数较少
                current_points_per_circle = max(4, int(6 * (r_factor + 1)))
                
                for j in range(current_points_per_circle):
                    angle = 2 * np.pi * j / current_points_per_circle
                    x = circle_center[0] + current_radius * (math.cos(angle) * right[0] + math.sin(angle) * up[0])
                    y = circle_center[1] + current_radius * (math.cos(angle) * right[1] + math.sin(angle) * up[1])
                    z = circle_center[2] + current_radius * (math.cos(angle) * right[2] + math.sin(angle) * up[2])
                    points.append([x, y, z])
                    colors.append(color)
        
        # 生成实心箭头（圆锥体）
        arrow_length = fixed_arrow_length * 0.5  # 箭头占总长度的50%
        arrow_radius = radius * 0.8  # 箭头半径
        arrow_base = np.array(start_point) + direction_normalized * shaft_length
        
        # 增加箭头的点数和层数，使其看起来更实心
        num_arrow_layers = 10  # 层数
        arrow_points_per_radius = 3  # 径向填充层数
        
        for i in range(num_arrow_layers + 1):
            t = i / num_arrow_layers
            current_radius = arrow_radius * (1 - t)
            # 只从箭身末端向前延伸，不连接到下一个点
            layer_center = arrow_base + direction_normalized * arrow_length * t
            
            # 径向填充：从中心到外边缘生成多个同心椭圆
            for r_factor in range(arrow_points_per_radius):
                current_point_radius = current_radius * (r_factor + 1) / arrow_points_per_radius
                # 外层圆的点数更多，内层圆的点数较少
                current_points_per_layer = max(3, int(5 * (r_factor + 1)))
                
                for j in range(current_points_per_layer):
                    angle = 2 * np.pi * j / current_points_per_layer
                    x = layer_center[0] + current_point_radius * (math.cos(angle) * right[0] + math.sin(angle) * up[0])
                    y = layer_center[1] + current_point_radius * (math.cos(angle) * right[1] + math.sin(angle) * up[1])
                    z = layer_center[2] + current_point_radius * (math.cos(angle) * right[2] + math.sin(angle) * up[2])
                    points.append([x, y, z])
                    colors.append(color)
        
        # 确保箭头顶点被添加
        arrow_tip = np.array(start_point) + direction_normalized * fixed_arrow_length
        points.append(arrow_tip)
        colors.append(color)
        
        # 移除采样机制，保留所有生成的点
        # 不再限制点数，确保箭头呈现完整的实体效果
        
        return points, colors
    
    def convert_labeled_points(self, labeled_points):
        """
        将包含标签的点列表转换为仅包含点坐标的列表，并进行坐标变换
        
        输入格式：[{"point": [x, y, z], "label": "0"}, ...]
        输出格式：[[X, Z, Y], ...]，其中X和Z取反
        
        参数:
            labeled_points: 包含标签的点列表
        
        返回:
            list: 转换后的点坐标列表
        """
        try:
            result = []
            for item in labeled_points:
                if "point" in item and len(item["point"]) >= 3:
                    # 提取点坐标 [x, y, z]
                    x, y, z = item["point"]
                    # 应用变换：[X, Y, Z] -> [-X, -Z, Y]
                    transformed_point = [-x, -z, y]
                    result.append(transformed_point)
                else:
                    print(f"警告: 无效的点数据格式: {item}")
            
            print(f"成功转换 {len(result)} 个带标签的点")
            return result
        except Exception as e:
            print(f"转换带标签的点时出错: {e}")
            return []
    
    def get_next_filename(self, base_path):
        """
        生成递增序号的文件名
        
        参数:
            base_path: 基础文件路径，如 "path/to/file.ply"
        
        返回:
            str: 递增序号的文件路径，如 "path/to/file_1.ply"
        """
        # 分离文件路径、文件名和扩展名
        directory, filename = os.path.split(base_path)
        name_without_ext, ext = os.path.splitext(filename)
        
        # 检查是否已经有带序号的文件
        counter = 1
        new_filename = f"{name_without_ext}_{counter}{ext}"
        new_path = os.path.join(directory, new_filename)
        
        # 找到下一个可用的序号
        while os.path.exists(new_path):
            counter += 1
            new_filename = f"{name_without_ext}_{counter}{ext}"
            new_path = os.path.join(directory, new_filename)
        
        return new_path
    
    def add_cube_annotation(self, input_ply_path, output_ply_path, cube_points, radius=0.2, points_per_edge=500):
        """
        添加立方体标注到PLY点云文件（绘制12条边和质心）
        
        参数:
            input_ply_path: 输入PLY文件路径
            output_ply_path: 输出PLY文件路径
            cube_points: 立方体点列表，格式为 [[centroid_x, centroid_y, centroid_z], [corner1_x, corner1_y, corner1_z], ...]
                        第一个点是质心，后八个点是立方体的八个角点，顺序为：左上前、右下前、右上前、左下前、左上后、右下后、右上后、左下后
            radius: 绘制尺寸，默认0.2
            points_per_edge: 每条边生成的点数，默认500
        
        返回:
            bool: 是否成功添加立方体标注
        """
        output_ply_path = self.get_next_filename(output_ply_path)
        try:
            # 读取原始点云
            original_points, original_colors = self.read_ply_file(input_ply_path)
            if len(original_points) == 0:
                return False
            
            # 将标注点扩展为球体/边，使其在点云中更明显
            all_points = original_points.tolist()
            all_colors = original_colors.tolist()
            
            # 检查输入的点数量是否正确
            if len(cube_points) < 9:
                print("错误: 输入的cube_points列表应包含9个点（1个质心和8个角点）")
                return False
            
            # 提取质心和8个角点
            centroid = cube_points[0]
            corner_points = cube_points[1:9]
            
            # 1. 添加质心点（修改为红色球体）
            centroid_points, centroid_colors = self._generate_sphere_points(
                centroid, radius, points_per_edge, self.RED
            )
            all_points.extend(centroid_points)
            all_colors.extend(centroid_colors)
            
            # 2. 绘制立方体的12条边（品红色）
            # 根据用户提供的角点顺序定义立方体12条边的连接关系（索引对）
            # 顺序为：左上前(0)、右下前(1)、右上前(2)、左下前(3)、左上后(4)、右下后(5)、右上后(6)、左下后(7)
            edge_indices = [
                (0, 2), (2, 1), (1, 3), (3, 0),  # 前方面四条边
                (4, 6), (6, 5), (5, 7), (7, 4),  # 后方面四条边
                (0, 4), (2, 6), (1, 5), (3, 7)   # 连接前后的四条边
            ]
            
            # 生成每条边的点
            for start_idx, end_idx in edge_indices:
                if start_idx < len(corner_points) and end_idx < len(corner_points):
                    start_point = corner_points[start_idx]
                    end_point = corner_points[end_idx]
                    edge_points, edge_colors = self._generate_edge_points(
                        start_point, end_point, radius, points_per_edge, self.PINK
                    )
                    all_points.extend(edge_points)
                    all_colors.extend(edge_colors)
            
            # 构建新的PLY文件
            total_points = len(all_points)
            
            # 创建PLY文件头部
            header = [
                'ply',
                'format ascii 1.0',
                f'element vertex {total_points}',
                'property float x',
                'property float y',
                'property float z',
                'property uchar red',
                'property uchar green',
                'property uchar blue',
                'end_header'
            ]
            
            # 写入新的PLY文件
            with open(output_ply_path, 'w', encoding='utf-8') as f:
                f.write('\n'.join(header) + '\n')
                
                for i in range(total_points):
                    x, y, z = all_points[i]
                    r = max(0, min(255, int(all_colors[i][0])))
                    g = max(0, min(255, int(all_colors[i][1])))
                    b = max(0, min(255, int(all_colors[i][2])))
                    f.write(f'{x:.6f} {y:.6f} {z:.6f} {r} {g} {b}\n')
            
            print(f"成功添加立方体标注到 {output_ply_path}")
            print(f"绘制了1个质心点（红色）和12条立方体边（品红色）")
            return True
            
        except Exception as e:
            print(f"添加立方体标注时出错: {e}")
            return False
    
    def _generate_edge_points(self, start_point, end_point, radius, num_points, color):
        """
        生成从start_point到end_point的实心边
        边由实心圆柱体组成
        
        参数:
            start_point: 边的起点坐标
            end_point: 边的终点坐标
            radius: 基础半径参数
            num_points: 生成的点数
            color: 边的颜色
            
        返回:
            tuple: (points, colors) 生成的边点和颜色列表
        """
        points = []
        colors = []
        
        # 计算方向向量
        direction = np.array(end_point) - np.array(start_point)
        length = np.linalg.norm(direction)
        if length == 0:
            # 如果起点和终点重合，返回球体
            return self._generate_sphere_points(start_point, radius, num_points, color)
        
        direction_normalized = direction / length
        
        # 计算正交基（用于生成圆柱体表面）
        if abs(direction_normalized[2]) < 0.9:  # 如果方向不是接近z轴
            up = np.array([0, 0, 1])
        else:
            up = np.array([1, 0, 0])
        
        right = np.cross(direction_normalized, up)
        right = right / np.linalg.norm(right)
        up = np.cross(right, direction_normalized)
        up = up / np.linalg.norm(up)
        
        # 设置固定的边粗细
        edge_radius = radius * 0.3  # 边的半径
        
        # 生成实心边（圆柱体）
        num_circles = 15  # 层数
        points_per_radius = 3  # 径向填充层数
        
        for i in range(num_circles + 1):
            t = i / num_circles
            # 从起点到终点生成圆柱体
            circle_center = np.array(start_point) + direction_normalized * length * t
            
            # 径向填充：从中心到外边缘生成多个同心圆
            for r_factor in range(points_per_radius):
                current_radius = edge_radius * (r_factor + 1) / points_per_radius
                # 外层圆的点数更多，内层圆的点数较少
                current_points_per_circle = max(4, int(6 * (r_factor + 1)))
                
                for j in range(current_points_per_circle):
                    angle = 2 * np.pi * j / current_points_per_circle
                    x = circle_center[0] + current_radius * (math.cos(angle) * right[0] + math.sin(angle) * up[0])
                    y = circle_center[1] + current_radius * (math.cos(angle) * right[1] + math.sin(angle) * up[1])
                    z = circle_center[2] + current_radius * (math.cos(angle) * right[2] + math.sin(angle) * up[2])
                    points.append([x, y, z])
                    colors.append(color)
        
        return points, colors
    
    def convert_labeled_points(self, labeled_points):
        """
        将包含标签的点列表转换为仅包含点坐标的列表，并进行坐标变换
        
        输入格式：[{"point": [x, y, z], "label": "0"}, ...]
        输出格式：[[X, Z, Y], ...]，其中X和Z取反
        
        参数:
            labeled_points: 包含标签的点列表
        
        返回:
            list: 转换后的点坐标列表
        """
        try:
            result = []
            for item in labeled_points:
                if "point" in item and len(item["point"]) >= 3:
                    # 提取点坐标 [x, y, z]
                    x, y, z = item["point"]
                    # 应用变换：[X, Y, Z] -> [-X, -Z, Y]
                    transformed_point = [-x, -z, y]
                    result.append(transformed_point)
                else:
                    print(f"警告: 无效的点数据格式: {item}")
            
            print(f"成功转换 {len(result)} 个带标签的点")
            return result
        except Exception as e:
            print(f"转换带标签的点时出错: {e}")
            return []
    
    def get_next_filename(self, base_path):
        """
        生成递增序号的文件名
        
        参数:
            base_path: 基础文件路径，如 "path/to/file.ply"
        
        返回:
            str: 递增序号的文件路径，如 "path/to/file_1.ply"
        """
        # 分离文件路径、文件名和扩展名
        directory, filename = os.path.split(base_path)
        name_without_ext, ext = os.path.splitext(filename)
        
        # 检查是否已经有带序号的文件
        counter = 1
        new_filename = f"{name_without_ext}_{counter}{ext}"
        new_path = os.path.join(directory, new_filename)
        
        # 找到下一个可用的序号
        while os.path.exists(new_path):
            counter += 1
            new_filename = f"{name_without_ext}_{counter}{ext}"
            new_path = os.path.join(directory, new_filename)
        
        return new_path

    def process_point_cloud(self, input_ply_path=None, depth_data=None, rgb_image=None, annotation_data=None, extend_distance=0.3, modeling_type=None, 
                                          output_ply_path=None, show_visualization=False, 
                                          fx=383.19929174573906, fy=384.76715878730715, 
                                          cx=317.944484051631, cy=231.71115593384292, 
                                          radius=0.05, ):
        """
        一站式处理点云：从深度数据生成点云或从PLY文件读取点云，添加标注，并保存到单个PLY文件
        
        参数:
            depth_data: 深度图像数据（如果使用深度数据生成点云）
            rgb_image: RGB彩色图像数据（如果使用深度数据生成点云）
            annotation_data: 用于添加标注的数据，根据modeling_type不同格式不同
                            - 对于"cube"类型: [[centroid_x, centroid_y, centroid_z], [corner1_x, corner1_y, corner1_z], ...] 或字典格式
                            - 对于"path"类型: [[x1, y1, z1], [x2, y2, z2], ...]
                            - 对于"sphere"类型: [[x1, y1, z1, r1], [x2, y2, z2, r2], ...]，或字典格式，
                              或新格式字典: {'id': int, 'label': str, 'center': [x, y, z], 'radius': float}
            modeling_type: 建模类型，可选值: "cube"(立方体)、"path"(路线)或"sphere"(球体)
            output_ply_path: 输出PLY文件路径
            show_visualization: 是否使用Open3D显示点云，默认为False
            fx, fy: 相机的焦距，默认为预定义值
            cx, cy: 相机的主点坐标，默认为预定义值
            radius: 标注元素的大小，默认为0.05
            input_ply_path: 输入PLY文件路径，如果提供，则从该文件读取点云数据，而不是从深度数据生成
        """
        self.model_type = modeling_type.lower()
        try:
            # 1. 获取点云数据：优先从input_ply_path读取，如果未提供则从深度数据生成
            if input_ply_path is not None:
                # 从PLY文件读取点云数据
                points_3d, colors = self.read_ply_file(input_ply_path)
                #print(f"从PLY文件读取点云数据: {input_ply_path}")
            else:
                # 检查必要参数
                if depth_data is None or rgb_image is None:
                    print("错误: 未提供input_ply_path，且缺少必要的depth_data或rgb_image参数")
                    return
                # 从深度数据和RGB图像生成点云
                rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
                points_3d, colors = self.depth_to_point_cloud(depth_data, rgb_image, fx, fy, cx, cy)
                # print("从深度数据和RGB图像生成点云")
            
            if len(points_3d) == 0:
                print("错误: 点云数据为空")
                return
            
            # 将原始点云转换为列表格式，便于处理
            all_points = points_3d.tolist()
            all_colors = colors.tolist()
            
            # 2. 根据建模类型添加标注
            if modeling_type.lower() == "cube":
                # 立方体建模类型
                # 检查是否为新的cubemodel格式（字典类型）
                if isinstance(annotation_data, dict):
                    # 从字典中提取数据
                    mass_center = annotation_data.get('mass_center', [0, 0, 0])
                    cube_center = annotation_data.get('cube_center', [0, 0, 0])
                    bbox3d = annotation_data.get('bbox3d', {})
                    
                    # 提取8个角点
                    corner_points = [
                        bbox3d.get('front_top_left', [0, 0, 0]),
                        bbox3d.get('front_bottom_right', [0, 0, 0]),
                        bbox3d.get('front_top_right', [0, 0, 0]),
                        bbox3d.get('front_bottom_left', [0, 0, 0]),
                        bbox3d.get('back_top_left', [0, 0, 0]),
                        bbox3d.get('back_bottom_right', [0, 0, 0]),
                        bbox3d.get('back_top_right', [0, 0, 0]),
                        bbox3d.get('back_bottom_left', [0, 0, 0])
                    ]
                    
                    # 添加质量中心（橙色球体）
                    mass_center_points, mass_center_colors = self._generate_sphere_points(
                        mass_center, radius, 1000, self.ORANGE
                    )
                    all_points.extend(mass_center_points)
                    all_colors.extend(mass_center_colors)
                    print(f"添加质量中心: {mass_center} (橙色)")
                    
                    # 添加立方体中心（青色球体）
                    cube_center_points, cube_center_colors = self._generate_sphere_points(
                        cube_center, radius, 1000, self.CYAN
                    )
                    all_points.extend(cube_center_points)
                    all_colors.extend(cube_center_colors)
                    print(f"添加立方体中心: {cube_center} (青色)")
                else:
                    # 旧格式：列表
                    if len(annotation_data) < 9:
                        print("错误: 立方体建模需要至少9个点（1个质心和8个角点）")
                        return
                    # 提取质心和8个角点
                    centroid = annotation_data[0]
                    corner_points = annotation_data[1:9]
                    
                    # 添加质心点（红色球体）- 旧格式保持兼容性
                    centroid_points, centroid_colors = self._generate_sphere_points(
                        centroid, radius, 1000, self.RED
                    )
                    all_points.extend(centroid_points)
                    all_colors.extend(centroid_colors)
                
                # 绘制立方体的12条边（品红色）
                # 顺序为：左上前(0)、右下前(1)、右上前(2)、左下前(3)、左上后(4)、右下后(5)、右上后(6)、左下后(7)
                edge_indices = [
                    (0, 2), (2, 1), (1, 3), (3, 0),  # 前方面四条边
                    (4, 6), (6, 5), (5, 7), (7, 4),  # 后方面四条边
                    (0, 4), (2, 6), (1, 5), (3, 7)   # 连接前后的四条边
                ]
                
                # 生成每条边的点
                for start_idx, end_idx in edge_indices:
                    if start_idx < len(corner_points) and end_idx < len(corner_points):
                        start_point = corner_points[start_idx]
                        end_point = corner_points[end_idx]
                        edge_points, edge_colors = self._generate_edge_points(
                            start_point, end_point, radius, 1000, self.PINK
                        )
                        all_points.extend(edge_points)
                        all_colors.extend(edge_colors)
            elif modeling_type.lower() == "path":
                # 路线建模类型
                if len(annotation_data) < 1:
                    print("错误: 路线建模需要至少1个点")
                    return
                
                # 添加路线标注点，与add_annotation_points功能相同
                for i, point in enumerate(annotation_data):
                    if len(annotation_data) == 1:
                        # 只有一个点的情况：显示紫色球体
                        sphere_points, sphere_colors = self._generate_sphere_points(
                            point, radius, 1000, self.PURPLE
                        )
                        all_points.extend(sphere_points)
                        all_colors.extend(sphere_colors)
                    elif i == 0:
                        # 起始点：红色球体 + 蓝色箭头
                        # 添加红色球体
                        sphere_points, sphere_colors = self._generate_sphere_points(
                            point, radius, 1000, self.RED
                        )
                        all_points.extend(sphere_points)
                        all_colors.extend(sphere_colors)
                        
                        # 添加指向第二个点的蓝色箭头
                        next_point = annotation_data[i + 1]
                        arrow_points, arrow_colors = self._generate_arrow_points(
                            point, next_point, radius, 1000, self.BLUE
                        )
                        all_points.extend(arrow_points)
                        all_colors.extend(arrow_colors)
                    elif i < len(annotation_data) - 1:
                        # 中间点：蓝色箭头（不显示球体）
                        next_point = annotation_data[i + 1]
                        arrow_points, arrow_colors = self._generate_arrow_points(
                            point, next_point, radius, 1000, self.BLUE
                        )
                        all_points.extend(arrow_points)
                        all_colors.extend(arrow_colors)
                    else:
                        # 最后一个点：绿色球体
                        sphere_points, sphere_colors = self._generate_sphere_points(
                            point, radius, 1000, self.GREEN
                        )
                        all_points.extend(sphere_points)
                        all_colors.extend(sphere_colors)
            elif modeling_type.lower() == "sphere":
                for each in annotation_data:
                    sphere_center = each['center']
                    sphere_radius = each['safety_radius']

                    # 添加球体中心（橙色球体）
                    sphere_center_points, sphere_center_colors = self._generate_sphere_points(
                        sphere_center, sphere_radius/10, 500, self.ORANGE
                    )
                    all_points.extend(sphere_center_points)
                    all_colors.extend(sphere_center_colors)
                    # print(f"添加球体中心: {sphere_center} (橙色)，半径: {sphere_radius/10}")
                    
                    # 添加膨胀球体（半透明青色）
                    full_sphere_points, full_sphere_colors = self._generate_sphere_points(
                        sphere_center, sphere_radius, 1000, self.CYAN
                    )
                    all_points.extend(full_sphere_points)
                    all_colors.extend(full_sphere_colors)

                    # 添加真实球体（绿色）
                    full_sphere_points, full_sphere_colors = self._generate_sphere_points(
                        sphere_center, sphere_radius-extend_distance, 1000, self.PINK
                    )
                    all_points.extend(full_sphere_points)
                    all_colors.extend(full_sphere_colors)

            else:
                print(f"错误: 不支持的建模类型 '{modeling_type}'，可选值为 'cube'、'path' 或 'sphere'")
                return
            
            # 3. 创建唯一的输出文件名   
            # 4. 直接保存完整的点云（原始点云+标注）到单个PLY文件
            self._save_combined_point_cloud(all_points, all_colors, output_ply_path)
            
            # 5. 可选地使用Open3D显示点云
            if show_visualization and HAS_OPEN3D:
                self._visualize_with_open3d(all_points, all_colors)
                
        except Exception as e:
            print(f"处理点云时出错: {e}")
    
    def _save_combined_point_cloud(self, all_points, all_colors, ply_file_path):
        """
        保存合并后的点云数据到PLY文件
        """
        try:
            # 确保points不为空
            if len(all_points) == 0:
                print("错误: 没有点云数据可保存")
                return
            
            # 创建PLY文件头部
            header = [
                'ply',
                'format ascii 1.0',
                f'element vertex {len(all_points)}',
                'property float x',
                'property float y',
                'property float z',
                'property uchar red',
                'property uchar green',
                'property uchar blue',
                'end_header'
            ]
            
            # 写入PLY文件
            with open(ply_file_path, 'w', encoding='utf-8') as f:
                f.write('\n'.join(header) + '\n')
                
                for i in range(len(all_points)):
                    x, y, z = all_points[i]
                    # 确保颜色值在0-255范围内
                    if i < len(all_colors):
                        r = max(0, min(255, int(all_colors[i][0])))
                        g = max(0, min(255, int(all_colors[i][1])))
                        b = max(0, min(255, int(all_colors[i][2])))
                    else:
                        r, g, b = 0, 0, 0
                    f.write(f'{x:.6f} {y:.6f} {z:.6f} {r} {g} {b}\n')
            
            # print(f'点云构建成功: {ply_file_path}')
            # print(f'保存的点云包含 {len(all_points)} 个点')
            
        except Exception as e:
            print(f"保存点云到PLY文件时出错: {e}")
    
    def _visualize_with_open3d(self, points, colors):
        """
        使用Open3D可视化点云
        """
        try:
            # 创建Open3D点云对象
            pcd = o3d.geometry.PointCloud()
            
            # 设置点坐标
            pcd.points = o3d.utility.Vector3dVector(np.array(points))
            
            # 设置点颜色（Open3D需要[0,1]范围的浮点数）
            color_array = np.array(colors)
            color_array = np.clip(color_array / 255.0, 0, 1)
            pcd.colors = o3d.utility.Vector3dVector(color_array)
            
            # 可视化点云
            o3d.visualization.draw_geometries([pcd], window_name="3DPointCloudVisualization")
            
        except Exception as e:
            print(f"使用Open3D可视化点云时出错: {e}")

if __name__ == "__main__":
    point_cloud_utils = PointCloudUtils()
    dep_data=[]
    rgb_image=[]
    point_cloud = point_cloud_utils.depth_to_point_cloud(dep_data, rgb_image, 383.19929174573906, 384.76715878730715, 317.944484051631, 231.71115593384292)

    result = point_cloud_utils.save_point_cloud_to_ply(point_cloud[0], point_cloud[1], "./log/20251025_221217/point_cloud.ply")
    print(result)
    annotation_points = [[1,0,0], [1,1,0], [1,2,1], [0,1,1]]
    result = point_cloud_utils.add_annotation_points("./log/20251025_221217/point_cloud.ply", "./log/20251025_221217/point_cloud_mask.ply", annotation_points,radius=0.05)
    print(result)
    result = point_cloud_utils.ply_to_obj("./log/20251025_221217/point_cloud_mask.ply", "./log/20251025_221217/point_cloud_mask.obj")
    print(result)
    
    # 测试convert_labeled_points函数
    labeled_points = [
        {'point': [0.0, 0.0, 0.0], 'label': '0'},
        {'point': [-0.5, 2.0, -0.2], 'label': '1'},
        {'point': [-1.0, 4.0, -0.4], 'label': '2'},
        {'point': [-1.5, 6.0, -0.3], 'label': '3'},
        {'point': [-1.771, 8.475, -0.094], 'label': '4'}
    ]
    converted_points = point_cloud_utils.convert_labeled_points(labeled_points)
    print("转换后的点列表:", converted_points)

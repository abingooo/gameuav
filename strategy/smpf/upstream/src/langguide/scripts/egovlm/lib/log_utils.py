# /home/uav/vla/lib/logread.py
"""
logread库 - 用于读取和处理日志目录中的图像和深度数据

这个库提供了简单易用的接口来读取、处理和显示存储在日志目录中的数据，
包括彩色图像、深度可视化图像和原始深度数据(npy文件)。
"""

import numpy as np
import cv2
import os
from PIL import Image
from datetime import datetime
from typing import Dict, List, Tuple, Optional, Union


class LogReader:
    """
    日志读取器类，用于读取和处理日志目录中的数据
    """
    
    def __init__(self, log_dir: str = './log'):
        """
        初始化日志读取器
        
        Args:
            log_dir: 日志根目录路径
        """
        self.log_dir = log_dir
        
    def read_depth_npy(self, file_path: str) -> np.ndarray:
        """
        读取深度图像npy文件
        
        Args:
            file_path: npy文件路径
            
        Returns:
            numpy数组，包含深度数据
            
        Raises:
            FileNotFoundError: 如果文件不存在
            Exception: 如果读取过程中出错
        """
        if not os.path.exists(file_path):
            raise FileNotFoundError(f"文件不存在: {file_path}")
        
        try:
            return np.load(file_path)
        except Exception as e:
            raise Exception(f"读取npy文件失败: {str(e)}")
    
    def read_image(self, file_path: str) -> np.ndarray:
        """
        读取图像文件（支持jpg、png等格式）
        
        Args:
            file_path: 图像文件路径
            
        Returns:
            numpy数组，包含图像数据（BGR格式）
            
        Raises:
            FileNotFoundError: 如果文件不存在
            Exception: 如果读取过程中出错
        """
        if not os.path.exists(file_path):
            raise FileNotFoundError(f"文件不存在: {file_path}")
        
        image = cv2.imread(file_path)
        if image is None:
            raise Exception(f"读取图像文件失败: {file_path}")
        
        return image
    
    def get_log_directories(self, sort_by_time: bool = True, reverse: bool = True) -> List[str]:
        """
        获取所有日志目录
        
        Args:
            sort_by_time: 是否按时间排序
            reverse: 是否降序排序（最新的在前）
            
        Returns:
            日志目录名称列表
            
        Raises:
            FileNotFoundError: 如果日志根目录不存在
        """
        if not os.path.exists(self.log_dir):
            raise FileNotFoundError(f"日志目录不存在: {self.log_dir}")
        
        # 获取所有日志子目录
        log_subdirs = [d for d in os.listdir(self.log_dir) if os.path.isdir(os.path.join(self.log_dir, d))]
        
        # 按时间排序
        if sort_by_time:
            log_subdirs.sort(
                key=lambda d: os.path.getmtime(os.path.join(self.log_dir, d)), 
                reverse=reverse
            )
        
        return log_subdirs
    
    def get_latest_log_directory(self) -> str:
        """
        获取最新的日志目录
        
        Returns:
            最新日志目录的名称
            
        Raises:
            FileNotFoundError: 如果日志根目录不存在
            ValueError: 如果没有找到日志子目录
        """
        log_dirs = self.get_log_directories(sort_by_time=True, reverse=True)
        
        if not log_dirs:
            raise ValueError(f"在 {self.log_dir} 中没有找到日志子目录")
        
        return log_dirs[0]
    
    def parse_timestamp(self, timestamp_str: str) -> Optional[datetime]:
        """
        解析时间戳字符串
        
        Args:
            timestamp_str: 时间戳字符串，格式应为 "%Y%m%d_%H%M%S"
            
        Returns:
            datetime对象，如果解析失败则返回None
        """
        try:
            return datetime.strptime(timestamp_str, "%Y%m%d_%H%M%S")
        except ValueError:
            return None
    
    def get_log_info(self, log_name: str) -> Dict[str, Union[str, Optional[datetime]]]:
        """
        获取日志目录的信息
        
        Args:
            log_name: 日志目录名称
            
        Returns:
            包含日志信息的字典
        """
        log_path = os.path.join(self.log_dir, log_name)
        
        # 解析时间戳
        timestamp = self.parse_timestamp(log_name)
        
        # 获取目录创建时间
        try:
            created_time = datetime.fromtimestamp(os.path.getctime(log_path))
        except OSError:
            created_time = None
        
        # 检查文件是否存在
        has_color = os.path.exists(os.path.join(log_path, 'color_image.jpg'))
        has_depth_vis = os.path.exists(os.path.join(log_path, 'depth_image.jpg'))
        has_depth_npy = os.path.exists(os.path.join(log_path, 'depth_image.npy'))
        has_log_file = os.path.exists(os.path.join(log_path, 'capture_log.txt'))
        
        return {
            'name': log_name,
            'path': log_path,
            'timestamp': timestamp,
            'created_time': created_time,
            'has_color_image': has_color,
            'has_depth_visualization': has_depth_vis,
            'has_depth_data': has_depth_npy,
            'has_log_file': has_log_file
        }
    
    def read_log_data(self, log_name: str) -> Dict[str, Union[np.ndarray, str, None]]:
        """
        读取指定日志目录中的所有数据
        
        Args:
            log_name: 日志目录名称
            
        Returns:
            包含所有数据的字典
            
        Raises:
            FileNotFoundError: 如果日志目录不存在
        """
        log_path = os.path.join(self.log_dir, log_name)
        
        if not os.path.exists(log_path):
            raise FileNotFoundError(f"日志目录不存在: {log_path}")
        
        # 构建文件路径
        color_path = os.path.join(log_path, 'color_image.jpg')
        depth_vis_path = os.path.join(log_path, 'depth_image.jpg')
        depth_npy_path = os.path.join(log_path, 'depth_image.npy')
        log_file_path = os.path.join(log_path, 'capture_log.txt')
        
        # 读取文件
        data = {
            'log_name': log_name,
            'log_path': log_path,
            'color_image': None,
            'depth_visualization': None,
            'depth_data': None,
            'log_content': None
        }
        
        try:
            if os.path.exists(color_path):
                data['color_image'] = self.read_image(color_path)

        except Exception as e:
            print(f"读取彩色图像失败: {str(e)}")
        
        try:
            if os.path.exists(depth_vis_path):
                data['depth_visualization'] = self.read_image(depth_vis_path)
        except Exception as e:
            print(f"读取深度可视化图像失败: {str(e)}")
        
        try:
            if os.path.exists(depth_npy_path):
                data['depth_data'] = self.read_depth_npy(depth_npy_path)
        except Exception as e:
            print(f"读取深度数据失败: {str(e)}")
        
        try:
            if os.path.exists(log_file_path):
                with open(log_file_path, 'r', encoding='utf-8') as f:
                    data['log_content'] = f.read()
        except Exception as e:
            print(f"读取日志文件失败: {str(e)}")
        
        return data
    
    def read_latest_log(self) -> Dict[str, Union[np.ndarray, str, None]]:
        """
        读取最新的日志数据
        
        Returns:
            包含所有数据的字典
        """
        latest_log = self.get_latest_log_directory()
        return self.read_log_data(latest_log)
    
    def display_images(self, color_img: np.ndarray, depth_img: np.ndarray, 
                      window_names: Tuple[str, str] = ('Color Image', 'Depth Visualization')) -> None:
        """
        显示彩色图像和深度可视化图像
        
        Args:
            color_img: 彩色图像数据
            depth_img: 深度可视化图像数据
            window_names: 窗口名称元组
        """
        # 创建窗口
        cv2.namedWindow(window_names[0], cv2.WINDOW_NORMAL)
        cv2.namedWindow(window_names[1], cv2.WINDOW_NORMAL)
        
        # 显示图像
        cv2.imshow(window_names[0], color_img)
        cv2.imshow(window_names[1], depth_img)
        
        # 等待用户按键
        print("按任意键关闭图像窗口...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
    def analyze_depth_data(self, depth_data: np.ndarray) -> Dict[str, Union[int, float]]:
        """
        分析深度数据，计算统计信息
        
        Args:
            depth_data: 深度数据数组
            
        Returns:
            包含统计信息的字典
        """
        # 过滤掉0值（通常表示无效深度）
        valid_depth = depth_data[depth_data > 0]
        
        stats = {
            'shape': depth_data.shape,
            'dtype': str(depth_data.dtype),
            'total_pixels': depth_data.size,
            'valid_pixels': valid_depth.size,
            'invalid_pixels': depth_data.size - valid_depth.size,
            'has_valid_data': valid_depth.size > 0
        }
        
        if valid_depth.size > 0:
            stats.update({
                'max_depth': float(valid_depth.max()),
                'min_depth': float(valid_depth.min()),
                'mean_depth': float(valid_depth.mean()),
                'median_depth': float(np.median(valid_depth)),
                'std_depth': float(valid_depth.std())
            })
        
        return stats
    
    def list_logs(self, show_details: bool = False) -> None:
        """
        列出所有可用的日志目录
        
        Args:
            show_details: 是否显示详细信息
        """
        logs = self.get_log_directories()
        
        print("\n可用的日志目录:")
        print("-" * 60)
        
        for i, log_dir_name in enumerate(logs, 1):
            # 基本信息
            timestamp = self.parse_timestamp(log_dir_name)
            time_str = timestamp.strftime("%Y-%m-%d %H:%M:%S") if timestamp else "未知时间"
            
            if not show_details:
                print(f"{i:2d}. {log_dir_name} ({time_str})")
            else:
                # 显示详细信息
                info = self.get_log_info(log_dir_name)
                status_icon = '✓' if all([info['has_color_image'], info['has_depth_visualization'], info['has_depth_data']]) else '⚠'
                print(f"{i:2d}. {log_dir_name} [{status_icon}] ({time_str})")
                print(f"    彩色图像: {'✓' if info['has_color_image'] else '✗'}")
                print(f"    深度可视化: {'✓' if info['has_depth_visualization'] else '✗'}")
                print(f"    深度数据: {'✓' if info['has_depth_data'] else '✗'}")
        
        print("-" * 60)
        print(f"总计: {len(logs)} 个日志目录")


# 提供简单的使用示例和便捷函数
def create_reader(log_dir: str = './log') -> LogReader:
    """
    创建日志读取器实例的便捷函数
    
    Args:
        log_dir: 日志根目录路径
        
    Returns:
        LogReader实例
    """
    return LogReader(log_dir)

def read_latest_data(log_dir: str = './log') -> Dict[str, Union[np.ndarray, str, None]]:
    """
    读取最新日志数据的便捷函数
    
    Args:
        log_dir: 日志根目录路径
        
    Returns:
        包含所有数据的字典
    """
    reader = LogReader(log_dir)
    return reader.read_latest_log()

def list_all_logs(log_dir: str = './log', show_details: bool = False) -> None:
    """
    列出所有日志的便捷函数
    
    Args:
        log_dir: 日志根目录路径
        show_details: 是否显示详细信息
    """
    reader = LogReader(log_dir)
    reader.list_logs(show_details)
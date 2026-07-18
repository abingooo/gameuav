
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R

class Solve3D:
    def __init__(self, camera_params=None, camera_config_path=None):
        """
        camera_params: dict，需包含 fx, fy, cx, cy（单位与 depth 一致）。
        camera_config_path: 可选，JSON 文件路径，若 camera_params 为空则尝试从此处读取。
                            默认优先读取 config/config.json 的 camera 字段，
                            若不存在则回退到 config/camera.json。
        """
        if camera_params is None:
            camera_params = self._load_camera_params(camera_config_path)

        required = ("fx", "fy", "cx", "cy")
        if not isinstance(camera_params, dict) or not all(k in camera_params for k in required):
            raise ValueError(f"camera_params 需包含 {required}")
        self.camera_params = camera_params

    def calculate_3d_position(self, pixel_coord, depth, precision=2):
        """
        根据像素坐标和深度计算3D坐标（右手系，前-左-上）。
        
        Args:
            pixel_coord: [x, y]（u,v）或序列 [[x,y], ...]
            depth: 深度值或与 pixel_coord 等长的序列（米或与内参一致的单位）
            precision: 输出小数位
            
        Returns:
            单点返回 [x,y,z]；序列返回 [[x,y,z], ...]，坐标系为前-左-上
        """
        if pixel_coord is None:
            raise ValueError("pixel_coord 无效")
        is_single = isinstance(pixel_coord, (list, tuple)) and len(pixel_coord) == 2 and not isinstance(pixel_coord[0], (list, tuple))
        coords = [pixel_coord] if is_single else pixel_coord

        # 深度支持单值或序列
        if depth is None:
            raise ValueError("depth 无效")
        if isinstance(depth, (int, float)):
            depths = [depth] * len(coords)
        else:
            depths = depth
        if len(depths) != len(coords):
            raise ValueError("depth 长度需与像素坐标一致")

        fx, fy, cx, cy = (self.camera_params[k] for k in ("fx", "fy", "cx", "cy"))

        results = []
        for (u, v), d in zip(coords, depths):
            if d is None or d <= 0:
                raise ValueError(f"depth 必须为正数，收到 {d}")
            u = float(u)
            v = float(v)
            x_cam = (u - cx) * d / fx   # 右
            y_cam = (v - cy) * d / fy   # 下
            z_cam = d                   # 前

            x_flu = z_cam               # 前
            y_flu = -x_cam              # 左
            z_flu = -y_cam              # 上

            results.append([
                round(float(x_flu), precision),
                round(float(y_flu), precision),
                round(float(z_flu), precision),
            ])

        return results[0] if is_single else results

    def get_depth_values(self, depth_map: np.ndarray, pixel_coords, default=None):
        """
        从深度图读取单个像素或像素序列的深度值。

        Args:
            depth_map: 2D/3D numpy 深度图（H x W [x C]），单位与内参一致。
            pixel_coords: [x, y] 或 [[x, y], ...] 序列。
            default: 越界时返回的值，默认 None（会抛错）。
        Returns:
            单个像素输入返回 float，序列返回同序列表。
        """
        if not isinstance(depth_map, np.ndarray) or depth_map.ndim < 2:
            raise ValueError("depth_map 必须为二维/三维 numpy 数组")

        h, w = depth_map.shape[:2]
        is_single = isinstance(pixel_coords, (list, tuple)) and len(pixel_coords) == 2 and not isinstance(pixel_coords[0], (list, tuple))
        coords_list = [pixel_coords] if is_single else pixel_coords

        depths = []
        for idx, pt in enumerate(coords_list):
            if not isinstance(pt, (list, tuple)) or len(pt) < 2:
                raise ValueError(f"像素坐标格式错误: {pt}")
            x, y = pt[0], pt[1]
            xi = int(x)
            yi = int(y)
            if xi < 0 or xi >= w or yi < 0 or yi >= h:
                if default is None:
                    raise ValueError(f"像素越界: ({x},{y})")
                depths.append(default)
                continue
            depths.append(float(depth_map[yi, xi]))

        return depths[0] if is_single else depths

    def depth_stats(
        self,
        depth_map: np.ndarray,
        pixel_coords,
        mode: str = "mean",
        default=None,
        lower_q: float = 5.0,
        upper_q: float = 95.0,
    ):
        """
        计算一组像素对应深度的单项统计量。

        mode 可选：
          - "min", "max", "median", "mean"
          - "min_clipped", "max_clipped", "median_clipped", "mean_clipped"（按分位去异常后再算）

        Args:
            depth_map: 深度图 numpy 数组。
            pixel_coords: 单个 [x,y] 或列表 [[x,y], ...]。
            mode: 选择要计算的统计量。
            default: 越界时的默认深度；None 时越界抛错。
            lower_q, upper_q: 去异常分位阈值（针对 *_clipped 模式）。
        Returns:
            float，所选统计值。
        """
        depths = self.get_depth_values(depth_map, pixel_coords, default=default)
        arr = np.asarray(depths, dtype=float)
        # 过滤无效与零深度
        arr = arr[np.isfinite(arr) & (arr != 0)]
        if arr.size == 0:
            raise ValueError("无有效深度值用于统计")

        base_modes = {"min", "max", "median", "mean"}
        clipped_modes = {"min_clipped", "max_clipped", "median_clipped", "mean_clipped"}
        if mode not in base_modes | clipped_modes:
            raise ValueError(f"不支持的 mode: {mode}")

        def _calc(a, which):
            if which == "min":
                return float(np.min(a))
            if which == "max":
                return float(np.max(a))
            if which == "median":
                return float(np.median(a))
            if which == "mean":
                return float(np.mean(a))
            raise ValueError(which)

        if mode in base_modes:
            return _calc(arr, mode)

        # clipped
        if arr.size < 3:
            return _calc(arr, mode.replace("_clipped", ""))
        lo, hi = np.percentile(arr, [lower_q, upper_q])
        clipped = arr[(arr >= lo) & (arr <= hi)]
        if clipped.size == 0:
            clipped = arr  # fallback
        return _calc(clipped, mode.replace("_clipped", ""))

    def build_sphere_model_from_sam(
        self,
        depth_map: np.ndarray,
        sam_points_dict: dict,
        min_radius: float = 0.3,
        radius_scale: float = 0.56,
        depth_mode: str = "median_clipped",
        prefer_axis: int = 2,
        prefer_max: bool = True,
        precision: int = 3,
    ):
        """
        基于 SAM 结果构建目标球体模型，返回 JSON 友好的字典。

        Args:
            depth_map: 深度图。
            sam_points_dict: 需包含 center / rdmpoints / box_2d。
            min_radius: 最小球半径下限。
            radius_scale: (最大角点距离/2) 的缩放系数。
            depth_mode: 深度统计模式，默认 median_clipped。
            prefer_axis: 球心候选选择的轴索引，默认 z 轴。
            prefer_max: True 选该轴最大值，False 选最小值。
            precision: 数值保留位数。
        Returns:
            dict: {
                "avg_depth", "objctr3dpos", "corners_2d", "corner3dpos",
                "max_corner3d_distance", "radius", "sphere_center", "sphere_centers"
            }
        """
        if not isinstance(sam_points_dict, dict):
            raise ValueError("sam_points_dict 必须为 dict")
        for k in ("center", "rdmpoints", "box_2d"):
            if k not in sam_points_dict:
                raise ValueError(f"sam_points_dict 缺少键: {k}")
        if not isinstance(sam_points_dict["box_2d"], (list, tuple)) or len(sam_points_dict["box_2d"]) != 4:
            raise ValueError("sam_points_dict['box_2d'] 应为 [ymin, xmin, ymax, xmax]")

        points_for_depth = [sam_points_dict["center"], *sam_points_dict["rdmpoints"]]
        avg_depth = float(self.depth_stats(depth_map, points_for_depth, mode=depth_mode))
        objctr3dpos = self.calculate_3d_position(sam_points_dict["center"], avg_depth, precision=precision)

        ymin, xmin, ymax, xmax = sam_points_dict["box_2d"]
        corners_2d = [[xmin, ymin], [xmax, ymin], [xmin, ymax], [xmax, ymax]]
        corner3dpos = self.calculate_3d_position(corners_2d, [avg_depth] * 4, precision=precision)

        corner_pairs = ((0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3))
        max_corner3d_distance = max(
            float(np.linalg.norm(np.asarray(corner3dpos[i]) - np.asarray(corner3dpos[j])))
            for i, j in corner_pairs
        )
        radius = max((max_corner3d_distance / 2.0) * float(radius_scale), float(min_radius))

        sphere_centers = self.calculate_sphere_centers(corner3dpos, radius, precision=precision)
        sphere_center = self.pick_sphere_center(
            sphere_centers, prefer_axis=prefer_axis, prefer_max=prefer_max
        )
        if not sphere_center:
            sphere_center = objctr3dpos

        return {
            "avg_depth": round(avg_depth, precision),
            "objctr3dpos": objctr3dpos,
            "corners_2d": corners_2d,
            "corner3dpos": corner3dpos,
            "max_corner3d_distance": round(max_corner3d_distance, precision),
            "radius": round(radius, precision),
            "sphere_center": sphere_center,
            "sphere_centers": sphere_centers,
        }

    @staticmethod
    def point_at_distance_from_b(a, b, distance, precision=3):
        """
        计算线段 AB 上、距离点 B 为 distance 的点坐标（从 B 指向 A）。
        Args:
            a, b: 点坐标，可迭代，至少包含 xyz。
            distance: 从 B 沿 BA 方向的距离，需为非负且不大于 |AB|。
            precision: 结果保留的小数位。
        Returns:
            tuple (x, y, z)。
        """
        if a is None or b is None:
            raise ValueError("点 A/B 不能为空")
        if distance is None or distance < 0:
            raise ValueError("distance 需为非负数")

        try:
            ax, ay, az = float(a[0]), float(a[1]), float(a[2])
            bx, by, bz = float(b[0]), float(b[1]), float(b[2])
        except Exception as exc:
            raise ValueError("点坐标格式错误，需提供 xyz") from exc

        vec = np.array([ax - bx, ay - by, az - bz], dtype=float)
        seg_len = float(np.linalg.norm(vec))
        if seg_len == 0:
            raise ValueError("A 与 B 重合，无法计算方向")
        if distance > seg_len:
            raise ValueError(f"distance 超过 |AB|，不在两点之间 (distance={distance}, |AB|={seg_len})")

        unit = vec / seg_len
        point = np.array([bx, by, bz], dtype=float) + unit * distance
        return tuple(round(float(v), precision) for v in point)

    @staticmethod
    def calculate_sphere_centers(corners, radius, tol=1e-8, precision=2):
        """
        四个共面角点 + 半径，计算球心候选解（0/1/2 个）。

        Args:
            corners: [[x,y,z], ...]，长度应为 4。
            radius: 球半径。
            tol: 数值容差。
            precision: 输出保留小数位。
        Returns:
            list: [] 或 [[x,y,z], [x,y,z]]。
        """
        if corners is None or len(corners) != 4:
            raise ValueError("corners 需为 4 个三维点")
        if radius is None or radius <= 0:
            raise ValueError("radius 需为正数")

        P = np.asarray(corners, dtype=float)
        if P.shape != (4, 3):
            raise ValueError("corners 格式需为 [[x,y,z], [x,y,z], [x,y,z], [x,y,z]]")

        # 平面内中心 O（四点均值）
        O = P.mean(axis=0)
        # 平面内等效半径 R0（O 到角点距离）
        R0 = float(np.linalg.norm(P[0] - O))
        if radius < R0 - tol:
            return []

        # 平面法向量（两条边叉乘）
        v1 = P[1] - P[0]
        v2 = P[2] - P[0]
        n = np.cross(v1, v2)
        norm_n = float(np.linalg.norm(n))
        if norm_n < tol:
            return []
        n = n / norm_n

        # 沿法线方向偏移
        h_sq = float(radius**2 - R0**2)
        if h_sq < 0:
            if h_sq > -tol:
                h_sq = 0.0
            else:
                return []
        h = float(np.sqrt(h_sq))

        c1 = O + h * n
        c2 = O - h * n
        c1 = [round(float(v), precision) for v in c1]
        c2 = [round(float(v), precision) for v in c2]
        return [c1, c2]

    @staticmethod
    def pick_sphere_center(corner_centers, prefer_axis=2, prefer_max=True):
        """
        从候选球心中挑选一个解；默认选择 z 更大的点（FLU 坐标系里更“上方”）。
        """
        if not corner_centers:
            return []
        if len(corner_centers) == 1:
            return corner_centers[0]
        if prefer_max:
            return max(corner_centers, key=lambda p: p[prefer_axis])
        return min(corner_centers, key=lambda p: p[prefer_axis])

    @staticmethod
    def _load_camera_params(camera_config_path=None):
        """
        读取相机参数，优先使用统一 config.json 中的 camera 段，回退到独立 camera.json。
        """
        config_dir = Path(__file__).resolve().parents[2] / "config"
        candidates = []
        if camera_config_path:
            candidates.append(Path(camera_config_path))
        else:
            candidates.append(config_dir / "config.json")
            candidates.append(config_dir / "camera.json")

        last_error = None
        for path in candidates:
            try:
                with Path(path).open("r", encoding="utf-8") as f:
                    data = json.load(f)
            except FileNotFoundError:
                last_error = FileNotFoundError(f"缺少相机参数文件: {path}")
                continue
            except Exception as exc:
                last_error = exc
                continue

            if isinstance(data, dict) and "camera" in data:
                data = data["camera"]
            if isinstance(data, dict):
                return data
            last_error = ValueError(f"{path} 中未找到相机参数")

        if last_error:
            raise last_error
        raise FileNotFoundError("缺少相机参数文件")

    @staticmethod
    def camera2world_transform(now_pos, now_ort, tgt_pos, precision=None):
        """
        坐标转换：局部坐标系 -> 世界坐标系 (带精度控制)
        
        参数:
        now_pos (list/np.array): 当前观测者的世界坐标 [x, y, z]
        now_ort (list/np.array): 当前姿态的四元数 [x, y, z, w]
        tgt_pos (list/np.array): 目标在局部坐标系下的位置 [x, y, z]
        precision (int, optional): 小数保留位数。默认为 None (不处理)。
        
        返回:
        np.array: 目标在世界坐标系下的位置 [x, y, z]
        """
        # 确保输入是 numpy 数组
        p_now = np.array(now_pos)
        p_tgt_local = np.array(tgt_pos)
        
        # 1. 创建旋转对象并执行旋转
        # 注意：如果 now_ort 没归一化，可以通过 R.from_quat(now_ort / np.linalg.norm(now_ort)) 保证严谨
        rotation = R.from_quat(now_ort)
        p_tgt_rotated = rotation.apply(p_tgt_local)
        
        # 2. 加上当前位置的平移偏移量
        p_tgt_world = p_tgt_rotated + p_now
        
        # 3. 精度控制
        if precision is not None:
            return np.round(p_tgt_world, decimals=precision)
        
        return p_tgt_world


    @staticmethod
    def add_vectors(a, b, precision=None):
        """
        将两个同长度向量按元素相加。
        Args:
            a, b: 可迭代数值，长度需一致。
            precision: 可选，指定小数位数。
        Returns:
            tuple 相加结果。
        """
        if a is None or b is None:
            raise ValueError("向量不能为空")
        if len(a) != len(b):
            raise ValueError("向量长度不一致")
        summed = [float(x) + float(y) for x, y in zip(a, b)]
        if precision is not None:
            summed = [round(v, precision) for v in summed]
        return tuple(summed)

if __name__ == "__main__":

    s = Solve3D()
    curr_pos = [10, 5, 1]
    curr_ort = [0.0, 0.0, 0.70710678, 0.70710678]  # 绕Z轴旋转90度
    target_local = [2, 0.0, 0.0]
    # 保留 3 位小数
    result = Solve3D.camera2world_transform(
        curr_pos, curr_ort, target_local, precision=3
    )

    print(f"原始转换结果: {result}") 
    # 输出示例: [10.235, 7.802, 1.   ]
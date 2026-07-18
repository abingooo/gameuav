#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
图像相关的处理与保存工具。
当前仅实现目标区域提取（用于后续 SAM 分割前的裁剪）。
"""

from __future__ import annotations

from typing import Any, Dict, Optional
import argparse
import sys
import os
import copy

import cv2
import numpy as np


class ImageProcess:
    """
    图像处理相关方法。
    """

    @staticmethod
    def extract_target_region(rgb_image: np.ndarray, box_2d) -> Optional[Dict[str, Any]]:
        """
        从 RGB 图像中提取目标区域，并稍微外扩边界框以便分割。

        Args:
            rgb_image: RGB 格式图像（HWC）。
            box_2d: box_2d（[y1, x1, y2, x2]）

        Returns:
            dict: 包含 bbox( [[x1,y1],[x2,y2]] )、imageROI。
            若缺少 box_2d 则返回 None。
        """

        box = box_2d.copy()
        if len(box) != 4:
            return None

        # 外扩边界框，增大分割视野
        box[0] = int(box[0] * 0.8)
        box[1] = int(box[1] * 0.8)
        box[2] = int(box[2] * 1.2)
        box[3] = int(box[3] * 1.2)

        y1, x1, y2, x2 = box

        height, width = rgb_image.shape[:2]
        y1 = max(0, min(y1, height - 1))
        x1 = max(0, min(x1, width - 1))
        y2 = max(0, min(y2, height - 1))
        x2 = max(0, min(x2, width - 1))

        # 若坐标顺序被破坏，则纠正为左上/右下
        if y2 < y1:
            y1, y2 = y2, y1
        if x2 < x1:
            x1, x2 = x2, x1

        image_roi = rgb_image[y1 : y2 + 1, x1 : x2 + 1]
        return {
            "bbox": [[x1, y1], [x2, y2]],
            "imageROI": image_roi,
        }

    @staticmethod
    def roi_to_image_coordinates(points: Any, pbox: Any, rgb_shape: Optional[tuple] = None) -> Any:
        """
        将 ROI 内的坐标映射回原始图像坐标系。
        Args:
            points: 单点 [x,y] / (x,y)、点列表 [[x,y], ...]，或末维>=2 的 numpy 数组。
            box: ROI 的边界框 [y1, x1, y2, x2] 或等价的可迭代。
            rgb_shape: 可选 (H, W)，若提供则对偏移做边界裁剪，与 extract_target_region 一致。
        Returns:
            与输入类型一致的坐标，已加上 ROI 左上角偏移。
        """
        box = pbox.copy()
        if len(box) != 4:
            raise ValueError(f"box_2d 长度错误: {box}")

        # 使用与 extract_target_region 相同的外扩与裁剪规则
        box[0] = int(box[0] * 0.8)
        box[1] = int(box[1] * 0.8)
        box[2] = int(box[2] * 1.2)
        box[3] = int(box[3] * 1.2)
        y1, x1, y2, x2 = box

        if rgb_shape:
            h, w = rgb_shape[:2]
            y1 = max(0, min(y1, h - 1))
            x1 = max(0, min(x1, w - 1))
            y2 = max(0, min(y2, h - 1))
            x2 = max(0, min(x2, w - 1))

        if y2 < y1:
            y1, y2 = y2, y1
        if x2 < x1:
            x1, x2 = x2, x1

        x_offset, y_offset = x1, y1

        if isinstance(points, np.ndarray):
            result = points.copy()
            if result.shape[-1] < 2:
                raise ValueError(f"点维度不足: {result.shape}")
            result[..., 0] = result[..., 0] + x_offset
            result[..., 1] = result[..., 1] + y_offset
            return result

        if isinstance(points, (list, tuple)) and points and not isinstance(points[0], (list, tuple)):
            if len(points) < 2:
                raise ValueError(f"点维度不足: {points}")
            return [points[0] + x_offset, points[1] + y_offset, *points[2:]]

        if isinstance(points, (list, tuple)):
            converted = []
            for pt in points:
                if not isinstance(pt, (list, tuple)) or len(pt) < 2:
                    raise ValueError(f"点格式不正确: {pt}")
                converted.append([pt[0] + x_offset, pt[1] + y_offset, *pt[2:]])
            return converted

        raise TypeError(f"不支持的 points 类型: {type(points).__name__}")

    @staticmethod
    def denormalize_bbox_1000(norm_bbox: Any, image_shape: tuple) -> Optional[list]:
        """
        将 0-1000 范围的标准化框 [ymin, xmin, ymax, xmax] 转换为像素坐标。

        Args:
            norm_bbox: 形如 [ymin, xmin, ymax, xmax] 的列表/元组。
            image_shape: 原图形状 (H, W) 或 (H, W, C)。
        Returns:
            [ymin, xmin, ymax, xmax] 像素坐标（int），无效输入则返回 None。
        """
        if not isinstance(norm_bbox, (list, tuple)) or len(norm_bbox) != 4:
            return None
        h, w = image_shape[0], image_shape[1]
        ymin, xmin, ymax, xmax = norm_bbox
        y_scale = h / 1000.0
        x_scale = w / 1000.0
        y1 = int(round(ymin * y_scale))
        x1 = int(round(xmin * x_scale))
        y2 = int(round(ymax * y_scale))
        x2 = int(round(xmax * x_scale))
        return [y1, x1, y2, x2]

    @staticmethod
    def samresult_adjust(samresult:Dict, box_2d:list):
        if samresult is None:
            raise ValueError("samresult 为空")
        # 根据现有日志，bbox 信息位于 bounding_box 内
        bbox_src = samresult.get("bounding_box", {})
        required_keys = ("x1", "y1", "x2", "y2")
        for k in required_keys:
            if k not in bbox_src:
                raise ValueError(f"samresult 缺少键: {k}")
        if "centroid" not in samresult or "random_points" not in samresult:
            raise ValueError("samresult 缺少 centroid 或 random_points")

        resb = ImageProcess.roi_to_image_coordinates(
            [[bbox_src["x1"], bbox_src["y1"]], [bbox_src["x2"], bbox_src["y2"]]],
            box_2d,
            (480,640)
        )
        bbox = [resb[0][1], resb[0][0], resb[1][1], resb[1][0]]
        center = ImageProcess.roi_to_image_coordinates(samresult["centroid"],box_2d,(480,640))
        rdmpoints = ImageProcess.roi_to_image_coordinates(samresult["random_points"],box_2d,(480,640))
        return {"box_2d":bbox,"center":center,"rdmpoints":rdmpoints}


class ImageSave:
    """图像保存相关方法。"""

    @staticmethod
    def save_numpy_image(image: np.ndarray, save_path: str) -> None:
        """
        将 numpy 图像保存到指定路径。
        Args:
            image: np.ndarray，支持灰度或 BGR/RGB。
            save_path: 保存文件路径，后缀决定编码格式（.jpg/.png 等）。
        Raises:
            ValueError: 图像为空或形状不符合预期。
            RuntimeError: 保存失败。
        """
        if image is None or not isinstance(image, np.ndarray) or image.size == 0:
            raise ValueError("图像为空或类型错误")
        if image.ndim not in (2, 3):
            raise ValueError(f"不支持的图像维度: {image.shape}")

        # OpenCV 期望 BGR/灰度，无需额外转换；若是 RGB 也可直接保存。
        ok = cv2.imwrite(save_path, image)
        if not ok:
            raise RuntimeError(f"保存图像失败: {save_path}")

    @staticmethod
    def save_with_bbox_and_text(
        image: np.ndarray,
        bbox: list,
        text: str,
        save_path: str,
        box_color: tuple = (0, 0, 255),
        text_color: tuple = (0, 0, 255),
        thickness: int = 2,
        font_scale: float = 0.7,
    ) -> None:
        """
        在图像上绘制矩形框并贴上文本后保存。

        Args:
            image: 原始图像 np.ndarray。
            bbox: 像素坐标 [ymin, xmin, ymax, xmax]。
            text: 要绘制的文本。
            save_path: 输出路径。
            box_color: 矩形框颜色 (B, G, R)，默认红色。
            text_color: 文本颜色 (B, G, R)，默认红色。
            thickness: 框线宽度。
            font_scale: 文本缩放。
        """
        if image is None or not isinstance(image, np.ndarray) or image.size == 0:
            raise ValueError("图像为空或类型错误")
        if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
            raise ValueError(f"bbox 应为长度4的列表/元组，当前: {bbox}")

        h, w = image.shape[:2]
        y1, x1, y2, x2 = map(int, bbox)
        # 边界裁剪
        y1, y2 = max(0, min(y1, h - 1)), max(0, min(y2, h - 1))
        x1, x2 = max(0, min(x1, w - 1)), max(0, min(x2, w - 1))
        if y2 < y1:
            y1, y2 = y2, y1
        if x2 < x1:
            x1, x2 = x2, x1

        canvas = image.copy()
        cv2.rectangle(canvas, (x1, y1), (x2, y2), box_color, thickness)

        draw_text = str(text) if text is not None else ""
        if draw_text:
            font = cv2.FONT_HERSHEY_SIMPLEX
            (tw, th), baseline = cv2.getTextSize(draw_text, font, font_scale, thickness)
            text_x = x1
            text_y = max(th + baseline, y1 - 4)
            cv2.putText(canvas, draw_text, (text_x, text_y), font, font_scale, text_color, thickness, cv2.LINE_AA)

        ok = cv2.imwrite(save_path, canvas)
        if not ok:
            raise RuntimeError(f"保存图像失败: {save_path}")

    @staticmethod
    def save_with_points_and_text(
        image: np.ndarray,
        points: list,
        texts: Optional[Any],
        save_path: str,
        point_color: tuple = (0, 255, 0),
        point_radius: int = 4,
        point_thickness: int = -1,
        text_color: tuple = (0, 0, 255),
        font_scale: float = 0.6,
        text_offset: tuple = (6, -6),
    ) -> None:
        """
        在图像上绘制点并为每个点添加文本后保存。

        Args:
            image: 原始图像 np.ndarray。
            points: 点列表 [[x, y], ...] 或 [(x, y), ...]，像素坐标。
            texts: 文本字符串，或与 points 等长的字符串列表；None 则不绘制文本。
            save_path: 输出路径。
            point_color: 点颜色 (B, G, R)，默认绿色。
            point_radius: 点半径，像素。
            point_thickness: 点线宽，-1 表示实心。
            text_color: 文本颜色 (B, G, R)。
            font_scale: 文本缩放。
            text_offset: 文本相对于点的 (dx, dy) 偏移。
        """
        if image is None or not isinstance(image, np.ndarray) or image.size == 0:
            raise ValueError("图像为空或类型错误")
        if not isinstance(points, (list, tuple)) or len(points) == 0:
            raise ValueError("points 需为非空列表/元组")

        h, w = image.shape[:2]
        canvas = image.copy()

        # 统一文本列表长度
        if texts is None:
            text_list = [""] * len(points)
        elif isinstance(texts, str):
            text_list = [texts] + [""] * (len(points) - 1)
        elif isinstance(texts, (list, tuple)):
            if len(texts) not in (1, len(points)):
                raise ValueError("texts 长度需为1或与 points 等长")
            text_list = list(texts)
            if len(text_list) == 1 and len(points) > 1:
                text_list = [text_list[0]] + [""] * (len(points) - 1)
        else:
            raise TypeError("texts 类型不支持")

        font = cv2.FONT_HERSHEY_SIMPLEX
        dx, dy = text_offset

        for idx, pt in enumerate(points):
            if not isinstance(pt, (list, tuple)) or len(pt) < 2:
                raise ValueError(f"点格式不正确: {pt}")
            x, y = int(pt[0]), int(pt[1])
            x = max(0, min(x, w - 1))
            y = max(0, min(y, h - 1))
            cv2.circle(canvas, (x, y), point_radius, point_color, point_thickness)

            label = str(text_list[idx]) if text_list[idx] is not None else ""
            if label:
                (tw, th), baseline = cv2.getTextSize(label, font, font_scale, 1)
                text_x = max(0, min(x + dx, w - tw))
                text_y = max(th + baseline, min(y + dy, h - 1))
                cv2.putText(canvas, label, (text_x, text_y), font, font_scale, text_color, 1, cv2.LINE_AA)

        ok = cv2.imwrite(save_path, canvas)
        if not ok:
            raise RuntimeError(f"保存图像失败: {save_path}")

    @staticmethod
    def save_sam_result(
        image: np.ndarray,
        result: Any,
        label: Any,
        save_path: str,
        box_color: tuple = (0, 255, 255),
        box_thickness: int = 2,
        center_color: tuple = (255, 255, 0),
        center_radius: int = 5,
        points_color: tuple = (0, 255, 0),
        points_radius: int = 4,
        text_color: tuple = (0, 255, 255),
        font_scale: float = 1,
    ) -> None:
        """
        绘制检测/分割结果（框 + 中心点 + 随机点）并保存。

        Args:
            image: 原始图像 np.ndarray。
            result: 单个 dict 或与 label 等长的 dict 列表，需含 box_2d、center、rdmpoints（像素坐标）。
            label: 单个字符串或与 result 等长的字符串列表。
            save_path: 输出路径。
            box_color: 框颜色 (B, G, R)。
            box_thickness: 框线宽。
            center_color: 中心点颜色。
            center_radius: 中心点半径。
            points_color: 随机点颜色。
            points_radius: 随机点半径。
            text_color: 文本颜色。
            font_scale: 文本缩放。
        """
        if image is None or not isinstance(image, np.ndarray) or image.size == 0:
            raise ValueError("图像为空或类型错误")

        results = result if isinstance(result, (list, tuple)) else [result]
        labels = label if isinstance(label, (list, tuple)) else [label]
        if len(results) != len(labels):
            raise ValueError("result 与 label 长度不一致")

        for idx, res in enumerate(results):
            if not isinstance(res, dict):
                raise ValueError(f"result[{idx}] 必须为字典")
            for k in ("box_2d", "center", "rdmpoints"):
                if k not in res:
                    raise ValueError(f"result[{idx}] 缺少键: {k}")

        h, w = image.shape[:2]
        canvas = image.copy()
        font = cv2.FONT_HERSHEY_SIMPLEX

        for res, lbl in zip(results, labels):
            ymin, xmin, ymax, xmax = map(int, res["box_2d"])
            ymin, ymax = max(0, min(ymin, h - 1)), max(0, min(ymax, h - 1))
            xmin, xmax = max(0, min(xmin, w - 1)), max(0, min(xmax, w - 1))
            if ymax < ymin:
                ymin, ymax = ymax, ymin
            if xmax < xmin:
                xmin, xmax = xmax, xmin

            cv2.rectangle(canvas, (xmin, ymin), (xmax, ymax), box_color, box_thickness)

            draw_text = str(lbl) if lbl is not None else ""
            if draw_text:
                (tw, th), baseline = cv2.getTextSize(draw_text, font, font_scale, 1)
                text_x = xmin
                text_y = max(th + baseline, ymin - 4)
                cv2.putText(canvas, draw_text, (text_x, text_y), font, font_scale, text_color, 2, cv2.LINE_AA)

            cx, cy = map(int, res["center"])
            cx = max(0, min(cx, w - 1))
            cy = max(0, min(cy, h - 1))
            cv2.circle(canvas, (cx, cy), center_radius, center_color, -1)

            rdmpoints = res.get("rdmpoints") or []
            for pt in rdmpoints:
                if not isinstance(pt, (list, tuple)) or len(pt) < 2:
                    continue
                px, py = int(pt[0]), int(pt[1])
                px = max(0, min(px, w - 1))
                py = max(0, min(py, h - 1))
                cv2.circle(canvas, (px, py), points_radius, points_color, -1)

        ok = cv2.imwrite(save_path, canvas)
        if not ok:
            raise RuntimeError(f"保存图像失败: {save_path}")

if __name__ == "__main__":
    print(ImageProcess.roi_to_image_coordinates([[0,0],[208,375]],[130, 160, 450, 280],(480,640)))

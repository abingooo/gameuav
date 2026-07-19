#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
SAMClient: 调用 LSAM 服务进行图像分割，支持文件路径或 numpy 图像输入。
不包含可视化逻辑，异常会抛出给调用方以便上层处理。
"""

from __future__ import annotations

import base64
from pathlib import Path
from typing import Any, Dict, Optional, Union
import argparse
import os
import sys

import cv2
import numpy as np
import requests


class SAMClient:
    def __init__(
        self,
        server_ip: Optional[str] = None,
        server_port: Optional[int] = None,
        timeout: Optional[float] = None,
        session: Optional[requests.Session] = None,
    ) -> None:
        """
        Args:
            server_ip: 推理服务 IP。
            server_port: 推理服务端口。
            timeout: 单次请求超时时间，秒。
            session: 可选的 requests.Session 以复用连接。
        """
        self.server_ip = server_ip or os.environ.get("SMPF_SAM_HOST", "10.246.1.94")
        self.server_port = int(
            server_port if server_port is not None else os.environ.get("SMPF_SAM_PORT", "5002")
        )
        self.timeout = float(
            timeout if timeout is not None else os.environ.get("SMPF_SAM_TIMEOUT_SEC", "20")
        )
        self._session = session or requests.Session()

    # 公共接口 -----------------------------------------------------------------
    def predict(self, image_input: Union[str, Path, np.ndarray], text_prompt: str) -> Dict[str, Any]:
        """
        调用 /predict 接口获取分割结果。

        Args:
            image_input: 图像路径或 numpy 数组（HWC，BGR/RGB/灰度均可）。
            text_prompt: 文本提示。
        Returns:
            服务器返回的 JSON 字典。
        Raises:
            FileNotFoundError: 路径输入不存在。
            TypeError/ValueError: 输入格式不符合要求。
            RuntimeError: 请求或解析失败。
        """
        img_b64 = self._encode_image(image_input)
        url = f"http://{self.server_ip}:{self.server_port}/predict"
        payload = {"image": img_b64, "text": text_prompt}

        try:
            resp = self._session.post(url, json=payload, timeout=self.timeout)
        except Exception as exc:
            raise RuntimeError(f"LSAM 请求失败: {exc}") from exc

        if resp.status_code != 200:
            msg = self._safe_json(resp) or resp.text
            raise RuntimeError(f"LSAM 返回异常状态码 {resp.status_code}: {msg}")

        data = self._safe_json(resp)
        if data is None:
            raise RuntimeError("LSAM 响应解析失败：非 JSON 格式")
        masks = data.get("masks")
        if not isinstance(masks, list) or not masks:
            raise RuntimeError("LSAM 响应缺少 masks 数据")

        if all(isinstance(m, dict) and "area" in m for m in masks):
            best_mask = max(masks, key=lambda m: m.get("area", 0))
        else:
            best_mask = masks[0]

        return best_mask

    # 内部工具 -----------------------------------------------------------------
    @staticmethod
    def _encode_image(image_input: Union[str, Path, np.ndarray]) -> str:
        """将输入图像编码为 base64 JPEG 字符串。"""
        if isinstance(image_input, (str, Path)):
            image_path = Path(image_input)
            if not image_path.exists():
                raise FileNotFoundError(f"图像文件不存在: {image_path}")
            binary = image_path.read_bytes()
            return base64.b64encode(binary).decode("utf-8")

        if not isinstance(image_input, np.ndarray):
            raise TypeError(f"不支持的图像输入类型: {type(image_input).__name__}")

        image_arr = SAMClient._to_uint8(image_input)
        if image_input.ndim == 2:
            image_bgr = cv2.cvtColor(image_arr, cv2.COLOR_GRAY2BGR)
        elif image_input.ndim == 3 and image_input.shape[2] in (3, 4):
            if image_input.shape[2] == 4:
                image_bgr = cv2.cvtColor(image_arr, cv2.COLOR_BGRA2BGR)
            else:
                # 默认认为输入为 BGR 或 RGB；无需转换通道顺序，保持原样编码
                image_bgr = image_arr
        else:
            raise ValueError(f"不支持的图像形状: {image_input.shape}")

        ok, encoded = cv2.imencode(".jpg", image_bgr, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not ok:
            raise RuntimeError("图像编码失败")
        return base64.b64encode(encoded.tobytes()).decode("utf-8")

    @staticmethod
    def _to_uint8(arr: np.ndarray) -> np.ndarray:
        """尽量保持数值信息，将常见浮点 0-1 或 0-255 范围转为 uint8。"""
        if arr.dtype == np.uint8:
            return arr
        if np.issubdtype(arr.dtype, np.floating):
            max_val = float(np.nanmax(arr))
            scale = 255.0 if max_val <= 1.0 else 1.0
            return np.clip(arr * scale, 0, 255).round().astype(np.uint8)
        return np.clip(arr, 0, 255).astype(np.uint8)

    @staticmethod
    def _safe_json(resp: requests.Response) -> Optional[Any]:
        try:
            return resp.json()
        except Exception:
            return None


def _cli() -> int:
    """
    简单命令行测试：
    python SAM.py --image path/to/img.jpg --text "find tree" --ip 127.0.0.1 --port 5002
    若未提供 --image，则会生成一张黑色 224x224 测试图像。
    """
    parser = argparse.ArgumentParser(description="SAMClient test runner")
    parser.add_argument("--image", type=Path, help="图像路径，留空则使用随机测试图像")
    parser.add_argument("--text", required=True, help="分割/检测提示文本")
    parser.add_argument("--ip", default="127.0.0.1", help="LSAM 服务 IP")
    parser.add_argument("--port", type=int, default=5002, help="LSAM 服务端口")
    parser.add_argument("--timeout", type=float, default=10.0, help="请求超时时间（秒）")
    args = parser.parse_args()

    if args.image:
        image_input: Union[Path, np.ndarray] = args.image
    else:
        # 构造一张简单的黑色图像，便于快速验证请求流程
        image_input = np.zeros((224, 224, 3), dtype=np.uint8)

    client = SAMClient(server_ip=args.ip, server_port=args.port, timeout=args.timeout)
    try:
        result = client.predict(image_input, args.text)
        print("LSAM result:", result)
        return 0
    except Exception as exc:
        print(f"LSAM 调用失败: {exc}")
        return 1


if __name__ == "__main__":
    sys.exit(_cli())

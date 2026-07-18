#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
读取模型配置，并提供等价于 curl 示例的极简 LLM 封装。
配置中的 key/base_url 是唯一来源，代码内不再硬编码。
"""

import base64
import os
import sys
import json
from concurrent.futures import FIRST_COMPLETED, ThreadPoolExecutor, wait
from pathlib import Path
from typing import Any, Dict, Generator, List, Optional, Union
import cv2
import numpy as np
import requests

# 确保当前文件所在目录以及上层包根目录加入模块搜索路径
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = Path(__file__).resolve().parents[2]  # guide_module
for path in (SCRIPT_DIR, str(PKG_ROOT)):
    if path not in sys.path:
        sys.path.insert(0, path)

from utils.Format import Format


def load_model_info(config_path: Path = None) -> Dict[str, Any]:
    """
    从 config/config.json 或 config/modelinfo.json 读取配置。
    Args:
        config_path: 可选自定义路径，默认优先 config/config.json，再回退到 config/modelinfo.json
    Returns:
        dict，读取失败则返回空字典。
    """
    try:
        base_dir = Path(__file__).resolve().parents[2] / "config"
        if config_path is not None:
            candidates = [Path(config_path)]
        else:
            candidates = [base_dir / "config.json", base_dir / "modelinfo.json"]

        for path in candidates:
            if not path.exists():
                continue
            with path.open("r", encoding="utf-8") as f:
                data = json.load(f)
            if isinstance(data, dict) and "modelinfo" in data:
                section = data.get("modelinfo")
                return section if isinstance(section, dict) else {}
            if isinstance(data, dict):
                return data
        return {}
    except Exception:
        return {}


class LLM:
    """
    LLM 客户端HTTP POST 请求。
    """

    def __init__(
        self,
        model_id: str = "gpt-5.2",
        model_dsp: str = "文本对话与推理",
        api_key: Optional[str] = None,
        base_url: Optional[str] = None,
        temperature = 0.5
    ) -> None:
        info = load_model_info()
        self.model_id = model_id or info.get("model_id", "gpt-4o-mini")
        self.model_dsp = model_dsp or info.get("model_dsp", "")
        # Environment variables override the imported compatibility config.
        self.api_key = (
            api_key
            or os.environ.get("SMPF_LLM_API_KEY")
            or os.environ.get("OPENAI_API_KEY")
            or info.get("openaikey")
            or info.get("OPENAIKEY")
            or ""
        )
        config_base = (
            os.environ.get("SMPF_LLM_BASE_URL")
            or os.environ.get("OPENAI_BASE_URL")
            or info.get("base_url")
            or info.get("BASE_URL")
        )
        self.base_url = base_url or config_base or ""
        # 确保指向 chat/completions 端点
        if self.base_url and "/chat/completions" not in self.base_url:
            self.base_url = self.base_url.rstrip("/") + "/chat/completions"
        if not self.api_key:
            raise ValueError("Missing SMPF_LLM_API_KEY or OPENAI_API_KEY")
        if not self.base_url:
            raise ValueError("缺少 base_url，请在 config/modelinfo.json 设置 base_url/BASE_URL 或在构造时传入 base_url。")

        self.temperature = temperature

    def chat(self, messages: List[Dict[str, str]], temperature = None) -> str:
        """
        发送聊天消息，返回模型回复文本（无流式）。
        messages 示例: [{"role": "user", "content": "Hello!"}]
        """
        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.api_key}",
        }
        payload = {
            "model": self.model_id,
            "messages": messages,
            "temperature": temperature,
        }
        if temperature is None:
            temperature = self.temperature
        resp = requests.post(self.base_url, headers=headers, data=json.dumps(payload))
        if resp.status_code != 200:
            raise RuntimeError(f"API请求失败: {resp.status_code} - {resp.text}")
        data = resp.json()
        try:
            return data["choices"][0]["message"]["content"]
        except Exception as exc:
            raise RuntimeError(f"响应解析失败: {data}") from exc

    def chat_text(self, text: str, temperature = None) -> Dict[str, Any]:
        """便捷接口：仅输入一段用户文本即可获取回复，附带耗时。"""
        import time
        start = time.time()
        if temperature is None:
            temperature = self.temperature
        content = self.chat([{"role": "user", "content": text}], temperature=temperature)
        content = Format.parse_json_block(content)
        return {"content": content, "time_cost": round(time.time() - start, 2)}

    def chat_multi(
        self,
        messages: Union[str, List[Dict[str, str]]],
        repeat: int = 2,
        temperature = None,
        max_workers: Optional[int] = None,
    ) -> Generator[Dict[str, Any], None, None]:
        """
        使用相同输入并发调用模型多次，返回最先完成的一个结果，其余任务会被取消。
        Args:
            messages: 聊天消息列表，或直接传入用户文本字符串。
            repeat: 调用次数。
            temperature: 采样温度。
            max_workers: 线程池并发度，默认与 repeat 相同，上限 32。
        Yields:
            dict，包含 content、time_cost 或 error（仅首个完成的任务）。
        """
        if repeat <= 0:
            raise ValueError("repeat 必须为正整数")
        max_workers = max(1, min(max_workers or repeat, repeat, 32))
        if isinstance(messages, str):
            messages = [{"role": "user", "content": messages}]

        if temperature is None:
            temperature = self.temperature

        def _task(idx: int) -> Dict[str, Any]:
            import time
            start = time.time()
            content = self.chat(messages, temperature=temperature)
            content = Format.parse_json_block(content)
            return {"content": content, "time_cost": round(time.time() - start, 2)}

        executor = ThreadPoolExecutor(max_workers=max_workers)
        futures = {executor.submit(_task, i): i for i in range(repeat)}
        try:
            done, pending = wait(futures, return_when=FIRST_COMPLETED)
            first_future = next(iter(done))
            try:
                first_result = first_future.result()
            except Exception as exc:
                first_result = {"error": str(exc)}
            for f in pending:
                f.cancel()
            yield first_result
        finally:
            try:
                executor.shutdown(wait=False, cancel_futures=True)
            except TypeError:
                executor.shutdown(wait=False)


class VLM:
    """
    简单 VLM 客户端，输入提示文本与 numpy 图像，返回模型响应文本。
    需要在 config/modelinfo.json 提供 base_url（或 vlm_base_url/BASE_URL），以及可选 openaikey。
    """

    def __init__(
        self,
        model_id: str = "gemini-2.5-flash",
        model_dsp: str = "视觉理解与多模态对话",
        api_key: Optional[str] = None,
        base_url: Optional[str] = None,
        temperature = 0.5
    ) -> None:
        info = load_model_info()
        self.model_id = model_id or info.get("vlm_model_id", "vlm-model")
        self.model_dsp = model_dsp or info.get("vlm_model_dsp", "")
        self.api_key = (
            api_key
            or os.environ.get("SMPF_VLM_API_KEY")
            or os.environ.get("OPENAI_API_KEY")
            or info.get("openaikey")
            or info.get("OPENAIKEY")
            or ""
        )
        config_base = (
            os.environ.get("SMPF_VLM_BASE_URL")
            or os.environ.get("OPENAI_BASE_URL")
            or info.get("base_url")
            or info.get("BASE_URL")
        )
        self.base_url = base_url or config_base or ""
        if not self.base_url:
            raise ValueError("缺少 base_url，请在 config/modelinfo.json 设置 base_url/BASE_URL 或在构造时传入 base_url。")
        # VLM 与 LLM 相同接口：确保指向 chat/completions 端点
        if "/chat/completions" not in self.base_url:
            self.base_url = self.base_url.rstrip("/") + "/chat/completions"
        self.temperature = temperature

    @staticmethod
    def _encode_image_rgb(image: np.ndarray, quality: int = 60) -> str:
        """将 RGB 图像编码为 base64 JPEG 字符串。"""
        if image is None:
            raise ValueError("图像为空")
        if len(image.shape) == 3 and image.shape[2] == 3:
            bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        else:
            bgr = image
        ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            raise ValueError("图像编码失败")
        return base64.b64encode(buf.tobytes()).decode("utf-8")

    def analyze(
        self,
        rgb_image: np.ndarray,
        prompt: str,
        temperature = None,
        repeat: int = 1,
        max_workers: Optional[int] = None,
    ) -> Dict[str, Any]:
        """
        发送图像与文本提示；repeat>1 时并发调用，返回最先完成的结果，其余任务会被取消。
        Args:
            rgb_image: RGB 图像。
            prompt: 文本提示。
            temperature: 采样温度。
            repeat: 调用次数，<=1 则单次调用。
            max_workers: 线程池并发度，默认与 repeat 相同，上限 32。
        Returns:
            dict，包含 content、time_cost 或 error（并发时仅首个完成的任务）。
        """
        import time
        if repeat <= 0:
            raise ValueError("repeat 必须为正整数")
        img_b64 = self._encode_image_rgb(rgb_image)
        if temperature is None:
            temperature = self.temperature
        headers = {"Content-Type": "application/json"}
        if self.api_key:
            headers["Authorization"] = f"Bearer {self.api_key}"

        payload = {
            "model": self.model_id,
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{img_b64}"
                            },
                        },
                    ],
                }
            ],
            "temperature": temperature,
        }

        # 单次调用直接执行
        if repeat == 1:
            start = time.time()
            resp = requests.post(self.base_url, headers=headers, data=json.dumps(payload))
            if resp.status_code != 200:
                raise RuntimeError(f"VLM请求失败: {resp.status_code} - {resp.text}")
            data = resp.json()
            try:
                content = Format.parse_json_block(data["choices"][0]["message"]["content"])
                return {"content": content, "time_cost": round(time.time() - start, 2)}
            except Exception as exc:
                raise RuntimeError(f"响应解析失败: {data}") from exc

        # 并发调用取首个完成
        max_workers = max(1, min(max_workers or repeat, repeat, 32))
        payload_json = json.dumps(payload)

        def _task() -> Dict[str, Any]:
            start = time.time()
            resp = requests.post(self.base_url, headers=headers, data=payload_json)
            if resp.status_code != 200:
                raise RuntimeError(f"VLM请求失败: {resp.status_code} - {resp.text}")
            data = resp.json()
            try:
                content = Format.parse_json_block(data["choices"][0]["message"]["content"])
                return {"content": content, "time_cost": round(time.time() - start, 2)}
            except Exception as exc:
                raise RuntimeError(f"响应解析失败: {data}") from exc

        executor = ThreadPoolExecutor(max_workers=max_workers)
        futures = {executor.submit(_task): i for i in range(repeat)}
        try:
            done, pending = wait(futures, return_when=FIRST_COMPLETED)
            first_future = next(iter(done))
            try:
                first_result = first_future.result()
            except Exception as exc:
                first_result = {"error": str(exc)}
            for f in pending:
                f.cancel()
            return first_result
        finally:
            try:
                executor.shutdown(wait=False, cancel_futures=True)
            except TypeError:
                executor.shutdown(wait=False)


if __name__ == "__main__":
    # try:
    #     llm = LLM()
    #     reply = llm.chat_text("Hello!")
    #     print("LLM reply:", reply)
    # except Exception as exc:
    #     print("LLM request failed:", exc)

    # # VLM 简单测试（需在 modelinfo.json 提供有效 base_url）
    # import numpy as np
    # try:
    #     # 构造一张简单的黑色图像
    #     dummy = np.zeros((224, 224, 3), dtype=np.uint8)
    #     vlm = VLM()
    #     vlm_reply = vlm.analyze(dummy, "Describe this image")
    #     print("VLM reply:", vlm_reply)
    # except Exception as exc:
    #     print("VLM request failed:", exc)

    llm1 = LLM()
    for res in llm1.chat_multi("tell me 10000 add 231238134 = ?", repeat=10):
        print(res)  # {"content": ..., "time_cost": ...} 或包含 "error"
    dummy = np.zeros((224, 224, 3), dtype=np.uint8)
    vlm1 = VLM()
    res = vlm1.analyze(dummy, "Describe this image", repeat=10)
    print(res)

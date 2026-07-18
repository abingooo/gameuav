#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
读取 prompts 目录下的模板文件，并根据 task 返回格式化后的提示词。
当前支持 detect、tasktype（文件名形如 detect_prompt.txt、tasktype_prompt.txt）。
"""

from pathlib import Path
from typing import Any


class PromptLoader:
    def __init__(self, prompts_dir: Path = None) -> None:
        self.prompts_dir = prompts_dir or Path(__file__).resolve().parents[1] / "prompts"

    def _read_template(self, filename: str) -> str:
        path = self.prompts_dir / filename
        if not path.exists():
            raise FileNotFoundError(f"prompt 模板不存在: {path}")
        return path.read_text(encoding="utf-8")

    def get(self, task: str, **kwargs: Any) -> str:
        """
        根据 task 读取并格式化提示词。
        支持占位符：{instruction}, {objects_json}，可按需扩展。
        """
        try:
            task = (task or "").lower()
            filename = f"{task}_prompt.txt"
            template = self._read_template(filename)
            return template.format(**kwargs)
        except Exception as exc:
            print("prompt load failed:", exc)
            return ""


if __name__ == "__main__":
    loader = PromptLoader()
    print(loader.get("detect", instruction="demo instruction"))
    

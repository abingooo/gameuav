#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import re
from typing import Any


class Format:
    @staticmethod
    def parse_json_block(text: str) -> Any:
        """
        从模型输出中提取 JSON：
        - 优先解析 ```json ... ``` 或 ``` ... ``` 代码块
        - 若无代码块则直接 json.loads
        """
        raw = text or ""
        block = re.search(r"```(?:json)?\s*(.*?)\s*```", raw, re.S | re.I)
        json_str = block.group(1) if block else raw
        try:
            return json.loads(json_str)
        except json.JSONDecodeError:
            return json_str

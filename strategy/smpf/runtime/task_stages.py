"""Strict decomposition of long-horizon language into ordered visual stages."""

from dataclasses import dataclass
import json
import math
import os
from typing import Tuple

import requests

from .model_defaults import DEFAULT_LLM_MODEL, resolve_llm_reasoning_effort


TASK_STAGES_SCHEMA = "smpf.task_stages.v1"


class TaskStageError(RuntimeError):
    pass


class TaskStageSchemaError(TaskStageError):
    pass


class TaskStageTransportError(TaskStageError):
    pass


@dataclass(frozen=True)
class TaskStage:
    instruction: str
    completion: str = "reach_target"

    def __post_init__(self):
        instruction = str(self.instruction or "").strip()
        if not instruction:
            raise ValueError("task stage instruction cannot be empty")
        if self.completion != "reach_target":
            raise ValueError("task stage completion must be reach_target")
        object.__setattr__(self, "instruction", instruction)


@dataclass(frozen=True)
class TaskStages:
    stages: Tuple[TaskStage, ...]

    def __post_init__(self):
        object.__setattr__(self, "stages", tuple(self.stages))
        if not 1 <= len(self.stages) <= 5:
            raise ValueError("task decomposition must contain 1 to 5 stages")


def parse_task_stages(content):
    if not isinstance(content, str) or not content.strip():
        raise TaskStageSchemaError("task stages must be a non-empty JSON string")
    raw = content.strip()
    if raw.startswith("```") or raw.endswith("```"):
        raise TaskStageSchemaError("task stages must not contain Markdown fences")
    try:
        data = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise TaskStageSchemaError("task stages are not valid JSON") from exc
    if not isinstance(data, dict) or set(data) != {"schema", "stages"}:
        raise TaskStageSchemaError("task stage root must contain exactly schema and stages")
    if data["schema"] != TASK_STAGES_SCHEMA:
        raise TaskStageSchemaError("task stage schema must be %s" % TASK_STAGES_SCHEMA)
    raw_stages = data["stages"]
    if not isinstance(raw_stages, list) or not 1 <= len(raw_stages) <= 5:
        raise TaskStageSchemaError("stages must contain 1 to 5 items")
    stages = []
    for index, raw_stage in enumerate(raw_stages):
        if not isinstance(raw_stage, dict) or set(raw_stage) != {"instruction", "completion"}:
            raise TaskStageSchemaError(
                "stages[%d] must contain exactly instruction and completion" % index
            )
        instruction = raw_stage["instruction"]
        completion = raw_stage["completion"]
        if not isinstance(instruction, str) or not instruction.strip():
            raise TaskStageSchemaError("stages[%d].instruction must be non-empty text" % index)
        if completion != "reach_target":
            raise TaskStageSchemaError("stages[%d].completion must be reach_target" % index)
        stages.append(TaskStage(instruction.strip(), completion))
    return TaskStages(tuple(stages))


def task_stage_prompt(instruction):
    return """Decompose this long-horizon indoor UAV instruction into ordered visual target stages:
{instruction}

Rules:
- Preserve the requested order and meaning.
- Each stage must name exactly one physical target that can be grounded in a camera image.
- Resolve references such as "the next" into a distinct-target instruction, for example "reach a different chair from the previous stage".
- Do not add takeoff, landing, arming, searching, or safety actions not present in the instruction.
- Use 1 to 5 stages and keep each instruction concise.
- Completion is always "reach_target"; local odometry and operator criteria decide actual completion.

Return raw JSON only with exactly this structure:
{{"schema":"smpf.task_stages.v1","stages":[{{"instruction":"reach the first chair","completion":"reach_target"}},{{"instruction":"reach a different chair from the previous stage","completion":"reach_target"}}]}}
Do not return Markdown.
""".format(instruction=str(instruction or "").strip())


class TaskStageClient:
    def __init__(
        self,
        api_key=None,
        base_url=None,
        model_id=None,
        reasoning_effort=None,
        timeout_sec=60.0,
        session=None,
    ):
        self.api_key = str(api_key or os.environ.get("SMPF_LLM_API_KEY", "")).strip()
        self.base_url = str(base_url or os.environ.get("SMPF_LLM_BASE_URL", "")).strip()
        self.model_id = str(model_id or os.environ.get("SMPF_LLM_MODEL", DEFAULT_LLM_MODEL)).strip()
        self.reasoning_effort = resolve_llm_reasoning_effort(reasoning_effort)
        self.timeout_sec = float(timeout_sec)
        if not self.api_key:
            raise ValueError("SMPF_LLM_API_KEY is required")
        if not self.base_url:
            raise ValueError("SMPF_LLM_BASE_URL is required")
        if "/chat/completions" not in self.base_url:
            self.base_url = self.base_url.rstrip("/") + "/chat/completions"
        if not math.isfinite(self.timeout_sec) or self.timeout_sec <= 0.0:
            raise ValueError("task stage timeout must be finite and positive")
        self._session = session or requests.Session()

    def decompose(self, instruction):
        payload = {
            "model": self.model_id,
            "messages": [
                {
                    "role": "system",
                    "content": "Return only the requested ordered task-stage JSON.",
                },
                {"role": "user", "content": task_stage_prompt(instruction)},
            ],
            "temperature": 0.0,
            "response_format": {"type": "json_object"},
            "reasoning_effort": self.reasoning_effort,
        }
        try:
            response = self._session.post(
                self.base_url,
                headers={
                    "Authorization": "Bearer %s" % self.api_key,
                    "Content-Type": "application/json",
                },
                json=payload,
                timeout=self.timeout_sec,
            )
        except requests.RequestException as exc:
            raise TaskStageTransportError("task stage request failed: %s" % exc) from exc
        except Exception as exc:
            raise TaskStageTransportError("task stage request failed: %s" % exc) from exc
        if int(response.status_code) != 200:
            detail = str(getattr(response, "text", ""))[:300]
            raise TaskStageTransportError(
                "task stage model returned HTTP %s: %s" % (response.status_code, detail)
            )
        try:
            data = response.json()
            content = data["choices"][0]["message"]["content"]
        except Exception as exc:
            raise TaskStageTransportError(
                "task stage response is missing choices[0].message.content"
            ) from exc
        return parse_task_stages(content)

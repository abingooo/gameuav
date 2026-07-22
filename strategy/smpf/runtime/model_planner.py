"""Strict model-planning contract with deterministic local verification."""

from dataclasses import dataclass
import json
import math
import os
from typing import Mapping, Optional, Tuple

import requests

from .contracts import ObjectSphere, ValidationResult
from .deterministic_planner import VisibilityGraphError, plan_visibility_graph
from .goal_validation import validate_goal_conditioned_polyline
from .model_defaults import (
    DEFAULT_LLM_MODEL,
    llm_sampling_parameters,
    resolve_llm_reasoning_effort,
)
from .model_trace import model_response_snapshot


PLAN_SCHEMA = "smpf.guidepoint_plan.v1"
PLAN_FRAME = "body_flu"


class ModelPlannerError(RuntimeError):
    """Base error for model transport, schema, and geometry failures."""


class ModelTransportError(ModelPlannerError):
    """The model endpoint could not return a usable completion."""


class PlanSchemaError(ModelPlannerError):
    """The model output does not match the guidepoint JSON contract."""


class PlanValidationError(ModelPlannerError):
    """The syntactically valid model plan failed deterministic geometry checks."""

    def __init__(self, message, validation_result, guidepoints_m=()):
        super().__init__(message)
        self.validation_result = validation_result
        self.guidepoints_m = tuple(guidepoints_m)


class DeterministicFallbackError(ModelPlannerError):
    """The model failed and the deterministic repair layer found no safe path."""


@dataclass(frozen=True)
class PlanningRequest:
    instruction: str
    object_spheres: Tuple[ObjectSphere, ...]
    bounds_flu_m: Optional[Mapping[str, float]] = None
    clearance_margin_m: float = 0.0
    fallback_goal_flu_m: Optional[Tuple[float, float, float]] = None
    fallback_goals_flu_m: Tuple[Tuple[float, float, float], ...] = ()
    target_sphere: Optional[ObjectSphere] = None
    min_target_standoff_m: float = 0.15
    max_target_standoff_m: float = 1.0
    min_target_progress_m: float = 0.10
    require_target_visibility: bool = True

    def __post_init__(self):
        instruction = str(self.instruction or "").strip()
        if not instruction:
            raise ValueError("planning instruction cannot be empty")
        object.__setattr__(self, "instruction", instruction)
        object.__setattr__(self, "object_spheres", tuple(self.object_spheres))
        for sphere in self.object_spheres:
            if sphere.frame_id != PLAN_FRAME:
                raise ValueError("all planning objects must use the body_flu frame")
        margin = float(self.clearance_margin_m)
        if not math.isfinite(margin) or margin < 0.0:
            raise ValueError("clearance margin must be finite and non-negative")
        object.__setattr__(self, "clearance_margin_m", margin)
        if self.target_sphere is not None:
            if self.target_sphere.frame_id != PLAN_FRAME:
                raise ValueError("planning target must use the body_flu frame")
            if not any(_same_sphere(sphere, self.target_sphere) for sphere in self.object_spheres):
                raise ValueError("planning target must also be present in object_spheres")
        minimum_standoff = float(self.min_target_standoff_m)
        maximum_standoff = float(self.max_target_standoff_m)
        minimum_progress = float(self.min_target_progress_m)
        if not all(math.isfinite(value) for value in (minimum_standoff, maximum_standoff)):
            raise ValueError("target standoff bounds must be finite")
        if minimum_standoff < 0.0 or maximum_standoff <= minimum_standoff:
            raise ValueError("target standoff band is invalid")
        if not math.isfinite(minimum_progress) or minimum_progress < 0.0:
            raise ValueError("minimum target progress must be finite and non-negative")
        object.__setattr__(self, "min_target_standoff_m", minimum_standoff)
        object.__setattr__(self, "max_target_standoff_m", maximum_standoff)
        object.__setattr__(self, "min_target_progress_m", minimum_progress)
        object.__setattr__(self, "require_target_visibility", bool(self.require_target_visibility))
        if self.fallback_goal_flu_m is not None:
            goal = tuple(float(value) for value in self.fallback_goal_flu_m)
            if len(goal) != 3 or not all(math.isfinite(value) for value in goal):
                raise ValueError("fallback goal must be a finite three-vector")
            object.__setattr__(self, "fallback_goal_flu_m", goal)
        fallback_goals = tuple(self.fallback_goals_flu_m or ())
        if len(fallback_goals) > 32:
            raise ValueError("fallback goals must contain at most 32 candidates")
        validated_goals = []
        for goal in fallback_goals:
            candidate = tuple(float(value) for value in goal)
            if len(candidate) != 3 or not all(math.isfinite(value) for value in candidate):
                raise ValueError("each fallback goal must be a finite three-vector")
            validated_goals.append(candidate)
        object.__setattr__(self, "fallback_goals_flu_m", tuple(validated_goals))


@dataclass(frozen=True)
class GuidepointPlan:
    guidepoints_m: Tuple[Tuple[float, float, float], ...]
    reasoning: str
    validation: ValidationResult
    attempts: int = 1
    planner_source: str = "llm"
    fallback_trigger: str = ""
    graph_candidate_count: int = 0
    graph_expanded_nodes: int = 0
    target_surface_distance_m: Optional[float] = None
    target_progress_m: Optional[float] = None
    target_visible: Optional[bool] = None


def _same_sphere(left, right):
    return (
        left.label == right.label
        and math.isclose(left.radius, right.radius, rel_tol=0.0, abs_tol=1e-7)
        and all(
            math.isclose(a, b, rel_tol=0.0, abs_tol=1e-7)
            for a, b in zip(left.center, right.center)
        )
    )


def _validate_request_path(points, request):
    return validate_goal_conditioned_polyline(
        points,
        request.object_spheres,
        target_sphere=request.target_sphere,
        bounds=request.bounds_flu_m,
        clearance_margin_m=request.clearance_margin_m,
        min_target_standoff_m=request.min_target_standoff_m,
        max_target_standoff_m=request.max_target_standoff_m,
        min_target_progress_m=request.min_target_progress_m,
        require_target_visibility=request.require_target_visibility,
        require_origin_start=True,
        origin_tolerance_m=0.02,
        bounds_start_index=1,
    )


def _parse_guidepoint(value, index):
    if not isinstance(value, list) or len(value) != 3:
        raise PlanSchemaError("guidepoints_m[%d] must contain exactly three numbers" % index)
    if any(isinstance(item, bool) or not isinstance(item, (int, float)) for item in value):
        raise PlanSchemaError("guidepoints_m[%d] must contain only numbers" % index)
    point = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in point):
        raise PlanSchemaError("guidepoints_m[%d] must contain finite numbers" % index)
    return point


def parse_and_validate_plan(content, request, attempts=1):
    """Parse exact JSON and verify the full guidepoint polyline locally."""
    if not isinstance(content, str) or not content.strip():
        raise PlanSchemaError("model plan must be a non-empty JSON string")
    raw = content.strip()
    if raw.startswith("```") or raw.endswith("```"):
        raise PlanSchemaError("model plan must not contain Markdown fences")
    try:
        data = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise PlanSchemaError("model plan is not valid JSON") from exc
    if not isinstance(data, dict):
        raise PlanSchemaError("model plan root must be an object")
    required_keys = {"schema", "frame", "guidepoints_m", "reasoning"}
    if set(data) != required_keys:
        raise PlanSchemaError("model plan must contain exactly %s" % sorted(required_keys))
    if data["schema"] != PLAN_SCHEMA:
        raise PlanSchemaError("model plan schema must be %s" % PLAN_SCHEMA)
    if data["frame"] != PLAN_FRAME:
        raise PlanSchemaError("model plan frame must be body_flu")
    raw_points = data["guidepoints_m"]
    if not isinstance(raw_points, list) or not 3 <= len(raw_points) <= 12:
        raise PlanSchemaError("guidepoints_m must contain 3 to 12 points")
    points = tuple(_parse_guidepoint(value, index) for index, value in enumerate(raw_points))
    reasoning = data["reasoning"]
    if not isinstance(reasoning, str) or not reasoning.strip():
        raise PlanSchemaError("reasoning must be a non-empty string")

    goal_validation = _validate_request_path(points, request)
    if not goal_validation.validation.valid:
        raise PlanValidationError(
            "model guidepoint path failed local geometry validation",
            goal_validation.validation,
            guidepoints_m=points,
        )
    return GuidepointPlan(
        points,
        reasoning.strip(),
        goal_validation.validation,
        int(attempts),
        target_surface_distance_m=goal_validation.target_surface_distance_m,
        target_progress_m=goal_validation.target_progress_m,
        target_visible=goal_validation.target_visible,
    )


def build_planning_prompt(request, correction_feedback=None):
    """Build the sole coordinate and output contract exposed to the model."""
    objects = [
        {
            "label": sphere.label,
            "role": "target" if request.target_sphere and _same_sphere(sphere, request.target_sphere) else "obstacle",
            "center_flu_m": list(sphere.center),
            "safety_radius_m": sphere.radius,
            "confidence": sphere.confidence,
        }
        for sphere in request.object_spheres
    ]
    bounds = dict(request.bounds_flu_m or {})
    target = None
    if request.target_sphere is not None:
        target = {
            "label": request.target_sphere.label,
            "center_flu_m": list(request.target_sphere.center),
            "safety_radius_m": request.target_sphere.radius,
            "min_surface_standoff_m": request.min_target_standoff_m,
            "max_surface_standoff_m": request.max_target_standoff_m,
            "min_progress_m": request.min_target_progress_m,
            "clear_line_of_sight_required": request.require_target_visibility,
        }
    feedback = correction_feedback or "none; this is the first attempt"
    return """You are the guidepoint planner for an indoor UAV.

COORDINATE CONTRACT
- All coordinates are relative to the UAV pose at observation time.
- Frame is ROS body FLU: x forward, y left, z up, meters.
- The first guidepoint is exactly [0.0, 0.0, 0.0].
- Never reinterpret the axes as camera optical coordinates.

TASK
{instruction}

TARGET TERMINAL CONTRACT
{target_json}

MODELED OBJECTS
{objects_json}

FLIGHT BOUNDS
{bounds_json}

LOCAL VERIFIER
- Every object is a solid sphere with center_flu_m and safety_radius_m.
- Every guidepoint and every complete connecting line segment must remain outside every sphere.
- Flight bounds apply to guidepoints after the current [0,0,0] start point.
- An additional clearance of {margin:.3f} m is added by the local verifier.
- Match the instruction with a short, smooth path of 3 to 12 points.
- When a target terminal contract is present, the final point must enter its
  permitted surface-standoff band and retain clear line of sight to the target.
- Do not output velocity, yaw, absolute/world coordinates, or controller commands.

PREVIOUS ATTEMPT FEEDBACK
{feedback}

Return raw JSON only, with exactly these four keys:
{{"schema":"smpf.guidepoint_plan.v1","frame":"body_flu","guidepoints_m":[[0.0,0.0,0.0],[x,y,z],[x,y,z]],"reasoning":"brief Chinese explanation"}}
""".format(
        instruction=request.instruction,
        target_json=json.dumps(target, ensure_ascii=False, separators=(",", ":")),
        objects_json=json.dumps(objects, ensure_ascii=False, separators=(",", ":")),
        bounds_json=json.dumps(bounds, ensure_ascii=False, separators=(",", ":")),
        margin=request.clearance_margin_m,
        feedback=feedback,
    )


def _validation_feedback(error):
    issues = []
    for issue in error.validation_result.issues[:12]:
        issues.append(
            {
                "kind": issue.kind,
                "point_or_segment_index": issue.index,
                "object": issue.object_label,
                "clearance_m": None
                if not math.isfinite(issue.clearance_m)
                else round(issue.clearance_m, 3),
                "message": issue.message,
            }
        )
    return json.dumps({"rejected_by_local_verifier": issues}, ensure_ascii=False, separators=(",", ":"))


class ModelPlannerClient:
    """OpenAI-compatible text client whose result cannot bypass local validation."""

    def __init__(
        self,
        api_key=None,
        base_url=None,
        model_id=None,
        reasoning_effort=None,
        timeout_sec=60.0,
        temperature=0.1,
        session=None,
    ):
        self.api_key = str(api_key or os.environ.get("SMPF_LLM_API_KEY", "")).strip()
        self.base_url = str(base_url or os.environ.get("SMPF_LLM_BASE_URL", "")).strip()
        self.model_id = str(model_id or os.environ.get("SMPF_LLM_MODEL", DEFAULT_LLM_MODEL)).strip()
        self.reasoning_effort = resolve_llm_reasoning_effort(reasoning_effort)
        self.timeout_sec = float(timeout_sec)
        self.temperature = float(temperature)
        self.raw_responses = []
        if not self.api_key:
            raise ValueError("SMPF_LLM_API_KEY is required")
        if not self.base_url:
            raise ValueError("SMPF_LLM_BASE_URL is required")
        if "/chat/completions" not in self.base_url:
            self.base_url = self.base_url.rstrip("/") + "/chat/completions"
        if not self.model_id:
            raise ValueError("SMPF_LLM_MODEL cannot be empty")
        if not math.isfinite(self.timeout_sec) or self.timeout_sec <= 0.0:
            raise ValueError("model timeout must be finite and positive")
        self._session = session or requests.Session()

    def plan(self, request, max_attempts=2, enable_deterministic_fallback=True):
        max_attempts = int(max_attempts)
        if max_attempts < 1 or max_attempts > 3:
            raise ValueError("max_attempts must be in [1, 3]")
        feedback = None
        last_error = None
        last_invalid_points = ()
        self.raw_responses = []
        for attempt in range(1, max_attempts + 1):
            content = self._complete(build_planning_prompt(request, feedback))
            try:
                return parse_and_validate_plan(content, request, attempts=attempt)
            except PlanValidationError as exc:
                last_error = exc
                last_invalid_points = exc.guidepoints_m
                feedback = _validation_feedback(exc)
            except PlanSchemaError as exc:
                last_error = exc
                feedback = json.dumps(
                    {"rejected_by_schema_validator": str(exc)},
                    ensure_ascii=False,
                    separators=(",", ":"),
                )
        if not enable_deterministic_fallback:
            raise last_error
        fallback_goals = []
        if last_invalid_points:
            fallback_goals.append((last_invalid_points[-1], "llm_path_validation"))
        if request.fallback_goal_flu_m is not None:
            fallback_goals.append((request.fallback_goal_flu_m, type(last_error).__name__))
        fallback_goals.extend(
            (goal, "target_approach_candidate_%d" % index)
            for index, goal in enumerate(request.fallback_goals_flu_m)
        )

        failures = []
        used_goals = set()
        for goal, trigger in fallback_goals:
            goal_key = tuple(round(float(value), 8) for value in goal)
            if goal_key in used_goals:
                continue
            used_goals.add(goal_key)
            try:
                fallback = plan_visibility_graph(
                    (0.0, 0.0, 0.0),
                    goal,
                    request.object_spheres,
                    bounds=request.bounds_flu_m,
                    clearance_margin_m=request.clearance_margin_m,
                )
            except VisibilityGraphError as exc:
                failures.append(str(exc))
                continue
            goal_validation = _validate_request_path(fallback.guidepoints_m, request)
            if not goal_validation.validation.valid:
                kinds = sorted({issue.kind for issue in goal_validation.validation.issues})
                failures.append("fallback failed goal validation: %s" % ",".join(kinds))
                continue
            return GuidepointPlan(
                fallback.guidepoints_m,
                "LLM plan rejected; deterministic visibility-graph repair selected a verified path.",
                goal_validation.validation,
                attempts=max_attempts,
                planner_source="visibility_graph_fallback",
                fallback_trigger=trigger,
                graph_candidate_count=fallback.candidate_count,
                graph_expanded_nodes=fallback.expanded_nodes,
                target_surface_distance_m=goal_validation.target_surface_distance_m,
                target_progress_m=goal_validation.target_progress_m,
                target_visible=goal_validation.target_visible,
            )
        if fallback_goals:
            raise DeterministicFallbackError(
                "model plan failed and deterministic fallback found no path: %s"
                % ("; ".join(failures) or "no valid fallback goal")
            ) from last_error
        raise last_error

    def _complete(self, prompt):
        payload = {
            "model": self.model_id,
            "messages": [
                {
                    "role": "system",
                    "content": "Return only the requested JSON. Coordinate and safety contracts are mandatory.",
                },
                {"role": "user", "content": prompt},
            ],
            "response_format": {"type": "json_object"},
            "reasoning_effort": self.reasoning_effort,
        }
        payload.update(llm_sampling_parameters(self.model_id, self.temperature))
        headers = {
            "Authorization": "Bearer %s" % self.api_key,
            "Content-Type": "application/json",
        }
        try:
            response = self._session.post(
                self.base_url,
                headers=headers,
                json=payload,
                timeout=self.timeout_sec,
            )
        except requests.RequestException as exc:
            raise ModelTransportError("model request failed: %s" % exc) from exc
        except Exception as exc:
            raise ModelTransportError("model request failed: %s" % exc) from exc
        self.raw_responses.append(model_response_snapshot(response))
        if int(response.status_code) != 200:
            detail = str(getattr(response, "text", ""))[:300]
            raise ModelTransportError("model returned HTTP %s: %s" % (response.status_code, detail))
        try:
            data = response.json()
            content = data["choices"][0]["message"]["content"]
        except Exception as exc:
            raise ModelTransportError("model response is missing choices[0].message.content") from exc
        if not isinstance(content, str):
            raise ModelTransportError("model completion content must be text")
        return content

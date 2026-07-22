"""Canonical model settings for the active SMPF runtime."""

import os

DEFAULT_VLM_MODEL = "gemini-3.5-flash"
DEFAULT_LLM_MODEL = "gpt-5.2"
DEFAULT_LLM_REASONING_EFFORT = "low"
SUPPORTED_LLM_REASONING_EFFORTS = frozenset(("minimal", "low", "medium", "high"))


def resolve_llm_reasoning_effort(value=None):
    raw = (
        os.environ.get("SMPF_LLM_REASONING_EFFORT", DEFAULT_LLM_REASONING_EFFORT)
        if value is None
        else value
    )
    effort = str(raw or "").strip().lower()
    if effort not in SUPPORTED_LLM_REASONING_EFFORTS:
        raise ValueError(
            "SMPF_LLM_REASONING_EFFORT must be one of %s"
            % sorted(SUPPORTED_LLM_REASONING_EFFORTS)
        )
    return effort


def llm_sampling_parameters(model_id, temperature):
    """Return optional sampling parameters accepted by the selected LLM."""
    normalized_model = str(model_id or "").strip().lower().rsplit("/", 1)[-1]
    if normalized_model.startswith("gpt-5") or normalized_model.startswith("kimi-k3"):
        return {}
    return {"temperature": float(temperature)}

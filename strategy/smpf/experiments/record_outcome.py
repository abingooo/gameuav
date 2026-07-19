#!/usr/bin/env python3

import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time


METHODS = {"spf", "smpf"}
MANIFESTS = {
    "real_world": Path(__file__).with_name("spf_realworld_tasks.json"),
    "simulation": Path(__file__).with_name("spf_simulation_tasks.json"),
}
PROFILE_PATH = Path(__file__).with_name("comparison_profile.json")
FORMAL_ENVIRONMENT = "real_world"
FORMAL_OUTCOME_SCHEMA = "gameuav.spf_comparison.outcome.v3"
METHOD_ORDERS = {
    "spf_then_smpf": ("spf", "smpf"),
    "smpf_then_spf": ("smpf", "spf"),
}
VARIANTS = {
    "spf": {"spf_adaptive"},
    "smpf": {
        "smpf_full",
        "smpf_llm_only",
        "smpf_no_corridor_filter",
        "smpf_no_goal_contract",
        "smpf_bounded_follow_goal",
        "smpf_no_target_identity",
    },
}
OUTCOMES = {
    "success",
    "collision",
    "target_not_visible",
    "target_out_of_range",
    "task_not_completed",
    "timeout",
    "aborted",
    "error",
}


def load_task_manifest(environment):
    try:
        path = MANIFESTS[environment]
    except KeyError as exc:
        raise ValueError("environment must be real_world or simulation") from exc
    manifest = json.loads(path.read_text(encoding="utf-8"))
    if manifest.get("schema") != "gameuav.spf_comparison.tasks.v1":
        raise ValueError("unsupported task manifest schema")
    if manifest.get("environment") != environment:
        raise ValueError("task manifest environment mismatch")
    return manifest


def load_comparison_profile(path=PROFILE_PATH):
    profile = json.loads(Path(path).read_text(encoding="utf-8"))
    if profile.get("schema") != "gameuav.spf_smpf.comparison_profile.v1":
        raise ValueError("unsupported comparison profile schema")
    if not str(profile.get("protocol_id", "")).strip():
        raise ValueError("comparison profile is missing protocol_id")
    return profile


def canonical_pair_id(task_id, repetition):
    return "real_world:%s:r%02d" % (task_id, int(repetition))


def trial_identity(record):
    """Return the identity whose uniqueness is required in an outcome file."""
    try:
        repetition = int(record["repetition"])
        return (
            str(record.get("environment", FORMAL_ENVIRONMENT)),
            str(record["method"]),
            str(record["task_id"]),
            repetition,
        )
    except (KeyError, TypeError, ValueError):
        return None


def method_profile_snapshot(profile, method, variant):
    method_profile = profile.get("methods", {}).get(variant)
    if not isinstance(method_profile, dict):
        raise ValueError("variant is missing from comparison profile: %s" % variant)
    if method_profile.get("method") != method:
        raise ValueError("comparison profile variant/method mismatch")
    required = (
        "visual_model",
        "visual_api_backend",
        "visual_api_protocol",
        "image_topic_current_default",
    )
    missing = [name for name in required if not str(method_profile.get(name, "")).strip()]
    if missing:
        raise ValueError("comparison method profile is missing: %s" % ", ".join(missing))
    return {
        "variant": variant,
        "operational_mode": method_profile.get("operational_mode"),
        "visual_model": method_profile["visual_model"],
        "visual_api_backend": method_profile["visual_api_backend"],
        "visual_api_protocol": method_profile["visual_api_protocol"],
        "planning_model": method_profile.get("planning_model"),
        "planning_reasoning_effort": method_profile.get("planning_reasoning_effort"),
        "planning_api_backend": method_profile.get("planning_api_backend"),
        "planning_api_protocol": method_profile.get("planning_api_protocol"),
        "image_topic": method_profile["image_topic_current_default"],
    }


def _optional_bool(args, name):
    value = getattr(args, name, None)
    return None if value is None else bool(value)


def _required_text(args, name, option):
    value = str(getattr(args, name, "") or "").strip()
    if not value:
        raise ValueError("%s is required for a formal real-world outcome" % option)
    return value


def _positive_finite(value, option):
    if value is None or not math.isfinite(value) or value <= 0.0:
        raise ValueError("%s must be finite and positive" % option)
    return float(value)


def _positive_integer(value, option):
    if isinstance(value, bool):
        raise ValueError("%s must be a positive integer" % option)
    try:
        integer = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("%s must be a positive integer" % option) from exc
    if value is None or integer != value or integer <= 0:
        raise ValueError("%s must be a positive integer" % option)
    return integer


def _target_distance_valid(distance_m, rule):
    if distance_m is None:
        return False
    minimum = float(rule["minimum"])
    maximum = float(rule["maximum"])
    return minimum <= distance_m <= maximum


def _formal_protocol_fields(args, task, profile, method, variant, repetition):
    primary_variants = profile.get("primary_scope", {}).get("method_variants", {})
    if primary_variants.get(method) != variant:
        raise ValueError(
            "formal real-world outcomes require primary variant %s for method %s"
            % (primary_variants.get(method), method)
        )
    if not bool(getattr(args, "confirm_fixed_profile", False)):
        raise ValueError(
            "--confirm-fixed-profile is required after checking the live model/API/topic configuration"
        )
    method_order = str(getattr(args, "method_order", "") or "").strip()
    if method_order not in METHOD_ORDERS:
        raise ValueError("--method-order must be spf_then_smpf or smpf_then_spf")
    pair_id = canonical_pair_id(task["id"], repetition)
    supplied_pair_id = str(getattr(args, "pair_id", "") or "").strip()
    if supplied_pair_id and supplied_pair_id != pair_id:
        raise ValueError("--pair-id must equal the canonical pair id %s" % pair_id)
    scene_id = _required_text(args, "scene_id", "--scene-id")
    start_pose_id = _required_text(args, "start_pose_id", "--start-pose-id")
    timeout_sec = _positive_finite(
        getattr(args, "trial_timeout_sec", None), "--trial-timeout-sec"
    )
    image_width = _positive_integer(getattr(args, "image_width", None), "--image-width")
    image_height = _positive_integer(getattr(args, "image_height", None), "--image-height")
    method_profile = method_profile_snapshot(profile, method, variant)
    return {
        "formal_protocol": True,
        "protocol_id": profile["protocol_id"],
        "pair_id": pair_id,
        "method_order": method_order,
        "method_order_index": METHOD_ORDERS[method_order].index(method) + 1,
        "scene_id": scene_id,
        "start_pose_id": start_pose_id,
        "trial_timeout_sec": timeout_sec,
        "fixed_profile_verified": True,
        "method_profile": method_profile,
        "image_input": {
            "topic": method_profile["image_topic"],
            "width": image_width,
            "height": image_height,
        },
    }


def build_record(args, task_manifest=None, comparison_profile=None):
    if args.method not in METHODS:
        raise ValueError("method must be spf or smpf")
    if args.outcome not in OUTCOMES:
        raise ValueError("unsupported outcome")
    environment = getattr(args, "environment", FORMAL_ENVIRONMENT)
    manifest = task_manifest or load_task_manifest(environment)
    if manifest.get("environment", environment) != environment:
        raise ValueError("task manifest environment mismatch")
    task_by_id = {task["id"]: task for task in manifest.get("tasks", [])}
    try:
        task = task_by_id[args.task_id]
    except KeyError as exc:
        raise ValueError("task_id is not present in the %s author manifest" % environment) from exc
    variant = getattr(args, "variant", None) or (
        "spf_adaptive" if args.method == "spf" else "smpf_full"
    )
    if variant not in VARIANTS[args.method]:
        raise ValueError("variant is not valid for method %s" % args.method)
    if args.repetition < 1 or args.repetition > 5:
        raise ValueError("repetition must be in [1, 5]")
    final_target_distance_m = getattr(args, "final_target_distance_m", None)
    numeric = (
        getattr(args, "duration_sec", None),
        getattr(args, "path_length_m", None),
        getattr(args, "api_calls", None),
        final_target_distance_m,
    )
    if any(value is not None and (not math.isfinite(value) or value < 0.0) for value in numeric):
        raise ValueError("numeric metrics must be finite and non-negative")
    task_completed = _optional_bool(args, "task_completed")
    target_visible = _optional_bool(args, "target_visible")
    distance_rule = manifest["target_distance_rule_m"]
    distance_valid = _target_distance_valid(final_target_distance_m, distance_rule)
    success_criterion_candidate = None
    if task_completed is True:
        success_criterion_candidate = "task_completed"
    elif target_visible is True and distance_valid:
        success_criterion_candidate = "target_visible_within_distance"
    if args.outcome == "success":
        if target_visible is not True:
            raise ValueError("success requires --target-visible under the paper protocol")
        if success_criterion_candidate is None:
            raise ValueError(
                "success requires --task-completed or a visible target inside the manifest distance rule"
            )
    if args.outcome == "target_not_visible" and target_visible is not False:
        raise ValueError("target_not_visible requires --target-not-visible")
    if args.outcome == "target_out_of_range":
        if target_visible is not True or final_target_distance_m is None or distance_valid:
            raise ValueError(
                "target_out_of_range requires a visible target outside the manifest distance rule"
            )
    if args.outcome == "task_not_completed" and task_completed is not False:
        raise ValueError("task_not_completed requires --task-not-completed")

    profile_fields = {
        "formal_protocol": False,
        "protocol_id": None,
        "pair_id": None,
        "method_order": None,
        "method_order_index": None,
        "scene_id": None,
        "start_pose_id": None,
        "trial_timeout_sec": None,
        "fixed_profile_verified": False,
        "method_profile": None,
        "image_input": None,
    }
    if environment == FORMAL_ENVIRONMENT:
        profile = comparison_profile or load_comparison_profile()
        profile_fields = _formal_protocol_fields(
            args, task, profile, args.method, variant, args.repetition
        )

    prompt = task["prompt"]
    return {
        "schema": FORMAL_OUTCOME_SCHEMA,
        "timestamp": time.time(),
        "method": args.method,
        "variant": variant,
        "environment": environment,
        "task_id": args.task_id,
        "category": task["category"],
        "prompt": prompt,
        "prompt_sha256": hashlib.sha256(prompt.encode("utf-8")).hexdigest(),
        "manifest_schema": manifest.get("schema"),
        "manifest_source_commit": manifest.get("source_commit"),
        "repetition": args.repetition,
        **profile_fields,
        "outcome": args.outcome,
        "success": args.outcome == "success",
        "success_criterion": (
            success_criterion_candidate if args.outcome == "success" else None
        ),
        "collision": args.outcome == "collision",
        "task_completed": task_completed,
        "target_visible": target_visible,
        "final_target_distance_m": final_target_distance_m,
        "target_distance_rule_m": distance_rule,
        "duration_sec": getattr(args, "duration_sec", None),
        "path_length_m": getattr(args, "path_length_m", None),
        "api_calls": getattr(args, "api_calls", None),
        "operator_note": getattr(args, "note", ""),
    }


def append_record(path, record):
    """Append one record while atomically rejecting duplicate trial identities."""
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    identity = trial_identity(record)
    if identity is None:
        raise ValueError("outcome record has no valid trial identity")
    with output.open("a+", encoding="utf-8") as stream:
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
        stream.seek(0)
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                continue
            try:
                existing = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError("output line %d is not valid JSON" % line_number) from exc
            if trial_identity(existing) == identity:
                raise ValueError(
                    "duplicate outcome for environment=%s method=%s task_id=%s repetition=%d"
                    % identity
                )
        stream.seek(0, 2)
        stream.write(json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n")
        stream.flush()
        os.fsync(stream.fileno())


def parser():
    result = argparse.ArgumentParser(description="Append one operator-verified SPF/SMPF trial outcome")
    result.add_argument("--method", required=True, choices=sorted(METHODS))
    result.add_argument("--variant", choices=sorted(set().union(*VARIANTS.values())))
    result.add_argument("--environment", choices=sorted(MANIFESTS), default=FORMAL_ENVIRONMENT)
    result.add_argument("--task-id", required=True)
    result.add_argument("--repetition", required=True, type=int)
    result.add_argument("--outcome", required=True, choices=sorted(OUTCOMES))
    result.add_argument("--method-order", choices=sorted(METHOD_ORDERS))
    result.add_argument("--pair-id", help="Optional assertion of the generated canonical pair ID")
    result.add_argument("--scene-id", help="Stable identifier for the paired physical scene layout")
    result.add_argument("--start-pose-id", help="Stable identifier for the paired marked start pose")
    result.add_argument("--trial-timeout-sec", type=float)
    result.add_argument("--image-width", type=int, help="Actual RGB frame width used by this method")
    result.add_argument("--image-height", type=int, help="Actual RGB frame height used by this method")
    result.add_argument(
        "--confirm-fixed-profile",
        action="store_true",
        help="Confirm the live model, API protocol, mode, and image topic match comparison_profile.json",
    )
    result.add_argument("--duration-sec", type=float)
    result.add_argument("--path-length-m", type=float)
    result.add_argument("--api-calls", type=float)
    result.add_argument("--final-target-distance-m", type=float)
    completed = result.add_mutually_exclusive_group()
    completed.add_argument("--task-completed", dest="task_completed", action="store_true")
    completed.add_argument("--task-not-completed", dest="task_completed", action="store_false")
    visible = result.add_mutually_exclusive_group()
    visible.add_argument("--target-visible", dest="target_visible", action="store_true")
    visible.add_argument("--target-not-visible", dest="target_visible", action="store_false")
    result.set_defaults(task_completed=None, target_visible=None)
    result.add_argument("--note", default="")
    result.add_argument("--output", type=Path, default=Path("runtime/spf_smpf_outcomes.jsonl"))
    return result


def main(argv=None):
    argument_parser = parser()
    args = argument_parser.parse_args(argv)
    try:
        record = build_record(args)
        append_record(args.output, record)
    except ValueError as exc:
        argument_parser.error(str(exc))
    print(json.dumps(record, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3

import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys

try:
    from .record_outcome import (
        FORMAL_ENVIRONMENT,
        FORMAL_OUTCOME_SCHEMA,
        METHOD_ORDERS,
        METHODS,
        OUTCOMES,
        PROFILE_PATH,
        canonical_pair_id,
        load_comparison_profile,
        method_profile_snapshot,
    )
except ImportError:  # Direct script execution.
    from record_outcome import (  # type: ignore
        FORMAL_ENVIRONMENT,
        FORMAL_OUTCOME_SCHEMA,
        METHOD_ORDERS,
        METHODS,
        OUTCOMES,
        PROFILE_PATH,
        canonical_pair_id,
        load_comparison_profile,
        method_profile_snapshot,
    )


SUPPORTED_OUTCOME_SCHEMAS = {
    "gameuav.spf_comparison.outcome.v1",
    "gameuav.spf_comparison.outcome.v2",
    FORMAL_OUTCOME_SCHEMA,
}
FAILURE_OUTCOMES = (
    "collision",
    "target_not_visible",
    "target_out_of_range",
    "task_not_completed",
    "timeout",
    "aborted",
    "error",
)


def load_jsonl(path):
    path = Path(path)
    if not path.exists():
        return []
    records = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        if not line.strip():
            continue
        try:
            record = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError("line %d is not valid JSON" % line_number) from exc
        if not isinstance(record, dict):
            raise ValueError("line %d must contain a JSON object" % line_number)
        if record.get("schema") not in SUPPORTED_OUTCOME_SCHEMAS:
            raise ValueError("line %d has an unsupported schema" % line_number)
        records.append(record)
    return records


def _aggregate(records, task_manifest, aggregate_category):
    manifest_task_ids = {task["id"] for task in task_manifest["tasks"]}
    categories = {task["id"]: task["category"] for task in task_manifest["tasks"]}
    categories.update({task["id"]: task["category"] for task in task_manifest.get("separate_tasks", [])})
    groups = defaultdict(list)
    for record in records:
        task_id = record["task_id"]
        if task_id not in categories:
            raise ValueError("unknown task_id: %s" % task_id)
        method = record["method"]
        variant = record.get("variant") or ("spf_adaptive" if method == "spf" else "smpf_full")
        groups[(method, variant, categories[task_id])].append(record)
        if task_id in manifest_task_ids:
            groups[(method, variant, aggregate_category)].append(record)

    summary = []
    for (method, variant, category), items in sorted(groups.items()):
        durations = [item["duration_sec"] for item in items if item.get("duration_sec") is not None]
        paths = [item["path_length_m"] for item in items if item.get("path_length_m") is not None]
        calls = [item["api_calls"] for item in items if item.get("api_calls") is not None]
        successes = sum(bool(item.get("success")) for item in items)
        summary.append(
            {
                "method": method,
                "variant": variant,
                "category": category,
                "trials": len(items),
                "successes": successes,
                "success_rate": successes / float(len(items)),
                "mean_duration_sec": statistics.mean(durations) if durations else None,
                "mean_path_length_m": statistics.mean(paths) if paths else None,
                "mean_api_calls": statistics.mean(calls) if calls else None,
                "failure_reasons": {
                    outcome: sum(item.get("outcome") == outcome for item in items)
                    for outcome in FAILURE_OUTCOMES
                },
            }
        )
    return summary


def _is_positive_number(value):
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(value)
        and value > 0
    )


def _is_positive_integer(value):
    return not isinstance(value, bool) and isinstance(value, int) and value > 0


def _record_key(record, task_ids, repetitions):
    method = record.get("method")
    task_id = record.get("task_id")
    repetition = record.get("repetition")
    if (
        method not in METHODS
        or task_id not in task_ids
        or isinstance(repetition, bool)
        or not isinstance(repetition, int)
        or repetition < 1
        or repetition > repetitions
    ):
        return None
    return (method, task_id, repetition)


def _validate_formal_record(record, task, manifest, profile):
    errors = []
    method = record.get("method")
    repetition = record.get("repetition")
    primary_variants = profile["primary_scope"]["method_variants"]
    expected_variant = primary_variants.get(method)

    if record.get("schema") != FORMAL_OUTCOME_SCHEMA:
        errors.append("legacy outcome schema cannot satisfy the formal protocol")
    if record.get("formal_protocol") is not True:
        errors.append("formal_protocol must be true")
    if record.get("environment") != FORMAL_ENVIRONMENT:
        errors.append("environment must be real_world")
    if method not in METHODS:
        errors.append("method must be spf or smpf")
    if expected_variant is None or record.get("variant") != expected_variant:
        errors.append("variant does not match the primary method profile")
    if record.get("category") != task["category"]:
        errors.append("category does not match the manifest")
    prompt = task["prompt"]
    if record.get("prompt") != prompt:
        errors.append("prompt does not exactly match the manifest")
    expected_prompt_hash = hashlib.sha256(prompt.encode("utf-8")).hexdigest()
    if record.get("prompt_sha256") != expected_prompt_hash:
        errors.append("prompt_sha256 does not match the manifest prompt")
    if record.get("manifest_schema") != manifest.get("schema"):
        errors.append("manifest_schema does not match")
    if record.get("manifest_source_commit") != manifest.get("source_commit"):
        errors.append("manifest_source_commit does not match")
    if record.get("protocol_id") != profile["protocol_id"]:
        errors.append("protocol_id does not match comparison_profile.json")
    if not isinstance(repetition, int) or isinstance(repetition, bool) or not 1 <= repetition <= 5:
        errors.append("repetition must be an integer in [1, 5]")
    else:
        if record.get("pair_id") != canonical_pair_id(task["id"], repetition):
            errors.append("pair_id is not canonical for task_id/repetition")
    method_order = record.get("method_order")
    if method_order not in METHOD_ORDERS:
        errors.append("method_order is invalid")
    elif method in METHODS:
        expected_index = METHOD_ORDERS[method_order].index(method) + 1
        if record.get("method_order_index") != expected_index:
            errors.append("method_order_index does not match method_order")
    for field in ("scene_id", "start_pose_id"):
        value = record.get(field)
        if not isinstance(value, str) or not value.strip():
            errors.append("%s must be a non-empty identifier" % field)
    if not _is_positive_number(record.get("trial_timeout_sec")):
        errors.append("trial_timeout_sec must be finite and positive")
    if record.get("fixed_profile_verified") is not True:
        errors.append("fixed_profile_verified must be true")
    if expected_variant is not None and method in METHODS:
        expected_method_profile = method_profile_snapshot(profile, method, expected_variant)
        if record.get("method_profile") != expected_method_profile:
            errors.append("method_profile does not match the fixed model/API/topic profile")
        image_input = record.get("image_input")
        if not isinstance(image_input, dict):
            errors.append("image_input must contain topic, width, and height")
        else:
            if image_input.get("topic") != expected_method_profile["image_topic"]:
                errors.append("image_input.topic does not match the fixed method topic")
            if not _is_positive_integer(image_input.get("width")):
                errors.append("image_input.width must be a positive integer")
            if not _is_positive_integer(image_input.get("height")):
                errors.append("image_input.height must be a positive integer")
    outcome = record.get("outcome")
    if outcome not in OUTCOMES:
        errors.append("outcome is unsupported")
    if record.get("success") is not (outcome == "success"):
        errors.append("success must be derived from outcome")
    if record.get("collision") is not (outcome == "collision"):
        errors.append("collision must be derived from outcome")
    if not _is_positive_number(record.get("timestamp")):
        errors.append("timestamp must be finite and positive")
    return errors


def _descriptor(key):
    method, task_id, repetition = key
    return {"method": method, "task_id": task_id, "repetition": repetition}


def _formal_summary(records, task_manifest, profile):
    tasks = task_manifest.get("tasks", [])
    task_by_id = {task["id"]: task for task in tasks}
    task_ids = set(task_by_id)
    repetitions = task_manifest.get("repetitions_per_task")
    primary_scope = profile.get("primary_scope", {})
    primary_variants = primary_scope.get("method_variants", {})
    expected_keys = {
        (method, task["id"], repetition)
        for method in sorted(METHODS)
        for task in tasks
        for repetition in range(1, int(repetitions or 0) + 1)
    }

    scope_errors = []
    if task_manifest.get("schema") != "gameuav.spf_comparison.tasks.v1":
        scope_errors.append("unsupported task manifest schema")
    if task_manifest.get("environment") != FORMAL_ENVIRONMENT:
        scope_errors.append("formal summary requires the real_world manifest")
    if len(tasks) != primary_scope.get("task_count"):
        scope_errors.append("manifest task count does not match comparison profile")
    if repetitions != primary_scope.get("repetitions_per_task"):
        scope_errors.append("manifest repetition count does not match comparison profile")
    if set(primary_variants) != METHODS:
        scope_errors.append("comparison profile must define exactly SPF and SMPF")
    if len(expected_keys) != primary_scope.get("expected_trials"):
        scope_errors.append("computed trial count does not match comparison profile")
    if primary_scope.get("expected_trials") != 110:
        scope_errors.append("formal real-world protocol must declare expected_trials=110")

    raw_by_key = defaultdict(list)
    valid_by_key = defaultdict(list)
    record_errors = []
    for record_index, record in enumerate(records, start=1):
        key = _record_key(record, task_ids, int(repetitions or 0))
        if key is not None:
            raw_by_key[key].append(record_index)
            task = task_by_id[key[1]]
            errors = _validate_formal_record(record, task, task_manifest, profile)
            if not errors:
                valid_by_key[key].append((record_index, record))
        else:
            errors = ["record does not identify an expected method/task/repetition"]
        if errors:
            record_errors.append(
                {
                    "record_index": record_index,
                    "identity": None if key is None else _descriptor(key),
                    "errors": errors,
                }
            )

    duplicates = [
        {**_descriptor(key), "count": len(indices), "record_indices": indices}
        for key, indices in sorted(raw_by_key.items())
        if len(indices) > 1
    ]
    missing = [
        _descriptor(key)
        for key in sorted(expected_keys)
        if not valid_by_key.get(key)
    ]
    accepted = {
        key: items[0][1]
        for key, items in valid_by_key.items()
        if len(items) == 1 and len(raw_by_key[key]) == 1
    }

    pair_errors = []
    method_order_counts = Counter()
    for task in tasks:
        for repetition in range(1, int(repetitions or 0) + 1):
            spf = accepted.get(("spf", task["id"], repetition))
            smpf = accepted.get(("smpf", task["id"], repetition))
            if spf is None or smpf is None:
                continue
            mismatched = [
                field
                for field in (
                    "pair_id",
                    "method_order",
                    "scene_id",
                    "start_pose_id",
                    "trial_timeout_sec",
                )
                if spf.get(field) != smpf.get(field)
            ]
            if mismatched:
                pair_errors.append(
                    {
                        "pair_id": canonical_pair_id(task["id"], repetition),
                        "task_id": task["id"],
                        "repetition": repetition,
                        "errors": ["paired records disagree on: %s" % ", ".join(mismatched)],
                    }
                )
            else:
                method_order_counts[spf["method_order"]] += 1

    resolution_errors = []
    resolutions = {}
    for method in sorted(METHODS):
        method_resolutions = sorted(
            {
                (record["image_input"]["width"], record["image_input"]["height"])
                for key, record in accepted.items()
                if key[0] == method
            }
        )
        resolutions[method] = [
            {"width": width, "height": height} for width, height in method_resolutions
        ]
        if len(method_resolutions) > 1:
            resolution_errors.append(
                "%s uses multiple image resolutions: %s" % (method, method_resolutions)
            )

    protocol_errors = {
        "scope": scope_errors,
        "records": record_errors,
        "pairs": pair_errors,
        "resolution": resolution_errors,
    }
    has_protocol_errors = any(protocol_errors.values())
    complete = (
        len(expected_keys) == 110
        and len(records) == 110
        and len(accepted) == 110
        and not missing
        and not duplicates
        and not has_protocol_errors
    )
    aggregate_category = "all_realworld"
    return {
        "schema": "gameuav.spf_comparison.summary.v2",
        "protocol_id": profile["protocol_id"],
        "coverage": {
            "expected": 110,
            "observed": len(records),
            "accepted_unique": len(accepted),
            "missing": missing,
            "missing_count": len(missing),
            "duplicates": duplicates,
            "duplicate_identity_count": len(duplicates),
            "complete": complete,
        },
        "protocol_errors": protocol_errors,
        "pairing": {
            "expected_pairs": 55,
            "complete_valid_pairs": sum(method_order_counts.values()),
            "method_order_counts": {
                order: method_order_counts[order] for order in sorted(METHOD_ORDERS)
            },
        },
        "image_resolutions_by_method": resolutions,
        "camera_input_limitation": profile["camera_input_policy"],
        "groups": _aggregate(list(accepted.values()), task_manifest, aggregate_category),
    }


def summarize(records, task_manifest, comparison_profile=None):
    environment = task_manifest.get("environment", FORMAL_ENVIRONMENT)
    if environment == FORMAL_ENVIRONMENT:
        profile = comparison_profile or load_comparison_profile()
        return _formal_summary(records, task_manifest, profile)
    if environment != "simulation":
        raise ValueError("unsupported task manifest environment: %s" % environment)
    for record in records:
        record_environment = record.get("environment")
        if record_environment is not None and record_environment != environment:
            raise ValueError("record environment does not match task manifest")
    return {
        "schema": "gameuav.spf_comparison.summary.v1",
        "groups": _aggregate(records, task_manifest, "all_simulation"),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description="Summarize operator-verified SPF/SMPF outcomes")
    parser.add_argument("outcomes", type=Path)
    parser.add_argument(
        "--tasks",
        type=Path,
        default=Path(__file__).with_name("spf_realworld_tasks.json"),
    )
    parser.add_argument("--profile", type=Path, default=PROFILE_PATH)
    args = parser.parse_args(argv)
    tasks = json.loads(args.tasks.read_text(encoding="utf-8"))
    profile = load_comparison_profile(args.profile) if tasks.get("environment") == FORMAL_ENVIRONMENT else None
    summary = summarize(load_jsonl(args.outcomes), tasks, profile)
    print(json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True))
    if tasks.get("environment") == FORMAL_ENVIRONMENT and not summary["coverage"]["complete"]:
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())

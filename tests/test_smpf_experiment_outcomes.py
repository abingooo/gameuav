import argparse
from collections import Counter
import copy
import json
from pathlib import Path
import tempfile
import unittest

from strategy.smpf.experiments.record_outcome import (
    append_record,
    build_record,
    canonical_pair_id,
)
from strategy.smpf.experiments.summarize_outcomes import load_jsonl, summarize


ROOT = Path(__file__).resolve().parents[1]
EXPERIMENTS = ROOT / "strategy/smpf/experiments"


def load_realworld_manifest():
    return json.loads((EXPERIMENTS / "spf_realworld_tasks.json").read_text())


def load_profile():
    return json.loads((EXPERIMENTS / "comparison_profile.json").read_text())


def formal_args(
    method="smpf",
    task_id="nav_chair_long",
    repetition=1,
    outcome="success",
    method_order="spf_then_smpf",
):
    return argparse.Namespace(
        method=method,
        variant="spf_adaptive" if method == "spf" else "smpf_full",
        environment="real_world",
        task_id=task_id,
        repetition=repetition,
        outcome=outcome,
        method_order=method_order,
        pair_id=None,
        scene_id="scene-%s-r%d" % (task_id, repetition),
        start_pose_id="start-%s-r%d" % (task_id, repetition),
        trial_timeout_sec=300.0,
        image_width=1280 if method == "spf" else 640,
        image_height=720 if method == "spf" else 480,
        confirm_fixed_profile=True,
        task_completed=True,
        target_visible=True,
        final_target_distance_m=None,
        duration_sec=10.0,
        path_length_m=3.0,
        api_calls=3.0,
        note="",
    )


def full_formal_records():
    manifest = load_realworld_manifest()
    records = []
    for task_index, task in enumerate(manifest["tasks"]):
        for repetition in range(1, 6):
            order = (
                "spf_then_smpf"
                if (task_index + repetition) % 2
                else "smpf_then_spf"
            )
            for method in ("spf", "smpf"):
                records.append(
                    build_record(
                        formal_args(
                            method=method,
                            task_id=task["id"],
                            repetition=repetition,
                            method_order=order,
                        ),
                        task_manifest=manifest,
                        comparison_profile=load_profile(),
                    )
                )
    return records


class SmpfExperimentOutcomesTest(unittest.TestCase):
    def test_manifest_and_profile_fix_the_110_trial_realworld_scope(self):
        manifest = load_realworld_manifest()
        profile = load_profile()
        self.assertEqual(len(manifest["tasks"]), 11)
        self.assertEqual(manifest["repetitions_per_task"], 5)
        self.assertTrue(manifest["search_reported_separately"])
        self.assertEqual(manifest["environment"], "real_world")
        self.assertEqual(manifest["separate_tasks"], [])
        self.assertEqual(
            Counter(task["category"] for task in manifest["tasks"]),
            Counter(
                {
                    "navigation": 1,
                    "obstacle": 2,
                    "long_horizon": 2,
                    "reasoning": 4,
                    "follow": 2,
                }
            ),
        )
        self.assertEqual(profile["primary_scope"]["task_count"], len(manifest["tasks"]))
        self.assertEqual(profile["primary_scope"]["expected_pairs"], 55)
        self.assertEqual(profile["primary_scope"]["expected_trials"], 110)
        self.assertEqual(
            profile["primary_scope"]["method_variants"],
            {"spf": "spf_adaptive", "smpf": "smpf_full"},
        )
        self.assertEqual(profile["primary_scope"]["scope_policy"], "real_world_only")
        self.assertFalse(profile["primary_scope"]["search_included"])
        self.assertEqual(
            set(profile["primary_scope"]["categories"]),
            {"navigation", "obstacle", "long_horizon", "reasoning", "follow"},
        )
        self.assertEqual(profile["methods"]["spf_adaptive"]["operational_mode"], "adaptive_mode")
        self.assertFalse(profile["methods"]["spf_adaptive"]["ego_occupancy_projection"])
        self.assertEqual(
            profile["methods"]["smpf_full"]["planning_model"],
            "gpt-5.2",
        )

    def test_simulation_manifest_matches_all_twenty_three_paper_tasks(self):
        manifest = json.loads((EXPERIMENTS / "spf_simulation_tasks.json").read_text())
        self.assertEqual(manifest["environment"], "simulation")
        self.assertEqual(manifest["author_platform"], "DRL Simulator")
        self.assertEqual(manifest["repetitions_per_task"], 5)
        self.assertEqual(len(manifest["tasks"]), 23)
        profile = load_profile()
        self.assertEqual(profile["separate_scope"]["task_count"], len(manifest["tasks"]))
        self.assertFalse(profile["separate_scope"]["follow_included"])
        self.assertEqual(
            profile["separate_scope"]["local_status"],
            "out_of_scope_real_world_only",
        )
        self.assertEqual(
            Counter(task["category"] for task in manifest["tasks"]),
            Counter(
                {
                    "navigation": 5,
                    "obstacle": 5,
                    "long_horizon": 5,
                    "reasoning": 3,
                    "search": 5,
                }
            ),
        )

    def test_published_manifests_cover_six_categories_without_invented_tasks(self):
        manifests = [
            json.loads((EXPERIMENTS / filename).read_text())
            for filename in ("spf_realworld_tasks.json", "spf_simulation_tasks.json")
        ]
        task_ids = [task["id"] for manifest in manifests for task in manifest["tasks"]]
        categories = {
            task["category"] for manifest in manifests for task in manifest["tasks"]
        }
        self.assertEqual(len(task_ids), len(set(task_ids)))
        self.assertEqual(
            categories,
            {"navigation", "obstacle", "long_horizon", "reasoning", "search", "follow"},
        )
        self.assertEqual(set(load_profile()["category_to_smpf_mode"]), categories)
        self.assertFalse(
            any(
                task["id"] == "search_out_of_view"
                for manifest in manifests
                for task in manifest["tasks"]
            )
        )

    def test_formal_record_embeds_manifest_pair_and_fixed_method_profile(self):
        args = formal_args(method="smpf")
        record = build_record(args)
        self.assertTrue(record["success"])
        self.assertEqual(record["schema"], "gameuav.spf_comparison.outcome.v3")
        self.assertEqual(record["category"], "navigation")
        self.assertEqual(record["success_criterion"], "task_completed")
        self.assertEqual(record["pair_id"], "real_world:nav_chair_long:r01")
        self.assertEqual(record["method_order_index"], 2)
        self.assertEqual(record["method_profile"]["visual_model"], "gemini-3.5-flash")
        self.assertEqual(record["method_profile"]["planning_model"], "gpt-5.2")
        self.assertEqual(record["method_profile"]["planning_reasoning_effort"], "low")
        self.assertEqual(record["method_profile"]["visual_api_protocol"], "chat.completions")
        self.assertEqual(record["image_input"], {
            "topic": "/camera/color/image_raw",
            "width": 640,
            "height": 480,
        })
        self.assertEqual(len(record["prompt_sha256"]), 64)
        args.outcome = "collision"
        self.assertFalse(build_record(args)["success"])

    def test_formal_record_requires_all_protocol_controls_and_confirmation(self):
        required = (
            ("method_order", None),
            ("scene_id", ""),
            ("start_pose_id", ""),
            ("trial_timeout_sec", None),
            ("image_width", None),
            ("image_height", 0),
            ("confirm_fixed_profile", False),
        )
        for field, value in required:
            with self.subTest(field=field):
                args = formal_args()
                setattr(args, field, value)
                with self.assertRaises(ValueError):
                    build_record(args)

    def test_formal_record_rejects_noncanonical_pair_and_nonprimary_variant(self):
        args = formal_args()
        args.pair_id = "pair-1"
        with self.assertRaises(ValueError):
            build_record(args)
        args = formal_args()
        args.variant = "smpf_llm_only"
        with self.assertRaises(ValueError):
            build_record(args)

    def test_target_visibility_success_obeys_manifest_distance_rule(self):
        args = formal_args(method="spf", task_id="follow_green_shirt")
        args.task_completed = None
        args.final_target_distance_m = 0.8
        self.assertEqual(
            build_record(args)["success_criterion"],
            "target_visible_within_distance",
        )
        args.target_visible = False
        with self.assertRaises(ValueError):
            build_record(args)
        args.target_visible = True
        args.final_target_distance_m = 1.2
        with self.assertRaises(ValueError):
            build_record(args)
        args.outcome = "target_out_of_range"
        self.assertFalse(build_record(args)["success"])

    def test_outcome_rejects_task_not_in_author_manifest(self):
        args = formal_args(task_id="invented_search_task")
        with self.assertRaises(ValueError):
            build_record(args)

    def test_append_rejects_duplicate_method_task_repetition(self):
        record = build_record(formal_args(method="spf"))
        duplicate = copy.deepcopy(record)
        duplicate["outcome"] = "timeout"
        duplicate["success"] = False
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "outcomes.jsonl"
            append_record(output, record)
            with self.assertRaisesRegex(ValueError, "duplicate outcome"):
                append_record(output, duplicate)
            self.assertEqual(len(output.read_text().splitlines()), 1)

    def test_empty_summary_reports_all_110_missing_and_not_complete(self):
        summary = summarize([], load_realworld_manifest(), load_profile())
        coverage = summary["coverage"]
        self.assertEqual(coverage["expected"], 110)
        self.assertEqual(coverage["observed"], 0)
        self.assertEqual(coverage["missing_count"], 110)
        self.assertEqual(coverage["duplicates"], [])
        self.assertFalse(coverage["complete"])

    def test_missing_outcome_file_loads_as_empty_dataset(self):
        with tempfile.TemporaryDirectory() as temporary:
            self.assertEqual(load_jsonl(Path(temporary) / "outcomes.jsonl"), [])

    def test_complete_summary_requires_all_110_valid_records_and_55_pairs(self):
        records = full_formal_records()
        summary = summarize(records, load_realworld_manifest(), load_profile())
        coverage = summary["coverage"]
        self.assertEqual(coverage["expected"], 110)
        self.assertEqual(coverage["observed"], 110)
        self.assertEqual(coverage["accepted_unique"], 110)
        self.assertEqual(coverage["missing"], [])
        self.assertEqual(coverage["duplicates"], [])
        self.assertTrue(coverage["complete"])
        self.assertEqual(summary["pairing"]["complete_valid_pairs"], 55)
        self.assertGreater(summary["pairing"]["method_order_counts"]["spf_then_smpf"], 0)
        self.assertGreater(summary["pairing"]["method_order_counts"]["smpf_then_spf"], 0)
        self.assertEqual(
            summary["camera_input_limitation"]["interpretation"],
            "not_a_camera_controlled_or_pure_planner_comparison",
        )
        aggregates = [item for item in summary["groups"] if item["category"] == "all_realworld"]
        self.assertEqual({item["method"] for item in aggregates}, {"spf", "smpf"})
        self.assertTrue(all(item["trials"] == 55 for item in aggregates))

    def test_missing_record_cannot_be_reported_complete(self):
        records = full_formal_records()[:-1]
        summary = summarize(records, load_realworld_manifest(), load_profile())
        self.assertEqual(summary["coverage"]["observed"], 109)
        self.assertEqual(summary["coverage"]["missing_count"], 1)
        self.assertFalse(summary["coverage"]["complete"])

    def test_summary_reports_manually_introduced_duplicate(self):
        records = full_formal_records()
        records.append(copy.deepcopy(records[0]))
        summary = summarize(records, load_realworld_manifest(), load_profile())
        self.assertEqual(summary["coverage"]["observed"], 111)
        self.assertEqual(summary["coverage"]["duplicate_identity_count"], 1)
        self.assertEqual(summary["coverage"]["duplicates"][0]["count"], 2)
        self.assertFalse(summary["coverage"]["complete"])

    def test_summary_rejects_pair_control_mismatch_and_resolution_drift(self):
        records = full_formal_records()
        records[1]["scene_id"] = "different-scene"
        records[3]["image_input"]["width"] = 800
        summary = summarize(records, load_realworld_manifest(), load_profile())
        self.assertEqual(summary["coverage"]["observed"], 110)
        self.assertEqual(summary["coverage"]["missing"], [])
        self.assertTrue(summary["protocol_errors"]["pairs"])
        self.assertTrue(summary["protocol_errors"]["resolution"])
        self.assertFalse(summary["coverage"]["complete"])

    def test_legacy_or_tampered_record_does_not_fill_a_formal_slot(self):
        records = full_formal_records()
        records[0]["schema"] = "gameuav.spf_comparison.outcome.v2"
        records[1]["prompt"] = "Fly somewhere else"
        summary = summarize(records, load_realworld_manifest(), load_profile())
        self.assertEqual(summary["coverage"]["missing_count"], 2)
        self.assertEqual(len(summary["protocol_errors"]["records"]), 2)
        self.assertFalse(summary["coverage"]["complete"])

    def test_simulation_summary_remains_available_as_reference(self):
        manifest = {
            "environment": "simulation",
            "tasks": [{"id": "search", "category": "search"}],
            "separate_tasks": [],
        }
        records = [
            {
                "method": "spf",
                "variant": "spf_adaptive",
                "environment": "simulation",
                "task_id": "search",
                "success": True,
                "outcome": "success",
                "duration_sec": 12.0,
                "path_length_m": 4.0,
                "api_calls": 3.0,
            }
        ]
        groups = summarize(records, manifest)["groups"]
        aggregate = next(item for item in groups if item["category"] == "all_simulation")
        self.assertEqual(aggregate["trials"], 1)
        self.assertFalse(any(item["category"] == "all_realworld" for item in groups))


if __name__ == "__main__":
    unittest.main()

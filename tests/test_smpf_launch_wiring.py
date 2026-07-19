from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


class SmpfLaunchWiringTest(unittest.TestCase):
    def _root(self, relative_path):
        return ET.parse(str(ROOT / relative_path)).getroot()

    def test_realsense_exposes_alignment_and_sync(self):
        root = self._root("launch/bringup_realsense.launch")
        defaults = {node.attrib["name"]: node.attrib.get("default") for node in root.findall("arg")}
        self.assertEqual(defaults["align_depth"], "false")
        self.assertEqual(defaults["enable_sync"], "false")
        include_args = {node.attrib["name"]: node.attrib.get("value") for node in root.find("include").findall("arg")}
        self.assertEqual(include_args["align_depth"], "$(arg align_depth)")
        self.assertEqual(include_args["enable_sync"], "$(arg enable_sync)")

    def test_alignment_options_reach_realflight_entrypoint(self):
        root = self._root("launch/bringup_realflight.launch")
        defaults = {node.attrib["name"]: node.attrib.get("default") for node in root.findall("arg")}
        names = set(defaults)
        self.assertIn("realsense_align_depth", names)
        self.assertIn("realsense_enable_sync", names)
        self.assertEqual(defaults["realsense_align_depth"], "true")
        self.assertEqual(defaults["realsense_enable_sync"], "true")
        self.assertEqual(defaults["use_smpf"], "true")
        self.assertEqual(defaults["smpf_execution_enabled"], "false")
        self.assertEqual(
            defaults["smpf_llm_reasoning_effort"],
            "$(optenv SMPF_LLM_REASONING_EFFORT low)",
        )
        flight_include = root.find("include")
        include_args = {node.attrib["name"]: node.attrib.get("value") for node in flight_include.findall("arg")}
        self.assertEqual(include_args["realsense_align_depth"], "$(arg realsense_align_depth)")
        self.assertEqual(include_args["realsense_enable_sync"], "$(arg realsense_enable_sync)")
        smpf_include = next(
            group.find("include")
            for group in root.findall("group")
            if group.find("include") is not None
            and group.find("include").attrib.get("file", "").endswith("bringup_smpf.launch")
        )
        smpf_args = {
            node.attrib["name"]: node.attrib.get("value")
            for node in smpf_include.findall("arg")
        }
        self.assertEqual(
            smpf_args["llm_reasoning_effort"],
            "$(arg smpf_llm_reasoning_effort)",
        )

    def test_smpf_bridge_clears_stale_private_parameters(self):
        root = self._root("ros_nodes/mission/smpf_bridge/launch/smpf_bridge.launch")
        node = root.find("node")
        self.assertEqual(node.attrib.get("clear_params"), "true")
        names = {param.attrib["name"] for param in node.findall("param")}
        self.assertNotIn("experiment_log_path", names)
        values = {param.attrib["name"]: param.attrib.get("value") for param in node.findall("param")}
        self.assertEqual(values["require_armed_for_execution"], "true")
        self.assertEqual(values["min_execution_z"], "0.20")
        self.assertEqual(values["goal_tolerance_yaw_deg"], "10.0")
        self.assertEqual(values["planning_yaw_topic"], "/planning/goal_yaw_deg")
        self.assertEqual(values["yaw_refresh_hz"], "2.0")
        self.assertEqual(values["memory_association_distance_m"], "0.35")
        self.assertEqual(values["dynamic_memory_association_distance_m"], "1.50")
        self.assertEqual(values["fallback_standoff_m"], "0.15")
        self.assertEqual(values["min_target_standoff_m"], "0.15")
        self.assertEqual(values["max_target_standoff_m"], "1.00")
        self.assertEqual(values["min_target_progress_m"], "0.10")
        self.assertEqual(values["require_target_visibility"], "true")
        self.assertEqual(values["max_body_camera_translation_m"], "0.75")
        self.assertEqual(values["max_realsense_extrinsic_translation_m"], "0.10")
        self.assertEqual(values["deterministic_fallback_enabled"], "true")
        self.assertEqual(values["completed_target_exclusion_enabled"], "true")
        self.assertEqual(values["goal_condition_validation_enabled"], "true")
        self.assertEqual(values["corridor_obstacle_filter_enabled"], "true")
        self.assertEqual(values["corridor_obstacle_margin_m"], "0.25")
        self.assertEqual(values["follow_step_limit_enabled"], "false")
        self.assertEqual(values["follow_max_step_m"], "0.50")
        self.assertEqual(values["follow_target_surface_standoff_m"], "0.15")
        self.assertEqual(values["follow_target_surface_tolerance_m"], "0.10")
        self.assertEqual(
            values["follow_metric_frame_max_age_sec"],
            "$(arg follow_metric_frame_max_age_sec)",
        )
        self.assertEqual(
            values["follow_metric_odom_skew_sec"],
            "$(arg follow_metric_odom_skew_sec)",
        )
        self.assertEqual(values["follow_sam_timeout_sec"], "$(arg follow_sam_timeout_sec)")
        self.assertEqual(values["llm_reasoning_effort"], "$(arg llm_reasoning_effort)")

        defaults = {node.attrib["name"]: node.attrib.get("default") for node in root.findall("arg")}
        self.assertEqual(defaults["follow_metric_frame_max_age_sec"], "1.0")
        self.assertEqual(defaults["follow_metric_odom_skew_sec"], "0.08")
        self.assertEqual(defaults["follow_sam_timeout_sec"], "0.75")

    def test_smpf_bringup_forwards_follow_freshness_contract(self):
        root = self._root("launch/bringup_smpf.launch")
        defaults = {node.attrib["name"]: node.attrib.get("default") for node in root.findall("arg")}
        self.assertEqual(defaults["follow_metric_frame_max_age_sec"], "1.0")
        self.assertEqual(defaults["follow_metric_odom_skew_sec"], "0.08")
        self.assertEqual(defaults["follow_sam_timeout_sec"], "0.75")
        include_args = {
            node.attrib["name"]: node.attrib.get("value")
            for node in root.find("include").findall("arg")
        }
        self.assertEqual(
            include_args["follow_metric_frame_max_age_sec"],
            "$(arg follow_metric_frame_max_age_sec)",
        )
        self.assertEqual(
            include_args["follow_metric_odom_skew_sec"],
            "$(arg follow_metric_odom_skew_sec)",
        )
        self.assertEqual(include_args["follow_sam_timeout_sec"], "$(arg follow_sam_timeout_sec)")


if __name__ == "__main__":
    unittest.main()

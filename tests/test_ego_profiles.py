from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]


def launch_root(relative_path):
    return ET.parse(str(ROOT / relative_path)).getroot()


def args_by_name(root):
    return {node.attrib["name"]: node.attrib for node in root.findall("arg")}


class EgoProfileTest(unittest.TestCase):
    def test_profile_is_forwarded_to_grid_map(self):
        bringup = launch_root("launch/bringup_ego.launch")
        self.assertEqual(args_by_name(bringup)["ego_profile"]["default"], "mapped")
        include_args = {
            node.attrib["name"]: node.attrib.get("value")
            for node in bringup.find("group").find("include").findall("arg")
        }
        self.assertEqual(include_args["ego_profile"], "$(arg ego_profile)")

        single = launch_root(
            "ros_nodes/planning/ego_planner_stack/plan_manage/launch/single_run_in_exp.launch"
        )
        self.assertEqual(args_by_name(single)["ego_profile"]["default"], "mapped")
        advanced = single.find("include")
        advanced_args = {
            node.attrib["name"]: node.attrib.get("value")
            for node in advanced.findall("arg")
        }
        self.assertEqual(advanced_args["ego_profile"], "$(arg ego_profile)")

        params = launch_root(
            "ros_nodes/planning/ego_planner_stack/plan_manage/launch/advanced_param_exp.xml"
        )
        planner = params.find("node")
        values = {
            node.attrib["name"]: node.attrib.get("value")
            for node in planner.findall("param")
        }
        self.assertEqual(values["grid_map/ego_profile"], "$(arg ego_profile)")

    def test_realflight_forwards_profile_to_ego(self):
        root = launch_root("launch/bringup_realflight.launch")
        self.assertEqual(args_by_name(root)["ego_profile"]["default"], "mapped")
        ego_include = next(
            node
            for node in root.findall("include")
            if node.attrib.get("file", "").endswith("bringup_ego.launch")
        )
        values = {
            node.attrib["name"]: node.attrib.get("value")
            for node in ego_include.findall("arg")
        }
        self.assertEqual(values["ego_profile"], "$(arg ego_profile)")

    def test_agent_exposes_two_fixed_mutually_exclusive_stacks(self):
        with (ROOT / "config/modules/uav_agent.yaml").open(encoding="utf-8") as stream:
            modules = yaml.safe_load(stream)["modules"]

        mapped = modules["egoctrl"]
        free_space = modules["egoctrl_nomap"]
        self.assertEqual(mapped["args"]["ego_profile"], "mapped")
        self.assertEqual(mapped["args"]["use_smpf"], "true")
        self.assertEqual(mapped["args"]["use_see_point_fly"], "false")
        self.assertNotIn("ego_profile", mapped["allowed_args"])
        self.assertEqual(mapped["conflicts"], ["egoctrl_nomap"])

        self.assertEqual(free_space["args"]["ego_profile"], "free_space")
        self.assertEqual(free_space["args"]["use_smpf"], "false")
        self.assertEqual(free_space["args"]["use_see_point_fly"], "false")
        self.assertNotIn("ego_profile", free_space["allowed_args"])
        self.assertEqual(free_space["conflicts"], ["egoctrl"])

    def test_free_space_grid_map_keeps_bounds_but_disables_scene_mapping(self):
        header = (
            ROOT
            / "ros_nodes/planning/ego_planner_stack/plan_env/include/plan_env/grid_map.h"
        ).read_text(encoding="utf-8")
        source = (
            ROOT
            / "ros_nodes/planning/ego_planner_stack/plan_env/src/grid_map.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn('mp_.ego_profile_ != "mapped"', source)
        self.assertIn('mp_.ego_profile_ != "free_space"', source)
        self.assertIn("if (mp_.obstacle_mapping_enabled_)", source)
        self.assertIn("scene obstacle mapping and depth-loss checks disabled", source)
        self.assertIn("if (!isInMap(pos)) return -1;\n  if (!mp_.obstacle_mapping_enabled_) return 0;", header)
        self.assertIn(
            "return mp_.obstacle_mapping_enabled_ && md_.flag_depth_odom_timeout_;",
            header,
        )


if __name__ == "__main__":
    unittest.main()

import tempfile
import time
import unittest
from pathlib import Path

import yaml

from agent.ros_command_executor.executor import (
    RosCommandExecutor,
    RosCommandRuntimeError,
    _spf_direct_samples_ok,
)


def write_config(path):
    config = {
        "commands": {
            "health": {
                "enabled": True,
                "type": "builtin",
            },
            "set_goal": {
                "enabled": True,
                "type": "publish",
                "topic": "/planning/goal",
                "msg_type": "geometry_msgs/PoseStamped",
                "args": {
                    "x": {"type": "float", "required": True, "min": -10.0, "max": 10.0},
                    "y": {"type": "float", "required": True},
                    "z": {"type": "float", "required": True},
                    "yaw": {"type": "float", "default": 0.0},
                    "frame_id": {"type": "string", "default": "world"},
                },
                "message": {
                    "header": {"frame_id": "{frame_id}"},
                    "pose": {
                        "position": {"x": "{x}", "y": "{y}", "z": "{z}"},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    },
                },
                "transforms": {"yaw_to_quaternion": True},
            },
            "takeoff": {
                "enabled": False,
                "type": "builtin",
            },
            "safe_takeoff": {
                "enabled": True,
                "type": "safety_command",
                "action": "takeoff",
                "dry_run": True,
                "allow_execute": False,
                "check_cache_ttl": 10.0,
                "args": {
                    "dry_run": {"type": "bool", "default": True},
                },
                "checks": {
                    "require_ros_master": True,
                    "required_nodes": ["/mavros", "/px4ctrl"],
                    "required_subscriber_topics": ["/px4ctrl/takeoff_land"],
                },
                "publish": {
                    "topic": "/px4ctrl/takeoff_land",
                    "msg_type": "quadrotor_msgs/TakeoffLand",
                    "message": {"takeoff_land_cmd": 1},
                },
            },
            "safe_land": {
                "enabled": True,
                "type": "safety_command",
                "action": "land",
                "dry_run": True,
                "allow_execute": True,
                "args": {
                    "dry_run": {"type": "bool", "default": True},
                },
                "checks": {
                    "require_ros_master": True,
                },
                "publish": {
                    "topic": "/px4ctrl/takeoff_land",
                    "msg_type": "quadrotor_msgs/TakeoffLand",
                    "message": {"takeoff_land_cmd": 2},
                },
            },
            "topic_hz": {
                "enabled": True,
                "type": "builtin",
                "args": {
                    "topic": {"type": "string", "required": True},
                    "timeout": {"type": "float", "default": 2.5},
                },
            },
        }
    }
    path.write_text(yaml.safe_dump(config), encoding="utf-8")


class RosCommandExecutorTest(unittest.TestCase):
    def make_executor(self, tmp_path):
        config_path = tmp_path / "ros_commands.yaml"
        write_config(config_path)
        return RosCommandExecutor(
            config_path=str(config_path),
            workspace_root=str(tmp_path),
            ros_setup="",
            workspace_setup="missing_setup.bash",
            ros_master_uri="http://127.0.0.1:9",
        )

    def test_health_without_ros_master(self):
        with tempfile.TemporaryDirectory() as tmp:
            executor = self.make_executor(Path(tmp))
            result = executor.execute("health")
            self.assertEqual(result["command"], "health")
            self.assertFalse(result["ros"]["master_reachable"])

    def test_spf_e2e_success_requires_direct_target_and_px4ctrl_command(self):
        self.assertTrue(
            _spf_direct_samples_ok(
                {
                    "spf_position": {"ok": True},
                    "position_cmd": {"ok": True},
                }
            )
        )
        self.assertFalse(
            _spf_direct_samples_ok(
                {
                    "planning_goal": {"ok": True},
                    "bspline": {"ok": True},
                    "position_cmd": {"ok": True},
                }
            )
        )

    def test_reject_unknown_command(self):
        with tempfile.TemporaryDirectory() as tmp:
            executor = self.make_executor(Path(tmp))
            with self.assertRaises(RosCommandRuntimeError):
                executor.execute("unknown")

    def test_reject_disabled_command(self):
        with tempfile.TemporaryDirectory() as tmp:
            executor = self.make_executor(Path(tmp))
            with self.assertRaises(RosCommandRuntimeError):
                executor.execute("takeoff")

    def test_topic_hz_parses_rate(self):
        with tempfile.TemporaryDirectory() as tmp:
            executor = self.make_executor(Path(tmp))
            executor.is_ros_master_reachable = lambda timeout=0.5: True
            calls = []

            def fake_run_capture(command, timeout=None):
                calls.append((command, timeout))
                return "subscribed to [/topic]\naverage rate: 29.995\n", 124, False

            executor._run_capture = fake_run_capture
            result = executor.execute("topic_hz", {"topic": "/topic", "timeout": "2.0"})

            self.assertTrue(result["active"])
            self.assertEqual(result["rate_hz"], 29.995)
            self.assertEqual(calls[0][0], ["timeout", "2.000s", "rostopic", "hz", "-w", "20", "/topic"])

    def test_validate_args_and_render_template(self):
        with tempfile.TemporaryDirectory() as tmp:
            executor = self.make_executor(Path(tmp))
            args = executor._validate_args(
                "set_goal",
                executor.commands["set_goal"],
                {"x": "1.0", "y": "2.0", "z": "3.0"},
            )
            rendered = executor._render_template(executor.commands["set_goal"]["message"], args)

            self.assertEqual(args["frame_id"], "world")
            self.assertEqual(rendered["header"]["frame_id"], "world")
            self.assertEqual(rendered["pose"]["position"]["x"], 1.0)

    def test_set_goal_yaw_renders_quaternion(self):
        with tempfile.TemporaryDirectory() as tmp:
            executor = self.make_executor(Path(tmp))
            args = executor._validate_args(
                "set_goal",
                executor.commands["set_goal"],
                {"x": "1.0", "y": "2.0", "z": "3.0", "yaw": "1.57079632679"},
            )
            rendered = executor._render_message(executor.commands["set_goal"], args)

            self.assertAlmostEqual(rendered["pose"]["orientation"]["z"], 0.70710678118, places=6)
            self.assertAlmostEqual(rendered["pose"]["orientation"]["w"], 0.70710678118, places=6)

    def test_reject_unlisted_arg(self):
        with tempfile.TemporaryDirectory() as tmp:
            executor = self.make_executor(Path(tmp))
            with self.assertRaises(RosCommandRuntimeError):
                executor.execute("set_goal", {"x": "1", "y": "2", "z": "3", "shell": "bad"})

    def test_reject_out_of_range_float(self):
        with tempfile.TemporaryDirectory() as tmp:
            executor = self.make_executor(Path(tmp))
            with self.assertRaises(RosCommandRuntimeError):
                executor.execute("set_goal", {"x": "11", "y": "2", "z": "3"})

    def test_safe_command_fails_when_ros_master_missing(self):
        with tempfile.TemporaryDirectory() as tmp:
            executor = self.make_executor(Path(tmp))
            result = executor.execute("safe_takeoff")

            self.assertFalse(result["ok"])
            self.assertFalse(result["executed"])
            self.assertFalse(result["checks_ok"])
            self.assertEqual(result["detail"], "safety checks failed")
            self.assertEqual(result["checks"][0]["name"], "ros_master")

    def test_safe_command_dry_run_does_not_publish(self):
        with tempfile.TemporaryDirectory() as tmp:
            executor = self.make_executor(Path(tmp))
            executor.is_ros_master_reachable = lambda timeout=0.5: True
            executor.safety_checker._ros_master_reachable = executor.is_ros_master_reachable

            published = []

            def fake_publish(config, args):
                published.append((config, args))
                return "published"

            executor._publish_message = fake_publish
            result = executor.execute("safe_land")

            self.assertTrue(result["ok"])
            self.assertTrue(result["checks_ok"])
            self.assertTrue(result["dry_run"])
            self.assertFalse(result["executed"])
            self.assertEqual(published, [])

    def test_safe_command_requires_allow_execute_for_real_publish(self):
        with tempfile.TemporaryDirectory() as tmp:
            executor = self.make_executor(Path(tmp))
            executor.is_ros_master_reachable = lambda timeout=0.5: True
            executor.safety_checker._ros_master_reachable = executor.is_ros_master_reachable
            executor.commands["safe_takeoff"]["checks"] = {"require_ros_master": True}
            result = executor.execute("safe_takeoff", {"dry_run": "false"})

            self.assertFalse(result["ok"])
            self.assertTrue(result["checks_ok"])
            self.assertFalse(result["dry_run"])
            self.assertFalse(result["allow_execute"])
            self.assertFalse(result["executed"])
            self.assertEqual(result["detail"], "execution is disabled by allow_execute=false")

    def test_safe_command_execute_reuses_recent_successful_checks(self):
        with tempfile.TemporaryDirectory() as tmp:
            executor = self.make_executor(Path(tmp))
            executor.commands["safe_takeoff"]["allow_execute"] = True
            executor.commands["safe_takeoff"]["checks"] = {"require_ros_master": True}
            executor.is_ros_master_reachable = lambda timeout=0.5: True
            executor.safety_checker._ros_master_reachable = executor.is_ros_master_reachable

            run_count = {"checks": 0}
            original_run_checks = executor.safety_checker.run_checks

            def fake_run_checks(config):
                run_count["checks"] += 1
                return original_run_checks(config)

            executor.safety_checker.run_checks = fake_run_checks

            published = []

            def fake_publish(config, args):
                published.append((config, args))
                return "published"

            executor._publish_message = fake_publish

            check_result = executor.execute("safe_takeoff")
            execute_result = executor.execute("safe_takeoff", {"dry_run": "false"})

            self.assertEqual(run_count["checks"], 1)
            self.assertTrue(check_result["checks_ok"])
            self.assertFalse(check_result["checks_cached"])
            self.assertFalse(check_result["executed"])
            self.assertTrue(execute_result["checks_cached"])
            self.assertTrue(execute_result["executed"])
            self.assertEqual(len(published), 1)

    def test_safe_command_cache_expires_before_execute(self):
        with tempfile.TemporaryDirectory() as tmp:
            executor = self.make_executor(Path(tmp))
            executor.commands["safe_takeoff"]["allow_execute"] = True
            executor.commands["safe_takeoff"]["check_cache_ttl"] = 0.01
            executor.commands["safe_takeoff"]["checks"] = {"require_ros_master": True}
            executor.is_ros_master_reachable = lambda timeout=0.5: True
            executor.safety_checker._ros_master_reachable = executor.is_ros_master_reachable

            run_count = {"checks": 0}
            original_run_checks = executor.safety_checker.run_checks

            def fake_run_checks(config):
                run_count["checks"] += 1
                return original_run_checks(config)

            executor.safety_checker.run_checks = fake_run_checks
            executor._publish_message = lambda config, args: "published"

            executor.execute("safe_takeoff")
            time.sleep(0.02)
            result = executor.execute("safe_takeoff", {"dry_run": "false"})

            self.assertEqual(run_count["checks"], 2)
            self.assertFalse(result["checks_cached"])
            self.assertTrue(result["executed"])

    def test_safe_command_failed_checks_are_not_cached(self):
        with tempfile.TemporaryDirectory() as tmp:
            executor = self.make_executor(Path(tmp))
            executor.commands["safe_takeoff"]["allow_execute"] = True
            executor.commands["safe_takeoff"]["checks"] = {"require_ros_master": True}
            executor.is_ros_master_reachable = lambda timeout=0.5: False
            executor.safety_checker._ros_master_reachable = executor.is_ros_master_reachable

            first = executor.execute("safe_takeoff")
            executor.is_ros_master_reachable = lambda timeout=0.5: True
            executor.safety_checker._ros_master_reachable = executor.is_ros_master_reachable
            executor._publish_message = lambda config, args: "published"
            second = executor.execute("safe_takeoff", {"dry_run": "false"})

            self.assertFalse(first["checks_ok"])
            self.assertFalse(first["checks_cached"])
            self.assertTrue(second["checks_ok"])
            self.assertFalse(second["checks_cached"])
            self.assertTrue(second["executed"])

    def test_safe_command_checks_can_pass_with_mocked_ros_cli(self):
        with tempfile.TemporaryDirectory() as tmp:
            executor = self.make_executor(Path(tmp))
            executor.is_ros_master_reachable = lambda timeout=0.5: True
            executor.safety_checker._ros_master_reachable = executor.is_ros_master_reachable
            executor.commands["safe_takeoff"]["checks"].update(
                {
                    "state": {
                        "topic": "/mavros/state",
                        "require_connected": True,
                        "require_armed": False,
                    },
                    "extended_state": {
                        "topic": "/mavros/extended_state",
                        "allowed_landed_states": [1],
                    },
                    "battery": {
                        "topic": "/mavros/battery",
                        "min_percentage": 0.25,
                        "min_voltage": 13.2,
                    },
                    "sample_topics": [
                        {"name": "odom", "topic": "/vins_fusion/imu_propagate"},
                    ],
                }
            )

            def fake_run(command, timeout=None):
                if command[:2] == ["rosnode", "list"]:
                    return "/mavros\n/px4ctrl\n"
                if command[:2] == ["rostopic", "list"]:
                    return (
                        "/mavros/state\n"
                        "/mavros/extended_state\n"
                        "/mavros/battery\n"
                        "/vins_fusion/imu_propagate\n"
                        "/px4ctrl/takeoff_land\n"
                    )
                if command[:2] == ["rostopic", "info"]:
                    if command[2] == "/px4ctrl/takeoff_land":
                        return "Type: quadrotor_msgs/TakeoffLand\n\nPublishers: None\n\nSubscribers:\n * /px4ctrl\n"
                    return "Type: std_msgs/String\n\nPublishers:\n * /test_pub\n\nSubscribers: None\n"
                if command[:4] == ["rostopic", "echo", "-n", "1"]:
                    topic = command[4]
                    if topic == "/mavros/state":
                        return "connected: true\narmed: false\nmode: POSCTL\n"
                    if topic == "/mavros/extended_state":
                        return "landed_state: 1\n"
                    if topic == "/mavros/battery":
                        return "percentage: 0.8\nvoltage: 16.1\n"
                    if topic == "/vins_fusion/imu_propagate":
                        return "header:\n  stamp:\n    secs: 1\n"
                raise AssertionError("unexpected command: %r" % (command,))

            executor.safety_checker._run = fake_run
            result = executor.execute("safe_takeoff")

            self.assertTrue(result["ok"])
            self.assertTrue(result["checks_ok"])
            self.assertFalse(result["executed"])
            self.assertEqual(result["detail"], "dry-run passed; command was not published")

    def test_safe_command_skips_sampling_missing_required_topics(self):
        with tempfile.TemporaryDirectory() as tmp:
            executor = self.make_executor(Path(tmp))
            executor.is_ros_master_reachable = lambda timeout=0.5: True
            executor.safety_checker._ros_master_reachable = executor.is_ros_master_reachable
            executor.commands["safe_takeoff"]["checks"].update(
                {
                    "state": {
                        "name": "mavros_state",
                        "topic": "/mavros/state",
                        "require_connected": True,
                    },
                    "required_topics": ["/mavros/state"],
                    "required_subscriber_topics": [],
                }
            )

            def fake_run(command, timeout=None):
                if command[:2] == ["rosnode", "list"]:
                    return "/mavros\n/px4ctrl\n"
                if command[:2] == ["rostopic", "list"]:
                    return "/px4ctrl/takeoff_land\n"
                if command[:2] == ["rostopic", "info"]:
                    raise AssertionError("must not inspect missing required topic")
                if command[:4] == ["rostopic", "echo", "-n", "1"]:
                    raise AssertionError("must not sample a missing required topic")
                raise AssertionError("unexpected command: %r" % (command,))

            executor.safety_checker._run = fake_run
            result = executor.execute("safe_takeoff")

            self.assertFalse(result["ok"])
            self.assertFalse(result["checks_ok"])
            self.assertIn(
                "topic is missing: /mavros/state",
                [item["detail"] for item in result["checks"]],
            )

    def test_safe_command_skips_sampling_topics_without_publishers(self):
        with tempfile.TemporaryDirectory() as tmp:
            executor = self.make_executor(Path(tmp))
            executor.is_ros_master_reachable = lambda timeout=0.5: True
            executor.safety_checker._ros_master_reachable = executor.is_ros_master_reachable
            executor.commands["safe_takeoff"]["checks"].update(
                {
                    "state": {
                        "name": "mavros_state",
                        "topic": "/mavros/state",
                        "require_connected": True,
                    },
                    "required_topics": ["/mavros/state"],
                    "required_subscriber_topics": [],
                }
            )

            def fake_run(command, timeout=None):
                if command[:2] == ["rosnode", "list"]:
                    return "/mavros\n/px4ctrl\n"
                if command[:2] == ["rostopic", "list"]:
                    return "/mavros/state\n"
                if command[:2] == ["rostopic", "info"]:
                    return "Type: mavros_msgs/State\n\nPublishers: None\n\nSubscribers:\n * /test\n"
                if command[:4] == ["rostopic", "echo", "-n", "1"]:
                    raise AssertionError("must not sample a topic without publishers")
                raise AssertionError("unexpected command: %r" % (command,))

            executor.safety_checker._run = fake_run
            result = executor.execute("safe_takeoff")

            self.assertFalse(result["ok"])
            self.assertFalse(result["checks_ok"])
            self.assertIn(
                "topic has no publisher: /mavros/state",
                [item["detail"] for item in result["checks"]],
            )

    def test_safe_command_requires_subscriber_topics(self):
        with tempfile.TemporaryDirectory() as tmp:
            executor = self.make_executor(Path(tmp))
            executor.is_ros_master_reachable = lambda timeout=0.5: True
            executor.safety_checker._ros_master_reachable = executor.is_ros_master_reachable

            def fake_run(command, timeout=None):
                if command[:2] == ["rosnode", "list"]:
                    return "/mavros\n/px4ctrl\n"
                if command[:2] == ["rostopic", "list"]:
                    return "/px4ctrl/takeoff_land\n"
                if command[:2] == ["rostopic", "info"]:
                    return "Type: quadrotor_msgs/TakeoffLand\n\nPublishers: None\n\nSubscribers:\n * /px4ctrl\n"
                raise AssertionError("unexpected command: %r" % (command,))

            executor.safety_checker._run = fake_run
            result = executor.execute("safe_takeoff")

            self.assertTrue(result["ok"])
            self.assertTrue(result["checks_ok"])
            self.assertIn(
                "required subscriber topics are present",
                [item["detail"] for item in result["checks"]],
            )

    def test_safe_command_fails_when_subscriber_topic_has_no_subscribers(self):
        with tempfile.TemporaryDirectory() as tmp:
            executor = self.make_executor(Path(tmp))
            executor.is_ros_master_reachable = lambda timeout=0.5: True
            executor.safety_checker._ros_master_reachable = executor.is_ros_master_reachable

            def fake_run(command, timeout=None):
                if command[:2] == ["rosnode", "list"]:
                    return "/mavros\n/px4ctrl\n"
                if command[:2] == ["rostopic", "list"]:
                    return "/px4ctrl/takeoff_land\n"
                if command[:2] == ["rostopic", "info"]:
                    return "Type: quadrotor_msgs/TakeoffLand\n\nPublishers: None\n\nSubscribers: None\n"
                raise AssertionError("unexpected command: %r" % (command,))

            executor.safety_checker._run = fake_run
            result = executor.execute("safe_takeoff")

            self.assertFalse(result["ok"])
            self.assertFalse(result["checks_ok"])
            self.assertIn(
                "topics have no subscribers: /px4ctrl/takeoff_land",
                [item["detail"] for item in result["checks"]],
            )


if __name__ == "__main__":
    unittest.main()

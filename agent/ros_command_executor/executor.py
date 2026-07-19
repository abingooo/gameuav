#!/usr/bin/env python3

import copy
import math
import os
import re
import shlex
import signal
import socket
import subprocess
import tempfile
import time
from dataclasses import dataclass, field

import yaml

from agent.ros_command_executor.safety_checks import RosSafetyChecker


VALID_ARG_CHARS = set("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_./:-,+")
VALID_STRING_CHARS = set("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_./:-,+# ")
SINGLE_PLACEHOLDER_RE = re.compile(r"^\{([A-Za-z_][A-Za-z0-9_]*)\}$")
ROSTOPIC_HZ_RATE_RE = re.compile(r"average rate:\s*([0-9]+(?:\.[0-9]+)?)")


class RosCommandConfigError(ValueError):
    pass


class RosCommandRuntimeError(RuntimeError):
    pass


@dataclass
class RosCommandExecutor:
    config_path: str
    workspace_root: str
    ros_setup: str = "/opt/ros/noetic/setup.bash"
    workspace_setup: str = "devel/setup.bash"
    ros_home: str = "/tmp/gameuav_ros_home"
    ros_log_dir: str = "/tmp/gameuav_ros_logs"
    ros_master_uri: str = "http://localhost:11311"
    default_timeout: float = 5.0
    env_extra: dict = field(default_factory=dict)

    def __post_init__(self):
        self.workspace_root = os.path.abspath(self.workspace_root)
        self.commands = self._load_commands(self.config_path)
        self._safety_check_cache = {}
        self.safety_checker = RosSafetyChecker(
            run_command=self._run,
            ros_master_reachable=self.is_ros_master_reachable,
        )

    def _resolve_path(self, path):
        if os.path.isabs(path):
            return path
        return os.path.join(self.workspace_root, path)

    def _load_commands(self, config_path):
        with open(config_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        commands = data.get("commands")
        if not isinstance(commands, dict):
            raise RosCommandConfigError("config must contain commands mapping")
        for name, config in commands.items():
            self._validate_command_config(name, config)
        return commands

    def _validate_command_config(self, name, config):
        if not isinstance(config, dict):
            raise RosCommandConfigError("command %s config must be object" % name)
        command_type = config.get("type")
        if command_type not in {"builtin", "publish", "safety_command"}:
            raise RosCommandConfigError("command %s has invalid type %s" % (name, command_type))
        if command_type == "publish":
            for key in ("topic", "msg_type", "message"):
                if key not in config:
                    raise RosCommandConfigError("publish command %s requires %s" % (name, key))
        if command_type == "safety_command":
            for key in ("action", "publish"):
                if key not in config:
                    raise RosCommandConfigError("safety_command %s requires %s" % (name, key))
            publish_config = config.get("publish")
            if not isinstance(publish_config, dict):
                raise RosCommandConfigError("safety_command %s publish must be object" % name)
            for key in ("topic", "msg_type", "message"):
                if key not in publish_config:
                    raise RosCommandConfigError("safety_command %s publish requires %s" % (name, key))
        args = config.get("args", {})
        if args is not None and not isinstance(args, dict):
            raise RosCommandConfigError("command %s args must be object" % name)

    def _build_env(self):
        env = os.environ.copy()
        env.update(self.env_extra)
        env.setdefault("ROS_HOME", self.ros_home)
        env.setdefault("ROS_LOG_DIR", self.ros_log_dir)
        env.setdefault("ROS_MASTER_URI", self.ros_master_uri)
        return env

    def _shell_command(self, command):
        workspace_setup = self._resolve_path(self.workspace_setup)
        parts = ["set -e"]
        if self.ros_setup:
            parts.append("source %s" % shlex.quote(self.ros_setup))
        if os.path.exists(workspace_setup):
            parts.append("source %s" % shlex.quote(workspace_setup))
        parts.append("exec " + " ".join(shlex.quote(str(part)) for part in command))
        return " && ".join(parts)

    def _run(self, command, timeout=None):
        timeout = self.default_timeout if timeout is None else float(timeout)
        proc = subprocess.run(
            ["/bin/bash", "-lc", self._shell_command(command)],
            cwd=self.workspace_root,
            env=self._build_env(),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout,
            check=False,
        )
        if proc.returncode != 0:
            raise RosCommandRuntimeError(
                "command failed rc=%d output=%s" % (proc.returncode, proc.stdout.strip())
            )
        return proc.stdout.strip()

    def is_ros_master_reachable(self, timeout=0.5):
        host = "localhost"
        port = 11311
        uri = self.ros_master_uri
        if uri.startswith("http://"):
            uri = uri[len("http://") :]
        if "/" in uri:
            uri = uri.split("/", 1)[0]
        if ":" in uri:
            host, port_str = uri.rsplit(":", 1)
            try:
                port = int(port_str)
            except ValueError:
                return False
        try:
            with socket.create_connection((host, port), timeout=timeout):
                return True
        except OSError:
            return False

    def list_commands(self):
        return {
            name: {
                "enabled": bool(config.get("enabled", False)),
                "type": config.get("type"),
                "description": config.get("description", ""),
                "args": config.get("args", {}),
            }
            for name, config in sorted(self.commands.items())
        }

    def execute(self, command_name, args=None):
        if command_name not in self.commands:
            raise RosCommandRuntimeError("ros command is not whitelisted: %s" % command_name)

        config = self.commands[command_name]
        if not bool(config.get("enabled", False)):
            raise RosCommandRuntimeError("ros command is disabled: %s" % command_name)

        args = self._validate_args(command_name, config, args or {})
        command_type = config["type"]

        if command_type == "builtin":
            return self._execute_builtin(command_name, config, args)
        if command_type == "publish":
            return self._execute_publish(command_name, config, args)
        if command_type == "safety_command":
            return self._execute_safety_command(command_name, config, args)

        raise RosCommandRuntimeError("unsupported ros command type: %s" % command_type)

    def _validate_args(self, command_name, config, requested_args):
        spec = config.get("args") or {}
        requested_args = requested_args or {}
        for key in requested_args:
            if key not in spec:
                raise RosCommandRuntimeError("argument is not allowed: %s" % key)

        result = {}
        for key, arg_spec in spec.items():
            if isinstance(arg_spec, str):
                arg_spec = {"type": arg_spec}
            arg_type = arg_spec.get("type", "string")
            required = bool(arg_spec.get("required", False))
            default = arg_spec.get("default")
            if key not in requested_args:
                if required and default is None:
                    raise RosCommandRuntimeError("missing required argument: %s" % key)
                if default is not None:
                    result[key] = self._coerce_arg(key, default, arg_type, arg_spec)
                continue
            result[key] = self._coerce_arg(key, requested_args[key], arg_type, arg_spec)

        return result

    def _coerce_arg(self, key, value, arg_type, arg_spec):
        if arg_type == "float":
            try:
                number = float(value)
            except (TypeError, ValueError):
                raise RosCommandRuntimeError("argument %s must be float" % key)
            if not math.isfinite(number):
                raise RosCommandRuntimeError("argument %s must be finite" % key)
            if "min" in arg_spec and number < float(arg_spec["min"]):
                raise RosCommandRuntimeError("argument %s is below min" % key)
            if "max" in arg_spec and number > float(arg_spec["max"]):
                raise RosCommandRuntimeError("argument %s is above max" % key)
            return number

        if arg_type == "int":
            try:
                return int(value)
            except (TypeError, ValueError):
                raise RosCommandRuntimeError("argument %s must be int" % key)

        if arg_type == "bool":
            value = str(value).lower()
            if value in {"true", "1", "yes"}:
                return True
            if value in {"false", "0", "no"}:
                return False
            raise RosCommandRuntimeError("argument %s must be bool" % key)

        value = str(value)
        allowed_values = arg_spec.get("allowed_values")
        if allowed_values is not None and value not in [str(item) for item in allowed_values]:
            raise RosCommandRuntimeError("argument %s has disallowed value %s" % (key, value))
        if arg_spec.get("allow_natural_text"):
            max_length = int(arg_spec.get("max_length", 300))
            if len(value) > max_length:
                raise RosCommandRuntimeError("argument %s exceeds max length" % key)
            if any(ord(char) < 32 and char not in "\t\n\r" for char in value):
                raise RosCommandRuntimeError("argument %s contains control characters" % key)
            return value
        valid_chars = VALID_STRING_CHARS if arg_type == "string" else VALID_ARG_CHARS
        if not set(value) <= valid_chars:
            raise RosCommandRuntimeError("argument %s contains unsafe characters" % key)
        return value

    def _execute_builtin(self, command_name, config, args):
        if command_name == "health":
            return {
                "ok": True,
                "command": command_name,
                "ros": {
                    "master_reachable": self.is_ros_master_reachable(),
                    "master_uri": self.ros_master_uri,
                },
            }

        if command_name == "topic_list":
            self._require_ros_master()
            output = self._run(["rostopic", "list"], timeout=config.get("timeout", self.default_timeout))
            return {
                "ok": True,
                "command": command_name,
                "topics": [line for line in output.splitlines() if line],
            }

        if command_name == "node_list":
            self._require_ros_master()
            output = self._run(["rosnode", "list"], timeout=config.get("timeout", self.default_timeout))
            return {
                "ok": True,
                "command": command_name,
                "nodes": [line for line in output.splitlines() if line],
            }

        if command_name == "topic_hz":
            self._require_ros_master()
            topic = _validated_topic_name(args.get("topic"))
            sample_timeout = float(args.get("timeout", config.get("timeout", 2.5)))
            sample_timeout = max(1.0, min(10.0, sample_timeout))
            started_at = time.time()
            output, returncode, timed_out = self._run_capture(
                ["timeout", "%.3fs" % sample_timeout, "rostopic", "hz", "-w", "20", topic],
                timeout=sample_timeout + 1.0,
            )
            rate_hz = _parse_rostopic_hz_rate(output)
            active = rate_hz is not None and rate_hz > 0.0
            return {
                "ok": True,
                "command": command_name,
                "topic": topic,
                "active": active,
                "rate_hz": rate_hz,
                "checked_at": time.time(),
                "probe_age_sec": max(0.0, time.time() - started_at),
                "detail": _topic_hz_detail(output, returncode, active, rate_hz, timed_out),
            }

        if command_name == "topic_info":
            self._require_ros_master()
            topic = _validated_topic_name(args.get("topic"))
            output = self._run(["rostopic", "info", topic], timeout=config.get("timeout", self.default_timeout))
            return {
                "ok": True,
                "command": command_name,
                "topic": topic,
                "info": _parse_rostopic_info(output),
                "detail": output,
            }

        if command_name == "topic_echo":
            self._require_ros_master()
            topic = _validated_topic_name(args.get("topic"))
            sample_timeout = float(args.get("timeout", config.get("timeout", 5.0)))
            sample_timeout = max(1.0, min(30.0, sample_timeout))
            output, returncode, timed_out = self._run_capture(
                ["timeout", "%.3fs" % sample_timeout, "rostopic", "echo", "-n", "1", topic],
                timeout=sample_timeout + 1.0,
            )
            message = None
            error = None
            if output.strip():
                try:
                    message = next((item for item in yaml.safe_load_all(output) if item is not None), None)
                except yaml.YAMLError as exc:
                    error = "failed to parse message: %s" % exc
            elif timed_out:
                error = "timed out waiting for %s" % topic
            elif returncode:
                error = "rostopic echo exited with rc=%s" % returncode
            return {
                "ok": error is None and message is not None,
                "command": command_name,
                "topic": topic,
                "message": message,
                "detail": error or "received message from %s" % topic,
                "timed_out": timed_out,
                "returncode": returncode,
            }

        if command_name == "spf_e2e_check":
            self._require_ros_master()
            command_text = str(args.get("data") or "").strip()
            if not command_text:
                raise RosCommandRuntimeError("missing SPF command text")
            sample_timeout = float(args.get("timeout", config.get("timeout", 45.0)))
            sample_timeout = max(5.0, min(120.0, sample_timeout))
            return self._execute_spf_e2e_check(command_text, sample_timeout)

        if command_name == "command_list":
            return {
                "ok": True,
                "command": command_name,
                "commands": self.list_commands(),
            }

        raise RosCommandRuntimeError("unsupported builtin ros command: %s" % command_name)

    def _run_capture(self, command, timeout=None):
        timeout = self.default_timeout if timeout is None else float(timeout)
        proc = subprocess.Popen(
            ["/bin/bash", "-lc", self._shell_command(command)],
            cwd=self.workspace_root,
            env=self._build_env(),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            output, _ = proc.communicate(timeout=timeout)
            return output or "", proc.returncode, False
        except subprocess.TimeoutExpired:
            _terminate_process_group(proc, signal.SIGTERM)
            try:
                output, _ = proc.communicate(timeout=0.5)
            except subprocess.TimeoutExpired:
                _terminate_process_group(proc, signal.SIGKILL)
                output, _ = proc.communicate()
            return output or "", proc.returncode, True

    def _execute_spf_e2e_check(self, command_text, timeout):
        started_at = time.time()
        node_output = self._run(["rosnode", "list"], timeout=5.0)
        node_list = [line for line in node_output.splitlines() if line]
        topics = [
            "/spf/status",
            "/spf/user_command",
            "/spf/action_command",
            "/spf/enable",
            "/spf/task/start",
            "/spf/task/control",
            "/spf/task/status",
            "/rgb1/image_raw",
            "/vins_fusion/imu_propagate",
            "/control/spf_position",
            "/planning/goal",
            "/planning/goal_yaw_deg",
            "/drone_0_planning/bspline",
            "/position_cmd",
            "/control/ego_position_cmd",
            "/control/position_cmd",
            "/px4ctrl/takeoff_land",
        ]
        topic_info = {}
        for topic in topics:
            try:
                output = self._run(["rostopic", "info", topic], timeout=5.0)
                topic_info[topic] = {
                    "ok": True,
                    "topic": topic,
                    "info": _parse_rostopic_info(output),
                    "detail": output,
                }
            except Exception as exc:
                topic_info[topic] = {
                    "ok": False,
                    "topic": topic,
                    "detail": str(exc),
                }

        capture_topics = {
            "spf_status": ("/spf/status", timeout),
            "spf_task_status": ("/spf/task/status", 3.0),
            "spf_position": ("/control/spf_position", timeout),
            "position_cmd": ("/control/position_cmd", min(timeout, 20.0)),
            "takeoff_land": ("/px4ctrl/takeoff_land", 2.0),
            "mavros_state": ("/mavros/state", 3.0),
            "vins_position": ("/vins_fusion/imu_propagate/pose/pose/position", 3.0),
        }
        procs = {}
        with tempfile.TemporaryDirectory(prefix="gameuav_spf_e2e_") as tmpdir:
            for name, (topic, capture_timeout) in capture_topics.items():
                path = os.path.join(tmpdir, "%s.yaml" % name)
                command = ["timeout", "%.3fs" % float(capture_timeout), "rostopic", "echo", "-n", "1", topic]
                shell = "%s > %s 2>&1" % (self._shell_command(command), shlex.quote(path))
                proc = subprocess.Popen(
                    ["/bin/bash", "-lc", shell],
                    cwd=self.workspace_root,
                    env=self._build_env(),
                    text=True,
                    start_new_session=True,
                )
                procs[name] = (proc, path, topic)
            time.sleep(0.5)
            publish_detail = self._publish_message(
                {
                    "topic": "/spf/user_command",
                    "msg_type": "std_msgs/String",
                    "message": {"data": "{data}"},
                    "timeout": 5.0,
                },
                {"data": command_text},
            )
            deadline = time.time() + timeout + 2.0
            for name in ("spf_position", "position_cmd"):
                proc, _path, _topic = procs[name]
                remaining = max(0.1, deadline - time.time())
                try:
                    proc.wait(timeout=remaining)
                except subprocess.TimeoutExpired:
                    _terminate_process_group(proc, signal.SIGTERM)
            for name, (proc, _path, _topic) in procs.items():
                if proc.poll() is None:
                    _terminate_process_group(proc, signal.SIGTERM)
                    try:
                        proc.wait(timeout=0.5)
                    except subprocess.TimeoutExpired:
                        _terminate_process_group(proc, signal.SIGKILL)
                        proc.wait()
            samples = {}
            for name, (proc, path, topic) in procs.items():
                try:
                    output = open(path, "r", encoding="utf-8", errors="replace").read()
                except OSError:
                    output = ""
                samples[name] = _parse_topic_echo_sample(name, topic, output, proc.returncode)

        return {
            "ok": _spf_direct_samples_ok(samples),
            "command": "spf_e2e_check",
            "spf_command": command_text,
            "elapsed_sec": max(0.0, time.time() - started_at),
            "publish": {
                "ok": True,
                "topic": "/spf/user_command",
                "msg_type": "std_msgs/String",
                "detail": publish_detail,
            },
            "nodes": node_list,
            "topics": topic_info,
            "samples": samples,
        }

    def _execute_publish(self, command_name, config, args):
        self._require_ros_master()
        output = self._publish_message(config, args)
        return {
            "ok": True,
            "command": command_name,
            "topic": config["topic"],
            "msg_type": config["msg_type"],
            "detail": output,
        }

    def _execute_safety_command(self, command_name, config, args):
        allow_execute = bool(config.get("allow_execute", False))
        dry_run = bool(args.get("dry_run", config.get("dry_run", True)))
        cache_ttl = float(config.get("check_cache_ttl", 0.0) or 0.0)
        cache_entry = self._get_safety_check_cache(command_name, cache_ttl) if not dry_run else None
        checks_cached = cache_entry is not None

        if checks_cached:
            checks = copy.deepcopy(cache_entry["checks"])
            checks_ok = True
        else:
            check_results = self.safety_checker.run_checks(config)
            checks = [item.as_dict() for item in check_results]
            checks_ok = all(item.ok for item in check_results)
            if checks_ok and cache_ttl > 0.0:
                self._set_safety_check_cache(command_name, checks)

        result = {
            "ok": checks_ok and (dry_run or allow_execute),
            "command": command_name,
            "action": config["action"],
            "dry_run": dry_run,
            "allow_execute": allow_execute,
            "checks_ok": checks_ok,
            "checks": checks,
            "checks_cached": checks_cached,
        }
        if checks_cached:
            result["check_cache_age_sec"] = max(0.0, time.time() - cache_entry["checked_at"])

        if not checks_ok:
            result["executed"] = False
            result["detail"] = "safety checks failed"
            return result

        if dry_run:
            result["executed"] = False
            result["detail"] = "dry-run passed; command was not published"
            return result

        if not allow_execute:
            result["executed"] = False
            result["ok"] = False
            result["detail"] = "execution is disabled by allow_execute=false"
            return result

        publish_config = config["publish"]
        output = self._publish_message(publish_config, args)
        result.update(
            {
                "executed": True,
                "topic": publish_config["topic"],
                "msg_type": publish_config["msg_type"],
                "detail": output,
            }
        )
        self._clear_safety_check_cache(command_name)
        return result

    def _get_safety_check_cache(self, command_name, ttl):
        if ttl <= 0.0:
            return None
        entry = self._safety_check_cache.get(command_name)
        if not entry:
            return None
        if time.time() - entry["checked_at"] > ttl:
            self._clear_safety_check_cache(command_name)
            return None
        return entry

    def _set_safety_check_cache(self, command_name, checks):
        self._safety_check_cache[command_name] = {
            "checked_at": time.time(),
            "checks": copy.deepcopy(checks),
        }

    def _clear_safety_check_cache(self, command_name):
        self._safety_check_cache.pop(command_name, None)

    def _publish_message(self, config, args):
        message = self._render_message(config, args)
        message_text = yaml.safe_dump(message, default_flow_style=True).strip()
        command = [
            "rostopic",
            "pub",
            "-1",
            config["topic"],
            config["msg_type"],
            message_text,
        ]
        return self._run(command, timeout=config.get("timeout", self.default_timeout))

    def _render_template(self, value, args):
        if isinstance(value, dict):
            return {key: self._render_template(item, args) for key, item in value.items()}
        if isinstance(value, list):
            return [self._render_template(item, args) for item in value]
        if isinstance(value, str):
            match = SINGLE_PLACEHOLDER_RE.match(value)
            if match:
                key = match.group(1)
                if key not in args:
                    raise RosCommandRuntimeError("missing template argument: %s" % key)
                return args[key]
            try:
                return value.format(**args)
            except KeyError as exc:
                raise RosCommandRuntimeError("missing template argument: %s" % exc)
        return value

    def _render_message(self, config, args):
        message = self._render_template(config["message"], args)
        transforms = config.get("transforms") or {}
        if transforms.get("yaw_to_quaternion"):
            message = copy.deepcopy(message)
            yaw = args.get("yaw")
            if yaw is not None:
                self._apply_yaw_to_quaternion(message, yaw)
        return message

    def _apply_yaw_to_quaternion(self, message, yaw):
        pose = message.setdefault("pose", {})
        orientation = pose.setdefault("orientation", {})
        half_yaw = float(yaw) / 2.0
        orientation["x"] = 0.0
        orientation["y"] = 0.0
        orientation["z"] = math.sin(half_yaw)
        orientation["w"] = math.cos(half_yaw)

    def _require_ros_master(self):
        if not self.is_ros_master_reachable():
            raise RosCommandRuntimeError("ROS master is not reachable")


def _terminate_process_group(proc, sig):
    try:
        os.killpg(proc.pid, sig)
    except ProcessLookupError:
        pass


def _parse_rostopic_hz_rate(output):
    rate_hz = None
    for match in ROSTOPIC_HZ_RATE_RE.finditer(output or ""):
        try:
            rate_hz = float(match.group(1))
        except ValueError:
            continue
    return rate_hz


def _topic_hz_detail(output, returncode, active, rate_hz=None, timed_out=False):
    output = (output or "").strip()
    if active:
        return "average rate: %.1f Hz" % rate_hz
    if output:
        return output.splitlines()[-1].strip()
    if timed_out:
        return "no hz sample before timeout"
    if returncode not in (None, 0):
        return "rostopic hz exited with rc=%s" % returncode
    return "no message before timeout"


def _validated_topic_name(topic):
    topic = str(topic or "")
    if not topic.startswith("/") or any(char.isspace() for char in topic):
        raise RosCommandRuntimeError("invalid ROS topic")
    return topic


def _parse_rostopic_info(output):
    result = {
        "type": None,
        "publishers": [],
        "subscribers": [],
    }
    section = None
    for line in (output or "").splitlines():
        stripped = line.strip()
        if stripped.startswith("Type:"):
            result["type"] = stripped.split(":", 1)[1].strip()
        elif stripped.startswith("Publishers:"):
            section = "publishers"
        elif stripped.startswith("Subscribers:"):
            section = "subscribers"
        elif stripped.startswith("* ") and section:
            node = stripped[2:].split("(", 1)[0].strip()
            if node:
                result[section].append(node)
    return result


def _parse_topic_echo_sample(name, topic, output, returncode):
    text = (output or "").strip()
    message = None
    error = None
    if text:
        try:
            message = next((item for item in yaml.safe_load_all(text) if item is not None), None)
        except yaml.YAMLError as exc:
            error = "failed to parse message: %s" % exc
    if message is None and error is None:
        if returncode == 124:
            error = "timed out waiting for %s" % topic
        elif returncode not in (None, 0):
            error = "rostopic echo exited with rc=%s" % returncode
        else:
            error = "no message captured"
    return {
        "ok": error is None and message is not None,
        "name": name,
        "topic": topic,
        "message": message,
        "detail": error or "received message from %s" % topic,
        "returncode": returncode,
    }


def _spf_direct_samples_ok(samples):
    return bool(
        samples.get("spf_position", {}).get("ok")
        and samples.get("position_cmd", {}).get("ok")
    )

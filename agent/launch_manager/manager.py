#!/usr/bin/env python3

import os
import signal
import shlex
import socket
import subprocess
import threading
import time
import xmlrpc.client
from dataclasses import dataclass, field
from pathlib import Path

import yaml


VALID_MODULE_TYPES = {"process", "launch"}
VALID_ARG_CHARS = set("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_./:-,+")
DEFAULT_MAX_MODULE_LOG_BYTES = 64 * 1024 * 1024
DEFAULT_RETAINED_LOG_TAIL_BYTES = 1024 * 1024


class _TimeoutXmlRpcTransport(xmlrpc.client.Transport):
    def __init__(self, timeout):
        super().__init__()
        self.timeout = float(timeout)

    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = self.timeout
        return connection


class ModuleConfigError(ValueError):
    pass


class ModuleRuntimeError(RuntimeError):
    pass


@dataclass
class ModuleProcess:
    name: str
    process: subprocess.Popen
    command: list
    log_path: str
    started_at: float
    status: str = "running"
    last_returncode: int = None


@dataclass
class LaunchManager:
    config_path: str
    workspace_root: str
    ros_setup: str = "/opt/ros/noetic/setup.bash"
    workspace_setup: str = "devel/setup.bash"
    log_dir: str = "logs/agent"
    ros_home: str = "/tmp/gameuav_ros_home"
    ros_log_dir: str = "/tmp/gameuav_ros_logs"
    ros_master_uri: str = "http://localhost:11311"
    env_extra: dict = field(default_factory=dict)
    max_module_log_bytes: int = DEFAULT_MAX_MODULE_LOG_BYTES
    retained_log_tail_bytes: int = DEFAULT_RETAINED_LOG_TAIL_BYTES
    _lifecycle_lock: object = field(default_factory=threading.RLock, init=False, repr=False)

    def __post_init__(self):
        self.workspace_root = os.path.abspath(self.workspace_root)
        self.log_dir = self._resolve_path(self.log_dir)
        Path(self.log_dir).mkdir(parents=True, exist_ok=True)
        self.modules = self._load_modules(self.config_path)
        self.processes = {}

    def _resolve_path(self, path):
        if os.path.isabs(path):
            return path
        return os.path.join(self.workspace_root, path)

    def _load_modules(self, config_path):
        with open(config_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        modules = data.get("modules")
        if not isinstance(modules, dict):
            raise ModuleConfigError("config must contain modules mapping")

        for name, config in modules.items():
            self._validate_module(name, config)
        for name, config in modules.items():
            unknown = set(config.get("conflicts", [])) - set(modules)
            if unknown:
                raise ModuleConfigError(
                    "module %s references unknown conflicts: %s"
                    % (name, ", ".join(sorted(unknown)))
                )
        return modules

    def _validate_module(self, name, config):
        if not isinstance(config, dict):
            raise ModuleConfigError("module %s config must be object" % name)
        module_type = config.get("type")
        if module_type not in VALID_MODULE_TYPES:
            raise ModuleConfigError("module %s has invalid type %s" % (name, module_type))
        if module_type == "process" and not config.get("command"):
            raise ModuleConfigError("process module %s requires command" % name)
        if module_type == "launch":
            if not config.get("package") and not config.get("file"):
                raise ModuleConfigError("launch module %s requires package or file" % name)
            if not config.get("launch") and not config.get("file"):
                raise ModuleConfigError("launch module %s requires launch or file" % name)
            requires_roscore = config.get("requires_roscore", True)
            if not isinstance(requires_roscore, bool):
                raise ModuleConfigError("module %s requires_roscore must be boolean" % name)

        allowed_args = config.get("allowed_args", {})
        if allowed_args is not None and not isinstance(allowed_args, dict):
            raise ModuleConfigError("module %s allowed_args must be object" % name)
        autostart = config.get("autostart", False)
        if not isinstance(autostart, bool):
            raise ModuleConfigError("module %s autostart must be boolean" % name)
        conflicts = config.get("conflicts", [])
        if not isinstance(conflicts, list) or any(
            not isinstance(item, str) or not item or item == name for item in conflicts
        ):
            raise ModuleConfigError("module %s conflicts must be a list of other module names" % name)

        pre_start = config.get("pre_start")
        if pre_start is not None:
            if not isinstance(pre_start, dict):
                raise ModuleConfigError("module %s pre_start must be object" % name)
            command = pre_start.get("command")
            if not isinstance(command, list) or not command:
                raise ModuleConfigError("module %s pre_start requires command list" % name)
            timeout = pre_start.get("timeout", 60.0)
            if not isinstance(timeout, (int, float)) or timeout <= 0:
                raise ModuleConfigError("module %s pre_start timeout must be positive" % name)

    def _module_config(self, name):
        if name not in self.modules:
            raise ModuleRuntimeError("module is not whitelisted: %s" % name)
        return self.modules[name]

    def _is_process_alive(self, proc):
        return proc.poll() is None

    def _refresh_status(self, name):
        proc_info = self.processes.get(name)
        if not proc_info:
            return None
        self._limit_log_size(proc_info.log_path)
        returncode = proc_info.process.poll()
        if returncode is None:
            proc_info.status = "running"
        else:
            proc_info.status = "exited"
            proc_info.last_returncode = returncode
        return proc_info

    def _limit_log_size(self, log_path):
        try:
            size = os.path.getsize(log_path)
            limit = max(0, int(self.max_module_log_bytes))
            if not limit or size <= limit:
                return
            tail_size = max(0, min(int(self.retained_log_tail_bytes), limit))
            with open(log_path, "rb") as source:
                if tail_size:
                    source.seek(-min(size, tail_size), os.SEEK_END)
                    tail = source.read()
                else:
                    tail = b""
            marker = (
                "[gameuav-agent] log truncated after exceeding %d bytes; retaining final %d bytes\n"
                % (limit, len(tail))
            ).encode("utf-8")
            with open(log_path, "r+b", buffering=0) as target:
                target.seek(0)
                target.write(marker)
                target.write(tail)
                target.truncate()
        except OSError:
            return

    def _validate_requested_args(self, config, requested_args):
        requested_args = requested_args or {}
        allowed_args = config.get("allowed_args") or {}
        defaults = config.get("args") or {}
        merged = dict(defaults)

        for key, value in requested_args.items():
            if key not in allowed_args:
                raise ModuleRuntimeError("argument is not allowed: %s" % key)
            value = str(value)
            allowed_values = allowed_args[key]
            if allowed_values is not None and value not in [str(v) for v in allowed_values]:
                raise ModuleRuntimeError("argument %s has disallowed value %s" % (key, value))
            if not set(value) <= VALID_ARG_CHARS:
                raise ModuleRuntimeError("argument %s contains unsafe characters" % key)
            merged[key] = value

        return merged

    def _build_command(self, name, config, requested_args=None):
        module_type = config["type"]
        args = self._validate_requested_args(config, requested_args)

        if module_type == "process":
            command = config["command"]
            if isinstance(command, str):
                command = command.split()
            if not isinstance(command, list) or not command:
                raise ModuleRuntimeError("invalid process command for %s" % name)
            return [str(part) for part in command]

        command = ["roslaunch", "--wait"]
        if config.get("file"):
            command.append(config["file"])
        else:
            command.extend([config["package"], config["launch"]])
        for key in sorted(args):
            command.append("%s:=%s" % (key, args[key]))
        return command

    def _build_pre_start_command(self, name, config, requested_args=None):
        pre_start = config.get("pre_start")
        if not pre_start:
            return None

        args = self._validate_requested_args(config, requested_args)
        command = []
        for part in pre_start["command"]:
            try:
                command.append(str(part).format_map(args))
            except KeyError as exc:
                raise ModuleConfigError(
                    "module %s pre_start references missing argument %s" % (name, exc.args[0])
                )
        return command

    def _build_env(self):
        env = os.environ.copy()
        env.update(self.env_extra)
        env["ROS_HOME"] = self.ros_home
        env["ROS_LOG_DIR"] = self.ros_log_dir
        env["ROS_MASTER_URI"] = self.ros_master_uri
        return env

    def _shell_command(self, command):
        workspace_setup = self._resolve_path(self.workspace_setup)
        parts = ["set -e"]
        if self.ros_setup:
            parts.append("source %s" % self.ros_setup)
        if os.path.exists(workspace_setup):
            parts.append("source %s" % workspace_setup)
        parts.append("exec " + " ".join(shlex.quote(p) for p in command))
        return " && ".join(parts)

    def _run_pre_start(self, name, config, requested_args, log_file):
        command = self._build_pre_start_command(name, config, requested_args)
        if not command:
            return

        timeout = float(config["pre_start"].get("timeout", 60.0))
        proc = subprocess.Popen(
            ["/bin/bash", "-lc", self._shell_command(command)],
            cwd=self.workspace_root,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            env=self._build_env(),
            preexec_fn=os.setsid,
            close_fds=True,
        )
        try:
            returncode = proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            try:
                proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                proc.wait()
            raise ModuleRuntimeError(
                "module %s pre_start timed out after %.1fs" % (name, timeout)
            )

        if returncode != 0:
            raise ModuleRuntimeError(
                "module %s pre_start failed with return code %d" % (name, returncode)
            )

    def _ensure_roscore_for_launch(self, name, config):
        if name == "roscore" or config.get("type") != "launch":
            return
        if not config.get("requires_roscore", True):
            return
        if "roscore" not in self.modules:
            return

        roscore = self._refresh_status("roscore")
        if roscore and roscore.status == "running":
            self._wait_for_ros_master()
            return
        self.start("roscore")

    def _ros_master_run_id(self, timeout=0.5):
        try:
            proxy = xmlrpc.client.ServerProxy(
                self.ros_master_uri,
                transport=_TimeoutXmlRpcTransport(timeout),
                allow_none=True,
            )
            code, _message, value = proxy.getParam("/gameuav_agent", "/run_id")
        except (OSError, ValueError, xmlrpc.client.Error):
            return None
        try:
            success = int(code) == 1
        except (TypeError, ValueError):
            return None
        if not success or not value:
            return None
        return str(value)

    def _wait_for_ros_master(self, timeout=5.0):
        deadline = time.monotonic() + float(timeout)
        stable_run_id = None
        stable_since = None
        while time.monotonic() < deadline:
            roscore = self._refresh_status("roscore")
            if roscore is not None and roscore.status != "running":
                raise ModuleRuntimeError(
                    "roscore exited before ROS master became ready: returncode=%s log=%s"
                    % (roscore.last_returncode, roscore.log_path)
                )

            remaining = max(0.05, deadline - time.monotonic())
            run_id = self._ros_master_run_id(timeout=min(0.2, remaining))
            sampled_at = time.monotonic()
            if run_id:
                if run_id != stable_run_id:
                    stable_run_id = run_id
                    stable_since = sampled_at
                elif stable_since is not None and sampled_at - stable_since >= 0.2:
                    return run_id
            else:
                stable_run_id = None
                stable_since = None
            time.sleep(0.05)

        raise ModuleRuntimeError(
            "ROS master did not expose a stable /run_id at %s within %.1fs"
            % (self.ros_master_uri, timeout)
        )

    def start(self, name, args=None):
        with self._lifecycle_lock:
            config = self._module_config(name)
            current = self._refresh_status(name)
            if current and current.status == "running":
                return self.status(name)
            if name == "roscore":
                external_run_id = self._ros_master_run_id(timeout=0.2)
                if external_run_id:
                    raise ModuleRuntimeError(
                        "ROS master is already running outside this agent at %s: run_id=%s"
                        % (self.ros_master_uri, external_run_id)
                    )

            running_conflicts = []
            for conflict_name in config.get("conflicts", []):
                conflict = self._refresh_status(conflict_name)
                if conflict is not None and conflict.status == "running":
                    running_conflicts.append(conflict_name)
            if running_conflicts:
                raise ModuleRuntimeError(
                    "cannot start %s while conflicting modules are running: %s; stop them first"
                    % (name, ", ".join(sorted(running_conflicts)))
                )

            command = self._build_command(name, config, args)
            self._ensure_roscore_for_launch(name, config)
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            log_path = os.path.join(self.log_dir, "%s_%s.log" % (name, timestamp))
            Path(os.path.dirname(log_path)).mkdir(parents=True, exist_ok=True)

            log_file = open(log_path, "ab", buffering=0)
            try:
                self._run_pre_start(name, config, args, log_file)
            except Exception:
                log_file.close()
                raise

            proc = subprocess.Popen(
                ["/bin/bash", "-lc", self._shell_command(command)],
                cwd=self.workspace_root,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                env=self._build_env(),
                preexec_fn=os.setsid,
                close_fds=True,
            )
            log_file.close()

            self.processes[name] = ModuleProcess(
                name=name,
                process=proc,
                command=command,
                log_path=log_path,
                started_at=time.time(),
            )
            if name == "roscore":
                try:
                    self._wait_for_ros_master()
                except Exception:
                    self.stop(name, timeout=2.0)
                    raise
            return self.status(name)

    def autostart(self):
        with self._lifecycle_lock:
            names = [name for name, config in self.modules.items() if config.get("autostart", False)]
            if "roscore" in names:
                names.remove("roscore")
                names.insert(0, "roscore")
            return {name: self.start(name) for name in names}

    def _running_roscore_dependents(self):
        dependents = []
        for module_name, config in self.modules.items():
            if module_name == "roscore" or config.get("type") != "launch":
                continue
            if not config.get("requires_roscore", True):
                continue
            status = self._refresh_status(module_name)
            if status is not None and status.status == "running":
                dependents.append(module_name)
        return sorted(dependents)

    def stop(self, name, timeout=8.0):
        with self._lifecycle_lock:
            self._module_config(name)
            proc_info = self._refresh_status(name)
            if not proc_info:
                return {
                    "module": name,
                    "status": "stopped",
                    "pid": None,
                    "detail": "not started by agent",
                }
            if proc_info.status != "running":
                return self.status(name)
            if name == "roscore":
                dependents = self._running_roscore_dependents()
                if dependents:
                    raise ModuleRuntimeError(
                        "cannot stop roscore while ROS launch modules are running: %s"
                        % ", ".join(dependents)
                    )

            pgid = os.getpgid(proc_info.process.pid)
            os.killpg(pgid, signal.SIGINT)
            deadline = time.time() + timeout
            while time.time() < deadline:
                if proc_info.process.poll() is not None:
                    return self.status(name)
                time.sleep(0.1)

            os.killpg(pgid, signal.SIGTERM)
            deadline = time.time() + 3.0
            while time.time() < deadline:
                if proc_info.process.poll() is not None:
                    return self.status(name)
                time.sleep(0.1)

            os.killpg(pgid, signal.SIGKILL)
            return self.status(name)

    def restart(self, name, args=None):
        with self._lifecycle_lock:
            self.stop(name)
            return self.start(name, args)

    def stop_all(self, timeout=8.0):
        with self._lifecycle_lock:
            names = list(reversed(self.processes))
            if "roscore" in names:
                names.remove("roscore")
                names.append("roscore")

            results = {}
            for name in names:
                try:
                    results[name] = self.stop(name, timeout=timeout)
                except (ModuleRuntimeError, OSError) as exc:
                    results[name] = {
                        "module": name,
                        "status": "error",
                        "detail": str(exc),
                    }
            return results

    def status(self, name=None):
        if name is None:
            return {module_name: self.status(module_name) for module_name in sorted(self.modules)}

        self._module_config(name)
        proc_info = self._refresh_status(name)
        config = self.modules[name]

        if not proc_info:
            return {
                "module": name,
                "status": "stopped",
                "pid": None,
                "type": config["type"],
                "log_path": None,
                "started_at": None,
                "returncode": None,
            }

        return {
            "module": name,
            "status": proc_info.status,
            "pid": proc_info.process.pid if proc_info.status == "running" else None,
            "type": config["type"],
            "command": proc_info.command,
            "log_path": proc_info.log_path,
            "started_at": proc_info.started_at,
            "returncode": proc_info.last_returncode,
        }

    def list_modules(self):
        return {
            name: {
                "type": config["type"],
                "description": config.get("description", ""),
                "allowed_args": config.get("allowed_args", {}),
                "conflicts": config.get("conflicts", []),
            }
            for name, config in sorted(self.modules.items())
        }

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

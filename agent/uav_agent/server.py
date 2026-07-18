#!/usr/bin/env python3

import argparse
import logging
import os
import socketserver
import sys
import time
from pathlib import Path

from agent.launch_manager.manager import LaunchManager, ModuleRuntimeError
from agent.ros_command_executor.executor import (
    RosCommandExecutor,
    RosCommandRuntimeError,
)
from comm.protocol.agent_protocol import (
    ALLOWED_ACTIONS,
    MESSAGE_TYPE_MODULE_COMMAND,
    MESSAGE_TYPE_ROS_COMMAND,
    ProtocolError,
    decode_message,
    encode_message,
    make_error,
    make_module_status,
    make_ros_command_result,
)


LOG = logging.getLogger("uav_agent")


class AgentState:
    def __init__(self, manager, ros_executor, uav_id, auth_token, allowed_sources=None, started_at=None):
        self.manager = manager
        self.ros_executor = ros_executor
        self.uav_id = uav_id
        self.auth_token = auth_token
        self.allowed_sources = set(allowed_sources or [])
        self.started_at = started_at if started_at is not None else time.time()


class AgentRequestHandler(socketserver.StreamRequestHandler):
    def handle(self):
        for raw_line in self.rfile:
            response = self.server.handle_raw_message(raw_line)
            self.wfile.write(encode_message(response))
            self.wfile.flush()


class ThreadedAgentServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, server_address, handler_class, state):
        super().__init__(server_address, handler_class)
        self.state = state

    def handle_raw_message(self, raw_line):
        request_id = None
        source_id = "unknown"
        sequence_id = None
        try:
            message = decode_message(raw_line)
            source_id = message["source_id"]
            sequence_id = message["sequence_id"]
            payload = message["payload"]
            request_id = payload.get("request_id")

            if self.state.allowed_sources and source_id not in self.state.allowed_sources:
                raise ProtocolError("source is not allowed: %s" % source_id)

            if message["target_id"] not in (self.state.uav_id, "*"):
                raise ProtocolError("target mismatch: %s" % message["target_id"])

            self._check_auth(payload)

            if message["message_type"] == MESSAGE_TYPE_ROS_COMMAND:
                result = self._handle_ros_command(payload)
                return make_ros_command_result(
                    source_id=self.state.uav_id,
                    target_id=source_id,
                    request_id=request_id,
                    payload=result,
                    sequence_id=sequence_id,
                )

            if message["message_type"] != MESSAGE_TYPE_MODULE_COMMAND:
                raise ProtocolError("unsupported message_type: %s" % message["message_type"])

            result = self._handle_module_command(payload)
            return make_module_status(
                source_id=self.state.uav_id,
                target_id=source_id,
                request_id=request_id,
                payload=result,
                sequence_id=sequence_id,
            )
        except (ProtocolError, ModuleRuntimeError, RosCommandRuntimeError, ValueError, OSError) as exc:
            LOG.warning("request rejected: %s", exc)
            return make_error(
                source_id=self.state.uav_id,
                target_id=source_id,
                request_id=request_id,
                code=exc.__class__.__name__,
                detail=str(exc),
                sequence_id=sequence_id,
            )
        except Exception as exc:
            LOG.exception("unexpected request failure")
            return make_error(
                source_id=self.state.uav_id,
                target_id=source_id,
                request_id=request_id,
                code="InternalError",
                detail=str(exc),
                sequence_id=sequence_id,
            )

    def _check_auth(self, payload):
        if not self.state.auth_token:
            return
        if payload.get("auth_token") != self.state.auth_token:
            raise ProtocolError("invalid auth token")

    def _handle_module_command(self, payload):
        action = payload.get("action")
        if action not in ALLOWED_ACTIONS:
            raise ProtocolError("unsupported action: %s" % action)

        module = payload.get("module")
        args = payload.get("args") or {}

        if action == "list":
            return {
                "ok": True,
                "action": action,
                "modules": self.state.manager.list_modules(),
            }

        if action == "health":
            return self._health()

        if not module:
            raise ProtocolError("module is required")

        if action == "start":
            status = self.state.manager.start(module, args=args)
        elif action == "stop":
            status = self.state.manager.stop(module)
        elif action == "restart":
            status = self.state.manager.restart(module, args=args)
        elif action == "status":
            status = self.state.manager.status(module)
        else:
            raise ProtocolError("unsupported action: %s" % action)

        return {
            "ok": True,
            "action": action,
            "module": module,
            "status": status,
        }

    def _handle_ros_command(self, payload):
        command = payload.get("command")
        if not command:
            raise ProtocolError("command is required")
        args = payload.get("args") or {}
        result = self.state.ros_executor.execute(command, args=args)
        result.setdefault("ok", True)
        result.setdefault("command", command)
        return result

    def _health(self):
        return {
            "ok": True,
            "action": "health",
            "agent": {
                "uav_id": self.state.uav_id,
                "pid": os.getpid(),
                "started_at": self.state.started_at,
                "uptime_sec": max(0.0, time.time() - self.state.started_at),
            },
            "ros": {
                "master_reachable": self.state.manager.is_ros_master_reachable(),
                "master_uri": self.state.manager.ros_master_uri,
                "commands": self.state.ros_executor.list_commands(),
            },
            "modules": self.state.manager.status(),
        }


def build_parser():
    parser = argparse.ArgumentParser(description="GameUAV local management agent")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--uav-id", default="uav1")
    parser.add_argument("--config", default="config/modules/uav_agent.yaml")
    parser.add_argument("--ros-command-config", default="config/ros_commands/ros_command_executor.yaml")
    parser.add_argument("--workspace-root", default=str(Path(__file__).resolve().parents[2]))
    parser.add_argument("--log-dir", default="logs/agent")
    parser.add_argument("--ros-home", default="/tmp/gameuav_ros_home")
    parser.add_argument("--ros-log-dir", default="/tmp/gameuav_ros_logs")
    parser.add_argument("--auth-token", default=os.environ.get("GAMEUAV_AGENT_TOKEN", "uavuavuavuav"))
    parser.add_argument("--allowed-source", action="append", default=[])
    parser.add_argument("--verbose", action="store_true")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    workspace_root = Path(args.workspace_root).resolve()
    config_path = Path(args.config)
    if not config_path.is_absolute():
        config_path = workspace_root / config_path
    ros_command_config_path = Path(args.ros_command_config)
    if not ros_command_config_path.is_absolute():
        ros_command_config_path = workspace_root / ros_command_config_path

    manager = LaunchManager(
        config_path=str(config_path),
        workspace_root=str(workspace_root),
        log_dir=args.log_dir,
        ros_home=args.ros_home,
        ros_log_dir=args.ros_log_dir,
    )
    autostart_results = manager.autostart()
    for module, status in autostart_results.items():
        LOG.info("autostarted module %s with status %s", module, status.get("status"))
    ros_executor = RosCommandExecutor(
        config_path=str(ros_command_config_path),
        workspace_root=str(workspace_root),
        ros_home=args.ros_home,
        ros_log_dir=args.ros_log_dir,
        ros_master_uri=manager.ros_master_uri,
    )
    state = AgentState(
        manager=manager,
        ros_executor=ros_executor,
        uav_id=args.uav_id,
        auth_token=args.auth_token,
        allowed_sources=args.allowed_source,
    )

    try:
        with ThreadedAgentServer((args.host, args.port), AgentRequestHandler, state) as server:
            LOG.info("uav_agent listening on %s:%d as %s", args.host, args.port, args.uav_id)
            server.serve_forever()
    except KeyboardInterrupt:
        LOG.info("stopping uav_agent")
    finally:
        LOG.info("stopping managed modules")
        for module, result in manager.stop_all().items():
            if result.get("status") == "error":
                LOG.warning("failed to stop module %s: %s", module, result.get("detail"))
    return 0


if __name__ == "__main__":
    sys.exit(main())

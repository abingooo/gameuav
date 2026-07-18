#!/usr/bin/env python3

import argparse
import json
import os
import socket
import sys
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

from comm.protocol.agent_protocol import decode_message, encode_message, make_module_command, make_ros_command


def parse_arg_values(values):
    result = {}
    for item in values or []:
        if ":=" in item:
            key, value = item.split(":=", 1)
        elif "=" in item:
            key, value = item.split("=", 1)
        else:
            raise ValueError("argument must use key:=value or key=value: %s" % item)
        result[key] = value
    return result


def add_common_args(parser):
    parser.add_argument("--arg", action="append", default=[], help="Whitelisted arg, key:=value")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--source-id", default="agentctl")
    parser.add_argument(
        "--target-id",
        default=os.environ.get("GAMEUAV_UAV_ID") or socket.gethostname(),
        help="Target UAV ID (default: GAMEUAV_UAV_ID or local hostname)",
    )
    parser.add_argument("--timeout", type=float)
    parser.add_argument("--auth-token", default="uavuavuavuav")


def build_module_parser():
    parser = argparse.ArgumentParser(description="Send a module command to uav_agent")
    parser.add_argument("action", choices=["start", "stop", "restart", "status", "list", "health"])
    parser.add_argument("module", nargs="?")
    add_common_args(parser)
    return parser


def build_ros_parser():
    parser = argparse.ArgumentParser(description="Send a ROS runtime command to uav_agent")
    parser.add_argument("command_group", choices=["ros"])
    parser.add_argument("ros_command")
    add_common_args(parser)
    return parser


def build_parser(argv=None):
    argv = argv or sys.argv[1:]
    if argv and argv[0] == "ros":
        return build_ros_parser()
    return build_module_parser()


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    args = build_parser(argv).parse_args(argv)
    command_args = parse_arg_values(args.arg)
    if getattr(args, "command_group", None) == "ros":
        message = make_ros_command(
            command=args.ros_command,
            args=command_args,
            source_id=args.source_id,
            target_id=args.target_id,
            auth_token=args.auth_token,
        )
    else:
        message = make_module_command(
            action=args.action,
            module=args.module,
            args=command_args,
            source_id=args.source_id,
            target_id=args.target_id,
            auth_token=args.auth_token,
        )

    is_slow_module_action = (
        not getattr(args, "command_group", None) and args.action in {"start", "restart"}
    )
    timeout = args.timeout if args.timeout is not None else (60.0 if is_slow_module_action else 5.0)
    with socket.create_connection((args.host, args.port), timeout=timeout) as sock:
        sock.settimeout(timeout)
        sock.sendall(encode_message(message))
        line = sock.makefile("rb").readline()

    response = decode_message(line)
    print(json.dumps(response, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if response["message_type"] != "error" else 1


if __name__ == "__main__":
    sys.exit(main())

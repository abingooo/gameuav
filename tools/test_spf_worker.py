#!/usr/bin/env python3

"""Smoke-test the GameUAV See-Point-Fly worker.

This script calls only the SPF worker HTTP API. It does not publish ROS topics
and does not command the UAV.
"""

import argparse
import base64
import json
from pathlib import Path
import shlex
import subprocess
import sys
import time
import urllib.error
import urllib.request

import cv2
import numpy as np


def build_test_image():
    image = np.zeros((360, 480, 3), dtype=np.uint8)
    image[:] = (40, 40, 40)
    cv2.circle(image, (360, 180), 45, (0, 0, 255), -1)
    cv2.putText(
        image,
        "red target",
        (290, 260),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
    )
    return image


def load_image(path):
    image = cv2.imread(path, cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("failed to read image: %s" % path)
    return image


def encode_jpeg_b64(image, quality):
    ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise RuntimeError("failed to encode image")
    return base64.b64encode(encoded.tobytes()).decode("ascii")


def capture_ros_image(topic, quality, timeout):
    repo_root = Path(__file__).resolve().parents[1]
    helper = repo_root / "tools" / "capture_ros_image.py"
    command = " ".join(
        [
            "source /opt/ros/noetic/setup.bash",
            "&&",
            "cd",
            shlex.quote(str(repo_root)),
            "&&",
            "env",
            "-u",
            "VIRTUAL_ENV",
            "-u",
            "PYTHONHOME",
            "/usr/bin/python3",
            shlex.quote(str(helper)),
            "--topic",
            shlex.quote(topic),
            "--quality",
            shlex.quote(str(quality)),
            "--timeout",
            shlex.quote(str(timeout)),
        ]
    )
    completed = subprocess.run(
        ["/bin/bash", "-lc", command],
        check=False,
        text=True,
        capture_output=True,
        timeout=max(10.0, float(timeout) + 5.0),
    )
    payload = parse_capture_payload(completed.stdout)
    if completed.returncode != 0 or not payload.get("ok"):
        detail = payload.get("detail") if payload else ""
        stderr = completed.stderr.strip()
        raise RuntimeError(detail or stderr or "failed to capture ROS image")
    return decode_data_url_image(payload["data_url"])


def parse_capture_payload(stdout):
    for line in reversed(stdout.splitlines()):
        line = line.strip()
        if not line:
            continue
        try:
            return json.loads(line)
        except json.JSONDecodeError:
            continue
    return {}


def decode_data_url_image(data_url):
    if "," in data_url:
        _, encoded = data_url.split(",", 1)
    else:
        encoded = data_url
    raw = base64.b64decode(encoded)
    image = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("failed to decode captured ROS image")
    return image


def post_infer(url, payload, timeout):
    data = json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(
        url,
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return response.status, json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        body = exc.read().decode("utf-8", "replace")
        try:
            payload = json.loads(body)
        except json.JSONDecodeError:
            payload = {"ok": False, "error": body}
        return exc.code, payload


def read_input_image(args):
    if args.ros_topic:
        return capture_ros_image(args.ros_topic, args.quality, args.ros_timeout)
    if args.image:
        return load_image(args.image)
    return build_test_image()


def run_once(args, index=None):
    image = read_input_image(args)
    payload = {
        "command": args.command,
        "image_jpeg_b64": encode_jpeg_b64(image, args.quality),
    }
    status, response = post_infer(args.url, payload, args.timeout)
    prefix = "run %d: " % index if index is not None else ""
    print("%s%d" % (prefix, status), flush=True)
    print(json.dumps(response, ensure_ascii=False, indent=2, sort_keys=True), flush=True)
    return 0 if 200 <= status < 300 and response.get("ok") else 1


def run_loop(args):
    index = 1
    worst_status = 0
    try:
        while args.count <= 0 or index <= args.count:
            started = time.monotonic()
            worst_status = max(worst_status, run_once(args, index))
            index += 1
            if args.count > 0 and index > args.count:
                break
            elapsed = time.monotonic() - started
            time.sleep(max(0.0, args.interval - elapsed))
    except KeyboardInterrupt:
        print("interrupted", file=sys.stderr)
    return worst_status


def main():
    parser = argparse.ArgumentParser(description="Test SPF worker /infer")
    parser.add_argument("--url", default="http://127.0.0.1:9310/infer")
    parser.add_argument("--command", default="fly toward the red target")
    parser.add_argument("--image", default="", help="Optional image path. Defaults to a generated red target.")
    parser.add_argument("--ros-topic", default="", help="Optional ROS Image topic, e.g. /usb_camera/image_raw.")
    parser.add_argument("--ros-timeout", type=float, default=5.0)
    parser.add_argument("--quality", type=int, default=85)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--loop", action="store_true", help="Run repeated SPF inferences.")
    parser.add_argument("--count", type=int, default=1, help="Number of loop iterations. Use 0 for infinite.")
    parser.add_argument("--interval", type=float, default=1.0, help="Minimum seconds between loop iterations.")
    args = parser.parse_args()

    if args.loop:
        return run_loop(args)
    return run_once(args)


if __name__ == "__main__":
    sys.exit(main())

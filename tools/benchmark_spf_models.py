#!/usr/bin/env python3

"""Benchmark SPF-compatible vision models on annotated target-point images."""

import argparse
import base64
import json
import math
import os
import re
import statistics
import time
import urllib.error
import urllib.request
from pathlib import Path

import cv2


ROOT = Path(__file__).resolve().parents[1]
TEST_ROOT = ROOT / "strategy" / "see_point_fly" / "upstream" / "src" / "spf_tools" / "vlm" / "test_images"

# Boxes use SPF's normalized coordinates: [xmin, ymin, xmax, ymax] on a 0-1000 scale.
CASES = [
    ("easyCar_1", TEST_ROOT / "TBT" / "easyCar_1.png", "fly toward the black police car", [285, 585, 460, 720]),
    ("easyCar_2", TEST_ROOT / "TBT" / "easyCar_2.png", "fly toward the black police car", [610, 580, 815, 715]),
    ("oneCar_1", TEST_ROOT / "TBT" / "oneCar_1.png", "fly toward the black sedan on the right, not the box truck", [645, 390, 745, 485]),
    ("oneCar_2", TEST_ROOT / "TBT" / "oneCar_2.png", "fly toward the black sedan on the right, not the box truck", [695, 575, 810, 680]),
    ("red_1", TEST_ROOT / "TBT" / "red_1.png", "fly through the red circular ring", [395, 115, 460, 245]),
    ("red_2", TEST_ROOT / "TBT" / "red_2.png", "fly through the red circular ring", [395, 320, 465, 450]),
    ("red_3", TEST_ROOT / "TBT" / "red_3.png", "fly through the red circular ring", [370, 350, 445, 490]),
    ("red_4", TEST_ROOT / "TBT" / "red_4.png", "fly through the red circular ring", [420, 165, 515, 335]),
    ("twoCar", TEST_ROOT / "twoCar.png", "fly toward the black police car on the far right", [730, 570, 900, 690]),
]

DEFAULT_MODELS = [
    "gpt-4.1-mini",
    "gpt-4.1",
    "gemini-2.5-flash-lite",
    "gemini-2.5-flash",
    "qwen3-vl-flash",
]

PROMPT = """You are a drone navigation expert analyzing a drone camera view.

Task: {instruction}

First, identify ALL objects in the image that match the description "{instruction}".
Then, select the MOST RELEVANT target object and place a single point DIRECTLY ON that object.

Return in this exact JSON format:
[{{"point": [y, x], "depth": depth_value, "label": "action description"}}]

Coordinate system:
- x: 0-1000 scale (500=center, >500=right, <500=left)
- y: 0-1000 scale (lower values=higher in image/sky)
- depth: 1-10 scale where 1 is very close and 10 is far away.

IMPORTANT:
- Place the point PRECISELY on the center of the target object.
- Choose the largest/closest matching object if multiple exist unless the instruction says otherwise.
- Return JSON only."""


def encode_image(path):
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("failed to read image: %s" % path)
    width = 640
    height = max(1, round(image.shape[0] * width / image.shape[1]))
    image = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
    ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
    if not ok:
        raise RuntimeError("failed to encode image: %s" % path)
    return base64.b64encode(encoded.tobytes()).decode("ascii")


def extract_content(payload):
    response_texts = []
    for item in payload.get("output", []) or []:
        for content_item in item.get("content", []) or []:
            if content_item.get("type") == "output_text" and content_item.get("text"):
                response_texts.append(content_item["text"])
    if response_texts:
        return "\n".join(response_texts)
    content = payload.get("choices", [{}])[0].get("message", {}).get("content", "")
    if isinstance(content, list):
        content = "\n".join(str(item.get("text", "")) for item in content if isinstance(item, dict))
    return str(content or "")


def parse_point(content):
    cleaned = re.sub(r"^```(?:json)?\s*|\s*```$", "", content.strip(), flags=re.IGNORECASE)
    try:
        payload = json.loads(cleaned)
    except json.JSONDecodeError:
        match = re.search(r"\[\s*\{.*?\}\s*\]", cleaned, flags=re.DOTALL)
        if not match:
            raise
        payload = json.loads(match.group(0))
    item = payload[0] if isinstance(payload, list) else payload
    y, x = item["point"]
    return float(x), float(y), item.get("depth"), item.get("label", "")


def request_model(base_url, api_key, model, image_b64, instruction, timeout):
    if model.startswith("gpt-5"):
        endpoint = "/responses"
        payload = {
            "model": model,
            "input": [{
                "role": "user",
                "content": [
                    {"type": "input_text", "text": PROMPT.format(instruction=instruction)},
                    {"type": "input_image", "image_url": "data:image/jpeg;base64,%s" % image_b64},
                ],
            }],
            "temperature": 0,
            "max_output_tokens": 512,
        }
    else:
        endpoint = "/chat/completions"
        payload = {
            "model": model,
            "messages": [{
                "role": "user",
                "content": [
                    {"type": "text", "text": PROMPT.format(instruction=instruction)},
                    {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64,%s" % image_b64}},
                ],
            }],
            "temperature": 0,
            "max_tokens": 2048 if model == "gemini-2.5-flash" else 256,
        }
    request = urllib.request.Request(
        base_url.rstrip("/") + endpoint,
        data=json.dumps(payload).encode("utf-8"),
        headers={"Authorization": "Bearer %s" % api_key, "Content-Type": "application/json"},
        method="POST",
    )
    started = time.monotonic()
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            body = json.loads(response.read().decode("utf-8"))
        return body, time.monotonic() - started, None
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode("utf-8", "replace")[:500]
        return None, time.monotonic() - started, "http %s: %s" % (exc.code, detail)
    except Exception as exc:
        return None, time.monotonic() - started, str(exc)


def summarize(results, models):
    summaries = []
    for model in models:
        rows = [row for row in results if row["model"] == model]
        valid = [row for row in rows if row["valid"]]
        hits = [row for row in valid if row["hit"]]
        recognized = [row for row in valid if row["target_recognized"]]
        latencies = [row["latency_sec"] for row in rows]
        errors = [row["center_error"] for row in valid]
        summaries.append({
            "model": model,
            "tests": len(rows),
            "valid_json": len(valid),
            "hits": len(hits),
            "accuracy": len(hits) / len(rows) if rows else 0.0,
            "recognized": len(recognized),
            "recognition_accuracy": len(recognized) / len(rows) if rows else 0.0,
            "valid_rate": len(valid) / len(rows) if rows else 0.0,
            "mean_center_error": statistics.mean(errors) if errors else None,
            "mean_latency_sec": statistics.mean(latencies) if latencies else None,
            "p95_latency_sec": sorted(latencies)[max(0, math.ceil(0.95 * len(latencies)) - 1)] if latencies else None,
        })
    return summaries


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-url", default="https://api.zhizengzeng.com/v1")
    parser.add_argument("--model", action="append", dest="models")
    parser.add_argument("--timeout", type=float, default=45.0)
    parser.add_argument("--output")
    args = parser.parse_args()

    api_key = os.environ.get("SPF_BENCH_API_KEY", "")
    if not api_key:
        raise SystemExit("SPF_BENCH_API_KEY is required")
    models = args.models or DEFAULT_MODELS
    images = {name: encode_image(path) for name, path, _instruction, _box in CASES}
    results = []

    for model in models:
        for name, _path, instruction, box in CASES:
            body, latency, request_error = request_model(
                args.base_url, api_key, model, images[name], instruction, args.timeout
            )
            row = {
                "model": model,
                "case": name,
                "instruction": instruction,
                "target_box": box,
                "latency_sec": latency,
                "valid": False,
                "hit": False,
            }
            try:
                if request_error:
                    raise RuntimeError(request_error)
                content = extract_content(body)
                x, y, depth, label = parse_point(content)
                xmin, ymin, xmax, ymax = box
                cx, cy = (xmin + xmax) / 2.0, (ymin + ymax) / 2.0
                hit = xmin <= x <= xmax and ymin <= y <= ymax
                swapped_hit = xmin <= y <= xmax and ymin <= x <= ymax
                row.update({
                    "valid": True,
                    "hit": hit,
                    "swapped_hit": swapped_hit,
                    "target_recognized": hit or swapped_hit,
                    "point": [y, x],
                    "depth": depth,
                    "label": label,
                    "center_error": math.hypot(x - cx, y - cy),
                    "raw": content,
                })
            except Exception as exc:
                row["error"] = str(exc)
                if body is not None:
                    row["raw"] = extract_content(body)
            results.append(row)
            print("%-24s %-10s valid=%-5s hit=%-5s latency=%.2fs" % (
                model, name, row["valid"], row["hit"], latency
            ), flush=True)

    report = {
        "created_at": time.time(),
        "base_url": args.base_url,
        "temperature": 0,
        "cases": len(CASES),
        "summaries": summarize(results, models),
        "results": results,
    }
    output = Path(args.output) if args.output else ROOT / "runtime" / ("spf_model_benchmark_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print("REPORT=%s" % output, flush=True)
    print(json.dumps(report["summaries"], ensure_ascii=False, indent=2), flush=True)


if __name__ == "__main__":
    main()

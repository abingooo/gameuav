#!/usr/bin/env python3

import argparse
import base64
import json
import math
import struct
import sys
import time


def main(argv=None):
    parser = argparse.ArgumentParser(description="Capture one ROS sensor_msgs/Image frame as a BMP data URL.")
    parser.add_argument("--topic", required=True)
    parser.add_argument("--quality", type=int, default=60)
    parser.add_argument("--timeout", type=float, default=3.0)
    args = parser.parse_args(argv)

    try:
        import rospy
        from sensor_msgs.msg import Image
    except Exception as exc:
        return emit_error("failed to import ROS image dependencies: %s" % exc)

    frame = {}
    rospy.init_node("gameuav_gcs_capture_image", anonymous=True, disable_signals=True)

    def callback(message):
        if frame:
            return
        frame["message"] = message

    subscriber = rospy.Subscriber(args.topic, Image, callback, queue_size=1)
    deadline = time.monotonic() + max(0.1, args.timeout)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown() and not frame and time.monotonic() < deadline:
        rate.sleep()
    subscriber.unregister()

    if not frame:
        return emit_error("timed out waiting for image on %s" % args.topic)

    try:
        payload = image_to_payload(frame["message"], args.quality)
    except Exception as exc:
        return emit_error("failed to encode image: %s" % exc)
    payload["ok"] = True
    payload["source_topic"] = args.topic
    print(json.dumps(payload, separators=(",", ":")))
    return 0


def emit_error(detail):
    print(json.dumps({"ok": False, "detail": detail}, separators=(",", ":")))
    return 1


def image_to_payload(message, quality):
    image = image_to_encoded_image(message, quality)
    return {
        "width": image["width"],
        "height": image["height"],
        "encoding": image["encoding"],
        "quality": image["quality"],
        "data_url": "data:%s;base64,%s" % (
            image["mime_type"],
            base64.b64encode(image["data"]).decode("ascii"),
        ),
    }


def image_to_encoded_image(message, quality):
    width = int(message.width)
    height = int(message.height)
    if width <= 0 or height <= 0:
        raise ValueError("image has invalid size")
    scale = max(0.05, min(1.0, int(quality) / 100.0))
    step = max(1, int(round(1.0 / scale)))
    out_width = max(1, int(math.ceil(width / step)))
    out_height = max(1, int(math.ceil(height / step)))
    sampled = convert_to_rgb(message, step)
    image_bytes, mime_type = encode_image(sampled, out_width, out_height, quality)
    return {
        "width": out_width,
        "height": out_height,
        "encoding": message.encoding,
        "quality": int(quality),
        "mime_type": mime_type,
        "data": image_bytes,
    }


def convert_to_rgb(message, step=1):
    cv_rgb = convert_to_rgb_cv(message, step)
    if cv_rgb is not None:
        return cv_rgb

    encoding = (message.encoding or "").lower()
    data = bytes(message.data)
    width = int(message.width)
    height = int(message.height)
    row_step = int(message.step) if message.step else width
    sample_step = max(1, int(step))
    out_width = max(1, int(math.ceil(width / sample_step)))
    out_height = max(1, int(math.ceil(height / sample_step)))
    rgb = bytearray(out_width * out_height * 3)
    out_index = 0
    for y in range(0, height, sample_step):
        src_row = y * row_step
        for x in range(0, width, sample_step):
            dst = out_index * 3
            out_index += 1
            if encoding in {"rgb8", "8uc3"}:
                src = src_row + x * 3
                rgb[dst : dst + 3] = data[src : src + 3]
            elif encoding == "bgr8":
                src = src_row + x * 3
                rgb[dst] = data[src + 2]
                rgb[dst + 1] = data[src + 1]
                rgb[dst + 2] = data[src]
            elif encoding in {"mono8", "8uc1"}:
                value = data[src_row + x]
                rgb[dst : dst + 3] = bytes([value, value, value])
            elif encoding in {"mono16", "16uc1"}:
                src = src_row + x * 2
                value = int.from_bytes(data[src : src + 2], byteorder="little", signed=False)
                value = min(255, value // 256)
                rgb[dst : dst + 3] = bytes([value, value, value])
            elif encoding in {"32fc1"}:
                src = src_row + x * 4
                value = struct.unpack_from("<f", data, src)[0]
                value = 0 if not math.isfinite(value) else max(0, min(255, int(value * 40)))
                rgb[dst : dst + 3] = bytes([value, value, value])
            else:
                raise ValueError("unsupported image encoding: %s" % message.encoding)
    return bytes(rgb)


def convert_to_rgb_cv(message, sample_step):
    try:
        import cv2
        import numpy as np
    except Exception:
        return None

    encoding = (message.encoding or "").lower()
    data = bytes(message.data)
    width = int(message.width)
    height = int(message.height)
    row_step = int(message.step) if message.step else width
    sample_step = max(1, int(sample_step))

    try:
        if encoding in {"rgb8", "bgr8", "8uc3"}:
            channels = 3
            rows = np.frombuffer(data, dtype=np.uint8).reshape(height, row_step)
            image = rows[:, : width * channels].reshape(height, width, channels)
            sampled = image[::sample_step, ::sample_step]
            if encoding == "bgr8":
                sampled = cv2.cvtColor(sampled, cv2.COLOR_BGR2RGB)
            return sampled.astype(np.uint8, copy=False).tobytes()

        if encoding in {"mono8", "8uc1"}:
            rows = np.frombuffer(data, dtype=np.uint8).reshape(height, row_step)
            sampled = rows[::sample_step, :width][:, ::sample_step]
            return cv2.cvtColor(sampled, cv2.COLOR_GRAY2RGB).tobytes()

        if encoding in {"mono16", "16uc1"}:
            row_values = row_step // 2
            rows = np.frombuffer(data, dtype="<u2").reshape(height, row_values)
            sampled = rows[::sample_step, :width][:, ::sample_step]
            return colorize_depth(sampled, cv2, np).tobytes()

        if encoding == "32fc1":
            row_values = row_step // 4
            rows = np.frombuffer(data, dtype="<f4").reshape(height, row_values)
            sampled = rows[::sample_step, :width][:, ::sample_step]
            return colorize_depth(sampled, cv2, np).tobytes()
    except Exception as exc:
        raise ValueError("failed to convert image with cv2: %s" % exc)
    return None


def colorize_depth(values, cv2, np):
    finite = np.isfinite(values)
    positive = finite & (values > 0)
    if positive.any():
        valid = values[positive].astype(np.float32)
        near = float(np.percentile(valid, 2))
        far = float(np.percentile(valid, 98))
        if far <= near:
            far = near + 1.0
        normalized = 255.0 - np.clip((values.astype(np.float32) - near) * (255.0 / (far - near)), 0, 255)
        normalized[~positive] = 0
    else:
        normalized = np.zeros(values.shape, dtype=np.float32)
    mono = normalized.astype(np.uint8)
    color = cv2.applyColorMap(mono, cv2.COLORMAP_TURBO if hasattr(cv2, "COLORMAP_TURBO") else cv2.COLORMAP_JET)
    color[~positive] = (0, 0, 0)
    return cv2.cvtColor(color, cv2.COLOR_BGR2RGB)


def encode_image(rgb, width, height, quality):
    try:
        from PIL import Image
        import io

        image = Image.frombytes("RGB", (width, height), rgb)
        output = io.BytesIO()
        image.save(output, format="JPEG", quality=max(25, min(85, int(quality))), optimize=False)
        return output.getvalue(), "image/jpeg"
    except Exception:
        return rgb_to_bmp(rgb, width, height), "image/bmp"


def rgb_to_bmp(rgb, width, height):
    row_stride = ((width * 3 + 3) // 4) * 4
    pixel_size = row_stride * height
    file_size = 54 + pixel_size
    header = bytearray()
    header.extend(b"BM")
    header.extend(struct.pack("<I", file_size))
    header.extend(b"\x00\x00\x00\x00")
    header.extend(struct.pack("<I", 54))
    header.extend(struct.pack("<I", 40))
    header.extend(struct.pack("<i", width))
    header.extend(struct.pack("<i", height))
    header.extend(struct.pack("<H", 1))
    header.extend(struct.pack("<H", 24))
    header.extend(struct.pack("<I", 0))
    header.extend(struct.pack("<I", pixel_size))
    header.extend(struct.pack("<i", 2835))
    header.extend(struct.pack("<i", 2835))
    header.extend(struct.pack("<I", 0))
    header.extend(struct.pack("<I", 0))

    body = bytearray()
    padding = b"\x00" * (row_stride - width * 3)
    for y in range(height - 1, -1, -1):
        row = y * width * 3
        for x in range(width):
            src = row + x * 3
            body.extend([rgb[src + 2], rgb[src + 1], rgb[src]])
        body.extend(padding)
    return bytes(header + body)


if __name__ == "__main__":
    sys.exit(main())

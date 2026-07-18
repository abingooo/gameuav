#!/usr/bin/env python3
import os
import sys
import time
from collections import deque

import rospy
from sensor_msgs.msg import Image, Imu


TOPICS = [
    ("/camera/infra1/image_rect_raw", Image, 20.0, "infra1"),
    ("/camera/infra2/image_rect_raw", Image, 20.0, "infra2"),
    ("/mavros/imu/data", Imu, 80.0, "imu"),
]


def _rate(samples):
    if len(samples) < 2:
        return 0.0
    duration = samples[-1] - samples[0]
    if duration <= 0.0:
        return 0.0
    return float(len(samples) - 1) / duration


def main():
    if len(sys.argv) < 2:
        print("wait_for_vins_inputs.py must be used as a roslaunch launch-prefix", file=sys.stderr)
        return 2

    timeout = float(os.environ.get("GAMEUAV_VINS_INPUT_WAIT_TIMEOUT", "45.0"))
    stable_for = float(os.environ.get("GAMEUAV_VINS_INPUT_STABLE_FOR", "2.0"))
    max_age = float(os.environ.get("GAMEUAV_VINS_INPUT_MAX_AGE", "0.5"))
    started_at = time.time()

    rospy.init_node("wait_for_vins_inputs", anonymous=True, disable_signals=True)
    samples = {name: deque(maxlen=256) for _, _, _, name in TOPICS}

    def callback(name):
        def _cb(_msg):
            samples[name].append(time.time())

        return _cb

    subscribers = [
        rospy.Subscriber(topic, msg_type, callback(name), queue_size=20)
        for topic, msg_type, _min_rate, name in TOPICS
    ]

    print("[wait_for_vins_inputs] waiting for stable VINS inputs before starting VINS", flush=True)
    stable_since = None
    last_report = 0.0
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        now = time.time()
        status = []
        ok = True
        for _topic, _msg_type, min_rate, name in TOPICS:
            topic_samples = samples[name]
            hz = _rate(topic_samples)
            age = float("inf") if not topic_samples else now - topic_samples[-1]
            item_ok = len(topic_samples) >= 3 and hz >= min_rate and age <= max_age
            ok = ok and item_ok
            status.append("%s %.1fHz age %.2fs min %.1fHz" % (name, hz, age, min_rate))

        if ok:
            if stable_since is None:
                stable_since = now
            if now - stable_since >= stable_for:
                for sub in subscribers:
                    sub.unregister()
                print("[wait_for_vins_inputs] stable: %s" % "; ".join(status), flush=True)
                os.execvp(sys.argv[1], sys.argv[1:])
        else:
            stable_since = None

        if now - last_report >= 1.0:
            print("[wait_for_vins_inputs] %s" % "; ".join(status), flush=True)
            last_report = now

        if now - started_at > timeout:
            print("[wait_for_vins_inputs] timeout after %.1fs: %s" % (timeout, "; ".join(status)), file=sys.stderr)
            return 1

        rate.sleep()

    return 1


if __name__ == "__main__":
    sys.exit(main())

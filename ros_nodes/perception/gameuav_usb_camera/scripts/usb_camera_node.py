#!/usr/bin/env python3

import time

import cv2
import rospy
from sensor_msgs.msg import Image


def make_image(frame, frame_id):
    height, width = frame.shape[:2]
    msg = Image()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.height = height
    msg.width = width
    msg.encoding = "bgr8"
    msg.is_bigendian = False
    msg.step = width * 3
    msg.data = frame.tobytes()
    return msg


def open_camera(device, width, height, fps, fourcc):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError("failed to open camera device: %s" % device)
    if fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
    cap.set(cv2.CAP_PROP_FPS, float(fps))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def main():
    rospy.init_node("gameuav_usb_camera")
    device = rospy.get_param("~device", "/dev/video0")
    width = int(rospy.get_param("~width", 640))
    height = int(rospy.get_param("~height", 480))
    fps = float(rospy.get_param("~fps", 30.0))
    frame_id = rospy.get_param("~frame_id", "usb_camera")
    fourcc = rospy.get_param("~fourcc", "MJPG")
    topic = rospy.get_param("~image_topic", "image_raw")

    publisher = rospy.Publisher(topic, Image, queue_size=1)
    cap = open_camera(device, width, height, fps, fourcc)
    period = 1.0 / max(1.0, fps)
    rospy.loginfo(
        "USB camera publishing %s from %s at %dx%d %.1f FPS",
        topic,
        device,
        width,
        height,
        fps,
    )

    try:
        while not rospy.is_shutdown():
            started = time.monotonic()
            ok, frame = cap.read()
            if ok and frame is not None:
                publisher.publish(make_image(frame, frame_id))
            else:
                rospy.logwarn_throttle(2.0, "failed to read frame from %s", device)
                time.sleep(0.05)
            elapsed = time.monotonic() - started
            if elapsed < period:
                time.sleep(period - elapsed)
    finally:
        cap.release()


if __name__ == "__main__":
    main()

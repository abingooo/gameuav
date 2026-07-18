#!/usr/bin/env python3

import argparse
import os
import signal
import subprocess
import sys
import time

import rosgraph
import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong


class FcuStateMonitor:
    def __init__(self):
        self.state = None
        self.last_message_at = None
        self.subscriber = rospy.Subscriber("/mavros/state", State, self._callback, queue_size=1)

    def _callback(self, message):
        self.state = message
        self.last_message_at = time.monotonic()

    def connected(self, max_message_age=2.5):
        return bool(
            self.state
            and self.state.connected
            and self.last_message_at is not None
            and time.monotonic() - self.last_message_at <= max_message_age
        )


def wait_until(predicate, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not rospy.is_shutdown():
        if predicate():
            return
        time.sleep(0.05)
    raise RuntimeError("timed out waiting for %s" % description)


def stop_process_group(proc):
    if proc.poll() is not None:
        return
    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    try:
        proc.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        try:
            proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            proc.wait()


def reboot_fcu(args):
    # An autostart launch may still be bringing up roscore when the agent starts.
    master_deadline = time.monotonic() + 5.0
    while time.monotonic() < master_deadline and not rosgraph.is_master_online():
        time.sleep(0.1)

    command = [
        "roslaunch",
        "launch/bringup_mavros.launch",
        "fcu_url:=%s" % args.fcu_url,
        "configure_stream_rates:=false",
        "respawn_mavros:=true",
    ]
    mavros = subprocess.Popen(command, preexec_fn=os.setsid)
    try:
        rospy.init_node("egoctrl_fcu_reboot", anonymous=True, disable_signals=True)
        monitor = FcuStateMonitor()

        wait_until(monitor.connected, args.connect_timeout, "initial FCU connection")
        if monitor.state.armed:
            raise RuntimeError("refusing to reboot an armed FCU")

        rospy.wait_for_service("/mavros/cmd/command", timeout=args.connect_timeout)
        reboot = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)
        response = reboot(
            broadcast=False,
            command=246,
            confirmation=1,
            param1=1.0,
            param2=0.0,
            param3=0.0,
            param4=0.0,
            param5=0.0,
            param6=0.0,
            param7=0.0,
        )
        if not response.success:
            raise RuntimeError("PX4 rejected reboot command with MAV_RESULT=%d" % response.result)

        wait_until(lambda: not monitor.connected(), args.disconnect_timeout, "FCU disconnect")
        wait_until(monitor.connected, args.reconnect_timeout, "FCU reconnect")
        if monitor.state.armed:
            raise RuntimeError("FCU unexpectedly reports armed after reboot")

        print("FCU reboot completed; settling for %.1fs" % args.settle_time, flush=True)
        time.sleep(args.settle_time)
    finally:
        stop_process_group(mavros)


def build_parser():
    parser = argparse.ArgumentParser(description="Safely reboot PX4 before starting EgoCtrl")
    parser.add_argument(
        "--fcu-url",
        default="/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00:57600",
    )
    parser.add_argument("--connect-timeout", type=float, default=15.0)
    parser.add_argument("--disconnect-timeout", type=float, default=10.0)
    parser.add_argument("--reconnect-timeout", type=float, default=20.0)
    parser.add_argument("--settle-time", type=float, default=3.0)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        reboot_fcu(args)
    except (RuntimeError, rospy.ROSException, rospy.ServiceException, OSError) as exc:
        print("FCU pre-start reboot failed: %s" % exc, file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

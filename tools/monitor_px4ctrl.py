#!/usr/bin/env python3

"""Read-only terminal monitor for px4ctrl and its critical ROS inputs."""

import argparse
import csv
import math
import os
import signal
import sys
import time


def add_workspace_python_path():
    workspace = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    paths = [
        os.path.join(workspace, "devel", "lib", "python3", "dist-packages"),
        "/opt/ros/noetic/lib/python3/dist-packages",
    ]
    for dist_packages in paths:
        if os.path.isdir(dist_packages) and dist_packages not in sys.path:
            sys.path.insert(0, dist_packages)


def finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def age(now, sample):
    return math.inf if sample is None else max(0.0, now - sample[0])


def fmt(value, digits=2):
    return "--" if not finite(value) else ("%.*f" % (digits, value))


def plausible(value, limit):
    return finite(value) and abs(value) <= limit


class Monitor:
    def __init__(self, rospy, args, message_types):
        self.rospy = rospy
        self.args = args
        self.msg = message_types
        self.samples = {}
        self.last_alerts = set()
        self.last_mode = None
        self.last_armed = None
        self.status_text = ""
        self.csv_file = None
        self.csv_writer = None
        self.started = time.monotonic()

        topics = [
            ("state", args.state_topic, message_types["State"]),
            ("extended", args.extended_state_topic, message_types["ExtendedState"]),
            ("odom", args.odom_topic, message_types["Odometry"]),
            ("cmd", args.command_topic, message_types["PositionCommand"]),
            ("attitude", args.attitude_topic, message_types["AttitudeTarget"]),
            ("battery", args.battery_topic, message_types["BatteryState"]),
            ("status", args.status_text_topic, message_types["StatusText"]),
            ("debug", args.debug_topic, message_types["Px4ctrlDebug"]),
        ]
        self.subscribers = [
            rospy.Subscriber(topic, msg_type, self.callback, callback_args=name, queue_size=20)
            for name, topic, msg_type in topics
        ]

        if args.csv:
            parent = os.path.dirname(os.path.abspath(args.csv))
            os.makedirs(parent, exist_ok=True)
            self.csv_file = open(args.csv, "a", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            if self.csv_file.tell() == 0:
                self.csv_writer.writerow([
                    "wall_time", "mode", "armed", "connected", "landed_state",
                    "odom_age_s", "z_m", "vz_mps", "cmd_age_s", "cmd_z_m",
                    "thrust", "setpoint_age_s", "battery_v", "battery_pct", "alerts",
                ])

    def callback(self, message, name):
        now = time.monotonic()
        self.samples[name] = (now, message)
        if name == "status":
            self.status_text = message.text.strip()

    def get(self, name):
        sample = self.samples.get(name)
        return None if sample is None else sample[1]

    def snapshot(self):
        now = time.monotonic()
        state = self.get("state")
        extended = self.get("extended")
        odom = self.get("odom")
        cmd = self.get("cmd")
        attitude = self.get("attitude")
        battery = self.get("battery")
        debug = self.get("debug")

        connected = bool(state and state.connected)
        armed = bool(state and state.armed)
        mode = state.mode if state else "NO_STATE"
        landed = extended.landed_state if extended else -1
        odom_age = age(now, self.samples.get("odom"))
        cmd_age = age(now, self.samples.get("cmd"))
        setpoint_age = age(now, self.samples.get("attitude"))
        z = odom.pose.pose.position.z if odom else math.nan
        vz = odom.twist.twist.linear.z if odom else math.nan
        cmd_z = cmd.position.z if cmd else math.nan
        thrust = attitude.thrust if attitude else math.nan
        if debug and finite(debug.des_thr):
            thrust = debug.des_thr
        voltage = battery.voltage if battery else math.nan
        percentage = battery.percentage * 100.0 if battery and finite(battery.percentage) else math.nan

        odom_valid = plausible(z, self.args.max_abs_position) and plausible(vz, self.args.max_abs_velocity)
        thrust_valid = not finite(thrust) or 0.0 <= thrust <= 1.0
        battery_valid = not finite(voltage) or (not armed and voltage == 0.0) or 1.0 < voltage < 100.0

        alerts = []
        if not connected:
            alerts.append("FCU_DISCONNECTED")
        if odom_age > self.args.odom_timeout:
            alerts.append("ODOM_STALE")
        if odom and not odom_valid:
            alerts.append("ODOM_INVALID")
            z = math.nan
            vz = math.nan
        if attitude and not thrust_valid:
            alerts.append("THRUST_INVALID")
            thrust = math.nan
        if battery and not battery_valid:
            alerts.append("BATTERY_INVALID")
            voltage = math.nan
            percentage = math.nan
        elif voltage == 0.0:
            voltage = math.nan
            percentage = math.nan
        if armed and mode == "OFFBOARD" and setpoint_age > self.args.setpoint_timeout:
            alerts.append("SETPOINT_STALE")
        if armed and mode == "OFFBOARD" and finite(thrust) and thrust < self.args.low_thrust:
            alerts.append("LOW_THRUST")
        if armed and finite(vz) and vz < -self.args.descent_speed:
            alerts.append("FAST_DESCENT")
        if armed and finite(voltage) and 1.0 < voltage < self.args.low_voltage:
            alerts.append("LOW_VOLTAGE")
        if state and not state.guided and mode == "OFFBOARD":
            alerts.append("OFFBOARD_NOT_GUIDED")

        return {
            "now": now, "mode": mode, "armed": armed, "connected": connected,
            "landed": landed, "odom_age": odom_age, "z": z, "vz": vz,
            "cmd_age": cmd_age, "cmd_z": cmd_z, "setpoint_age": setpoint_age,
            "thrust": thrust, "voltage": voltage, "percentage": percentage,
            "alerts": alerts, "debug_seen": debug is not None,
        }

    def display(self, data):
        state_changed = data["mode"] != self.last_mode or data["armed"] != self.last_armed
        alert_set = set(data["alerts"])
        new_alerts = alert_set - self.last_alerts
        if state_changed:
            print("\n[STATE] mode=%s armed=%s connected=%s landed=%s" % (
                data["mode"], data["armed"], data["connected"], data["landed"]
            ))
        if new_alerts:
            print("\n[ALERT] %s" % ", ".join(sorted(new_alerts)))
        self.last_mode = data["mode"]
        self.last_armed = data["armed"]
        self.last_alerts = alert_set

        line = (
            "mode=%-10s arm=%-3s z=%7sm vz=%7sm/s odom=%5ss "
            "cmd_z=%7sm cmd=%5ss thrust=%5s sp=%5ss bat=%6sV %6s%% [%s]"
        ) % (
            data["mode"], "YES" if data["armed"] else "no",
            fmt(data["z"]), fmt(data["vz"]), fmt(data["odom_age"]),
            fmt(data["cmd_z"]), fmt(data["cmd_age"]), fmt(data["thrust"], 3),
            fmt(data["setpoint_age"]), fmt(data["voltage"]), fmt(data["percentage"], 1),
            ",".join(data["alerts"]) or "OK",
        )
        if sys.stdout.isatty():
            sys.stdout.write("\r\033[2K" + line)
            sys.stdout.flush()
        else:
            print(line)

        if self.csv_writer:
            self.csv_writer.writerow([
                time.strftime("%Y-%m-%dT%H:%M:%S%z"), data["mode"], int(data["armed"]),
                int(data["connected"]), data["landed"], fmt(data["odom_age"], 3),
                fmt(data["z"], 4), fmt(data["vz"], 4), fmt(data["cmd_age"], 3),
                fmt(data["cmd_z"], 4), fmt(data["thrust"], 4),
                fmt(data["setpoint_age"], 3), fmt(data["voltage"], 3),
                fmt(data["percentage"], 1), "|".join(data["alerts"]),
            ])
            self.csv_file.flush()

    def close(self):
        for subscriber in self.subscribers:
            subscriber.unregister()
        if self.csv_file:
            self.csv_file.close()
        if sys.stdout.isatty():
            print()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Read-only live monitor for px4ctrl, MAVROS, and VINS.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--rate", type=float, default=2.0, help="terminal/CSV update rate")
    parser.add_argument("--duration", type=float, default=0.0, help="stop after N seconds; 0 runs until Ctrl-C")
    parser.add_argument("--csv", help="append samples to this CSV file")
    parser.add_argument("--odom-timeout", type=float, default=0.5)
    parser.add_argument("--setpoint-timeout", type=float, default=0.3)
    parser.add_argument("--low-thrust", type=float, default=0.10)
    parser.add_argument("--descent-speed", type=float, default=0.5)
    parser.add_argument("--low-voltage", type=float, default=13.8)
    parser.add_argument("--max-abs-position", type=float, default=1000.0, help="reject odometry beyond this many metres")
    parser.add_argument("--max-abs-velocity", type=float, default=50.0, help="reject odometry speed beyond this many m/s")
    parser.add_argument("--state-topic", default="/mavros/state")
    parser.add_argument("--extended-state-topic", default="/mavros/extended_state")
    parser.add_argument("--odom-topic", default="/vins_fusion/imu_propagate")
    parser.add_argument("--command-topic", default="/control/position_cmd")
    parser.add_argument("--attitude-topic", default="/mavros/setpoint_raw/attitude")
    parser.add_argument("--battery-topic", default="/mavros/battery")
    parser.add_argument("--status-text-topic", default="/mavros/statustext/recv")
    parser.add_argument("--debug-topic", default="/debugPx4ctrl")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    add_workspace_python_path()
    try:
        import rospy
        from mavros_msgs.msg import AttitudeTarget, ExtendedState, State, StatusText
        from nav_msgs.msg import Odometry
        from quadrotor_msgs.msg import PositionCommand, Px4ctrlDebug
        from sensor_msgs.msg import BatteryState
    except ImportError as exc:
        print("ROS message import failed: %s" % exc, file=sys.stderr)
        print("Run after sourcing /opt/ros/noetic/setup.bash and this workspace/devel/setup.bash.", file=sys.stderr)
        return 2

    message_types = locals()
    try:
        rospy.init_node("px4ctrl_monitor", anonymous=True, disable_signals=True)
    except Exception as exc:
        print("Cannot connect to ROS master: %s" % exc, file=sys.stderr)
        return 2

    monitor = Monitor(rospy, args, message_types)
    stop = [False]

    def request_stop(_signum, _frame):
        stop[0] = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    period = 1.0 / max(0.1, args.rate)
    try:
        while not stop[0] and not rospy.is_shutdown():
            data = monitor.snapshot()
            monitor.display(data)
            if args.duration > 0 and time.monotonic() - monitor.started >= args.duration:
                break
            time.sleep(period)
    finally:
        monitor.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

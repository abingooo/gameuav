#!/usr/bin/python3

"""Record the evidence needed to diagnose VINS divergence during flight."""

import csv
from datetime import datetime
import hashlib
import json
import math
import os
import subprocess
import threading
import time


CSV_FIELDS = [
    "wall_time", "ros_time", "armed", "mode",
    "opt_age_s", "opt_hz", "opt_stamp_age_s",
    "opt_x_m", "opt_y_m", "opt_z_m", "opt_vx_mps", "opt_vy_mps", "opt_vz_mps",
    "prop_age_s", "prop_hz", "prop_stamp_age_s",
    "prop_x_m", "prop_y_m", "prop_z_m", "prop_vx_mps", "prop_vy_mps", "prop_vz_mps",
    "px4_age_s", "px4_hz", "px4_x_m", "px4_y_m", "px4_z_m",
    "aligned_dx_m", "aligned_dy_m", "aligned_dz_m", "aligned_error_m",
    "prop_opt_error_m", "prop_opt_speed_error_mps",
    "imu_age_s", "imu_hz", "imu_stamp_age_s",
    "acc_x_mps2", "acc_y_mps2", "acc_z_mps2", "acc_norm_mps2", "acc_peak_mps2",
    "gyro_x_rps", "gyro_y_rps", "gyro_z_rps", "gyro_norm_rps", "gyro_peak_rps",
    "left_age_s", "left_hz", "left_stamp_age_s",
    "right_age_s", "right_hz", "right_stamp_age_s", "stereo_stamp_delta_s",
    "feature_age_s", "feature_hz", "feature_count", "alerts",
]


def finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def norm3(values):
    if not values or not all(finite(value) for value in values):
        return math.nan
    return math.sqrt(sum(value * value for value in values))


def vector_difference(left, right):
    if (
        not left
        or not right
        or len(left) != 3
        or len(right) != 3
        or not all(finite(value) for value in left + right)
    ):
        return None
    return [left[index] - right[index] for index in range(3)]


def csv_value(value):
    if value is None:
        return ""
    if isinstance(value, float) and not math.isfinite(value):
        return ""
    return value


def json_safe(value):
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


class VinsHealthMonitor:
    def __init__(self, rospy, message_types):
        self.rospy = rospy
        self.types = message_types
        self.lock = threading.Lock()
        self.samples = {}
        self.last_receive = {}
        self.rates = {}
        self.imu_peaks = {"acc": math.nan, "gyro": math.nan}
        self.baseline = None
        self.active_alerts = set()
        self.last_armed = None
        self.last_mode = None
        self.started_mono = time.monotonic()

        self.output_root = rospy.get_param(
            "~output_root",
            "/home/uav/Desktop/uav_project/gameuav/runtime/vins_monitor",
        )
        self.sample_rate = max(1.0, float(rospy.get_param("~sample_rate", 10.0)))
        self.config_path = rospy.get_param(
            "~config_path",
            "/home/uav/Desktop/uav_project/gameuav/ros_nodes/state_estimation/"
            "VINS-Fusion/config/fast_drone_250.yaml",
        )
        self.thresholds = {
            "odom_stale_s": float(rospy.get_param("~odom_stale_s", 0.5)),
            "sensor_stale_s": float(rospy.get_param("~sensor_stale_s", 0.5)),
            "feature_stale_s": float(rospy.get_param("~feature_stale_s", 1.0)),
            "min_features": int(rospy.get_param("~min_features", 15)),
            "position_limit_m": float(rospy.get_param("~position_limit_m", 20.0)),
            "speed_limit_mps": float(rospy.get_param("~speed_limit_mps", 8.0)),
            "aligned_error_m": float(rospy.get_param("~aligned_error_m", 1.0)),
            "prop_opt_error_m": float(rospy.get_param("~prop_opt_error_m", 0.5)),
            "acc_peak_mps2": float(rospy.get_param("~acc_peak_mps2", 30.0)),
            "gyro_peak_rps": float(rospy.get_param("~gyro_peak_rps", 6.0)),
            "stereo_skew_s": float(rospy.get_param("~stereo_skew_s", 0.05)),
            "stamp_age_s": float(rospy.get_param("~stamp_age_s", 0.5)),
            "startup_grace_s": float(rospy.get_param("~startup_grace_s", 2.0)),
        }

        run_name = "%s_%d" % (datetime.now().strftime("%Y%m%d_%H%M%S"), os.getpid())
        self.run_dir = os.path.join(self.output_root, run_name)
        os.makedirs(self.run_dir, exist_ok=True)
        self.csv_path = os.path.join(self.run_dir, "samples.csv")
        self.events_path = os.path.join(self.run_dir, "events.jsonl")
        self.metadata_path = os.path.join(self.run_dir, "metadata.json")
        self.csv_file = open(self.csv_path, "w", encoding="utf-8", newline="")
        self.events_file = open(self.events_path, "a", encoding="utf-8")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
        self.writer.writeheader()
        self.rows_since_flush = 0
        self._write_metadata()
        self._update_latest_link(run_name)

        topics = {
            "state": (rospy.get_param("~state_topic", "/mavros/state"), message_types["State"], self._state_values),
            "opt": (rospy.get_param("~optimized_odom_topic", "/vins_fusion/odometry"), message_types["Odometry"], self._odom_values),
            "prop": (rospy.get_param("~propagated_odom_topic", "/vins_fusion/imu_propagate"), message_types["Odometry"], self._odom_values),
            "px4": (rospy.get_param("~px4_pose_topic", "/mavros/local_position/pose"), message_types["PoseStamped"], self._pose_values),
            "imu": (rospy.get_param("~imu_topic", "/mavros/imu/data"), message_types["Imu"], self._imu_values),
            "left": (rospy.get_param("~left_image_topic", "/camera/infra1/image_rect_raw"), message_types["Image"], self._image_values),
            "right": (rospy.get_param("~right_image_topic", "/camera/infra2/image_rect_raw"), message_types["Image"], self._image_values),
            "feature": (rospy.get_param("~feature_topic", "/vins_fusion/point_cloud"), message_types["PointCloud"], self._feature_values),
        }
        self.topic_names = {name: topic for name, (topic, _type, _extractor) in topics.items()}
        self.subscribers = [
            rospy.Subscriber(
                topic,
                message_type,
                self._callback,
                callback_args=(name, extractor),
                queue_size=50,
                tcp_nodelay=name in {"opt", "prop", "px4", "imu"},
            )
            for name, (topic, message_type, extractor) in topics.items()
        ]
        self._event("monitor_started", {"run_dir": self.run_dir, "topics": self.topic_names})
        rospy.loginfo("[vins_health_monitor] logging to %s", self.run_dir)

    def _write_metadata(self):
        metadata = {
            "schema": "gameuav.vins_monitor.v1",
            "started_at": datetime.now().astimezone().isoformat(),
            "pid": os.getpid(),
            "config_path": self.config_path,
            "config_sha256": self._sha256(self.config_path),
            "git_revision": self._git_revision(),
            "thresholds": self.thresholds,
            "csv_fields": CSV_FIELDS,
        }
        with open(self.metadata_path, "w", encoding="utf-8") as stream:
            json.dump(metadata, stream, ensure_ascii=True, indent=2, sort_keys=True)
            stream.write("\n")

    def _update_latest_link(self, run_name):
        os.makedirs(self.output_root, exist_ok=True)
        latest = os.path.join(self.output_root, "latest")
        temporary = latest + ".tmp.%d" % os.getpid()
        try:
            os.symlink(run_name, temporary)
            os.replace(temporary, latest)
        except OSError:
            try:
                if os.path.lexists(temporary):
                    os.unlink(temporary)
            except OSError:
                pass

    @staticmethod
    def _sha256(path):
        try:
            digest = hashlib.sha256()
            with open(path, "rb") as stream:
                for chunk in iter(lambda: stream.read(65536), b""):
                    digest.update(chunk)
            return digest.hexdigest()
        except OSError:
            return None

    @staticmethod
    def _git_revision():
        try:
            return subprocess.check_output(
                ["git", "rev-parse", "HEAD"],
                cwd="/home/uav/Desktop/uav_project/gameuav",
                stderr=subprocess.DEVNULL,
                text=True,
                timeout=2.0,
            ).strip()
        except (OSError, subprocess.SubprocessError):
            return None

    @staticmethod
    def _header_stamp(message):
        header = getattr(message, "header", None)
        return header.stamp.to_sec() if header is not None else math.nan

    @staticmethod
    def _state_values(message):
        return {"armed": bool(message.armed), "mode": str(message.mode)}

    @staticmethod
    def _odom_values(message):
        position = message.pose.pose.position
        velocity = message.twist.twist.linear
        orientation = message.pose.pose.orientation
        return {
            "p": [position.x, position.y, position.z],
            "v": [velocity.x, velocity.y, velocity.z],
            "q": [orientation.x, orientation.y, orientation.z, orientation.w],
        }

    @staticmethod
    def _pose_values(message):
        position = message.pose.position
        orientation = message.pose.orientation
        return {
            "p": [position.x, position.y, position.z],
            "q": [orientation.x, orientation.y, orientation.z, orientation.w],
        }

    @staticmethod
    def _imu_values(message):
        acceleration = message.linear_acceleration
        angular_velocity = message.angular_velocity
        return {
            "acc": [acceleration.x, acceleration.y, acceleration.z],
            "gyro": [angular_velocity.x, angular_velocity.y, angular_velocity.z],
        }

    @staticmethod
    def _image_values(message):
        return {"seq": message.header.seq, "width": message.width, "height": message.height}

    @staticmethod
    def _feature_values(message):
        return {"count": len(message.points), "channels": len(message.channels)}

    def _callback(self, message, callback_args):
        name, extractor = callback_args
        received = time.monotonic()
        values = extractor(message)
        stamp = self._header_stamp(message)
        with self.lock:
            previous = self.last_receive.get(name)
            if previous is not None and received > previous:
                instant_rate = 1.0 / (received - previous)
                old_rate = self.rates.get(name)
                self.rates[name] = instant_rate if old_rate is None else 0.15 * instant_rate + 0.85 * old_rate
            self.last_receive[name] = received
            self.samples[name] = {"received": received, "stamp": stamp, "values": values}
            if name == "imu":
                acc_norm = norm3(values["acc"])
                gyro_norm = norm3(values["gyro"])
                if finite(acc_norm):
                    self.imu_peaks["acc"] = max(acc_norm, self.imu_peaks["acc"]) if finite(self.imu_peaks["acc"]) else acc_norm
                if finite(gyro_norm):
                    self.imu_peaks["gyro"] = max(gyro_norm, self.imu_peaks["gyro"]) if finite(self.imu_peaks["gyro"]) else gyro_norm

    def _snapshot_samples(self):
        with self.lock:
            samples = {name: dict(sample) for name, sample in self.samples.items()}
            rates = dict(self.rates)
            peaks = dict(self.imu_peaks)
            self.imu_peaks = {"acc": math.nan, "gyro": math.nan}
        return samples, rates, peaks

    @staticmethod
    def _age(sample, now_mono):
        return math.inf if sample is None else max(0.0, now_mono - sample["received"])

    @staticmethod
    def _stamp_age(sample, ros_now):
        if sample is None or not finite(sample.get("stamp")):
            return math.inf
        return ros_now - sample["stamp"]

    @staticmethod
    def _sample_vector(sample, key):
        if sample is None:
            return None
        return sample["values"].get(key)

    def build_row(self):
        now_mono = time.monotonic()
        ros_now = self.rospy.Time.now().to_sec()
        samples, rates, peaks = self._snapshot_samples()
        state = samples.get("state")
        opt = samples.get("opt")
        prop = samples.get("prop")
        px4 = samples.get("px4")
        imu = samples.get("imu")
        left = samples.get("left")
        right = samples.get("right")
        feature = samples.get("feature")

        opt_p = self._sample_vector(opt, "p")
        opt_v = self._sample_vector(opt, "v")
        prop_p = self._sample_vector(prop, "p")
        prop_v = self._sample_vector(prop, "v")
        px4_p = self._sample_vector(px4, "p")
        acc = self._sample_vector(imu, "acc")
        gyro = self._sample_vector(imu, "gyro")

        vins_px4 = vector_difference(opt_p, px4_p)
        armed = bool(state and state["values"].get("armed"))
        if self.baseline is None and vins_px4 and not armed:
            self.baseline = vins_px4
            self._event("alignment_baseline_set", {"vins_minus_px4_m": self.baseline})
        aligned = vector_difference(vins_px4, self.baseline) if vins_px4 and self.baseline else None
        prop_opt = vector_difference(prop_p, opt_p)
        prop_opt_v = vector_difference(prop_v, opt_v)
        stereo_delta = (
            left["stamp"] - right["stamp"]
            if left and right and finite(left.get("stamp")) and finite(right.get("stamp"))
            else math.nan
        )

        row = {field: None for field in CSV_FIELDS}
        row.update({
            "wall_time": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "ros_time": ros_now,
            "armed": int(armed),
            "mode": state["values"].get("mode", "") if state else "",
            "opt_age_s": self._age(opt, now_mono), "opt_hz": rates.get("opt"), "opt_stamp_age_s": self._stamp_age(opt, ros_now),
            "prop_age_s": self._age(prop, now_mono), "prop_hz": rates.get("prop"), "prop_stamp_age_s": self._stamp_age(prop, ros_now),
            "px4_age_s": self._age(px4, now_mono), "px4_hz": rates.get("px4"),
            "aligned_error_m": norm3(aligned), "prop_opt_error_m": norm3(prop_opt), "prop_opt_speed_error_mps": norm3(prop_opt_v),
            "imu_age_s": self._age(imu, now_mono), "imu_hz": rates.get("imu"), "imu_stamp_age_s": self._stamp_age(imu, ros_now),
            "acc_norm_mps2": norm3(acc), "acc_peak_mps2": peaks["acc"],
            "gyro_norm_rps": norm3(gyro), "gyro_peak_rps": peaks["gyro"],
            "left_age_s": self._age(left, now_mono), "left_hz": rates.get("left"), "left_stamp_age_s": self._stamp_age(left, ros_now),
            "right_age_s": self._age(right, now_mono), "right_hz": rates.get("right"), "right_stamp_age_s": self._stamp_age(right, ros_now),
            "stereo_stamp_delta_s": stereo_delta,
            "feature_age_s": self._age(feature, now_mono), "feature_hz": rates.get("feature"),
            "feature_count": feature["values"].get("count") if feature else None,
        })
        for prefix, values in (("opt", opt_p), ("prop", prop_p), ("px4", px4_p)):
            if values:
                row.update({"%s_x_m" % prefix: values[0], "%s_y_m" % prefix: values[1], "%s_z_m" % prefix: values[2]})
        for prefix, values in (("opt", opt_v), ("prop", prop_v)):
            if values:
                row.update({"%s_vx_mps" % prefix: values[0], "%s_vy_mps" % prefix: values[1], "%s_vz_mps" % prefix: values[2]})
        if aligned:
            row.update({"aligned_dx_m": aligned[0], "aligned_dy_m": aligned[1], "aligned_dz_m": aligned[2]})
        if acc:
            row.update({"acc_x_mps2": acc[0], "acc_y_mps2": acc[1], "acc_z_mps2": acc[2]})
        if gyro:
            row.update({"gyro_x_rps": gyro[0], "gyro_y_rps": gyro[1], "gyro_z_rps": gyro[2]})

        alerts = self._alerts(row)
        row["alerts"] = "|".join(sorted(alerts))
        self._record_alert_changes(alerts, row)
        self._record_flight_state(row)
        return row

    def _alerts(self, row):
        threshold = self.thresholds
        alerts = set()
        startup_grace = time.monotonic() - self.started_mono < threshold["startup_grace_s"]
        for prefix in ("opt", "prop"):
            if not startup_grace and row["%s_age_s" % prefix] > threshold["odom_stale_s"]:
                alerts.add("%s_ODOM_STALE" % prefix.upper())
            position = [row["%s_%s_m" % (prefix, axis)] for axis in "xyz"]
            velocity = [row["%s_v%s_mps" % (prefix, axis)] for axis in "xyz"]
            if any(value is not None and not finite(value) for value in position + velocity):
                alerts.add("%s_ODOM_NONFINITE" % prefix.upper())
            if finite(norm3(position)) and norm3(position) > threshold["position_limit_m"]:
                alerts.add("%s_POSITION_DIVERGED" % prefix.upper())
            if finite(norm3(velocity)) and norm3(velocity) > threshold["speed_limit_mps"]:
                alerts.add("%s_SPEED_DIVERGED" % prefix.upper())
        for name in ("imu", "left", "right"):
            if not startup_grace and row["%s_age_s" % name] > threshold["sensor_stale_s"]:
                alerts.add("%s_STALE" % name.upper())
        if not startup_grace and row["feature_age_s"] > threshold["feature_stale_s"]:
            alerts.add("FEATURE_STALE")
        if row["feature_count"] is not None and row["feature_count"] < threshold["min_features"]:
            alerts.add("LOW_FEATURE_COUNT")
        if finite(row["aligned_error_m"]) and row["aligned_error_m"] > threshold["aligned_error_m"]:
            alerts.add("VINS_PX4_DRIFT")
        if finite(row["prop_opt_error_m"]) and row["prop_opt_error_m"] > threshold["prop_opt_error_m"]:
            alerts.add("PROP_OPT_DIVERGENCE")
        if finite(row["acc_peak_mps2"]) and row["acc_peak_mps2"] > threshold["acc_peak_mps2"]:
            alerts.add("IMU_ACCEL_SPIKE")
        if finite(row["gyro_peak_rps"]) and row["gyro_peak_rps"] > threshold["gyro_peak_rps"]:
            alerts.add("IMU_GYRO_SPIKE")
        if finite(row["stereo_stamp_delta_s"]) and abs(row["stereo_stamp_delta_s"]) > threshold["stereo_skew_s"]:
            alerts.add("STEREO_TIMESTAMP_SKEW")
        for name in ("opt", "prop", "imu", "left", "right"):
            age = row["%s_stamp_age_s" % name]
            if finite(age) and abs(age) > threshold["stamp_age_s"]:
                alerts.add("%s_TIMESTAMP_OFFSET" % name.upper())
        return alerts

    def _record_alert_changes(self, alerts, row):
        for alert in sorted(alerts - self.active_alerts):
            self._event("alert_started", {"alert": alert, "sample": self._event_sample(row)})
            self.rospy.logwarn("[vins_health_monitor] %s", alert)
        for alert in sorted(self.active_alerts - alerts):
            self._event("alert_cleared", {"alert": alert, "sample": self._event_sample(row)})
        self.active_alerts = alerts

    def _record_flight_state(self, row):
        state = (row["armed"], row["mode"])
        if state != (self.last_armed, self.last_mode):
            self._event("flight_state", {"armed": bool(row["armed"]), "mode": row["mode"]})
            self.last_armed, self.last_mode = state

    @staticmethod
    def _event_sample(row):
        keys = [
            "wall_time", "armed", "mode", "opt_x_m", "opt_y_m", "opt_z_m",
            "opt_vx_mps", "opt_vy_mps", "opt_vz_mps", "px4_x_m", "px4_y_m",
            "px4_z_m", "aligned_error_m", "prop_opt_error_m", "feature_count",
            "acc_peak_mps2", "gyro_peak_rps", "stereo_stamp_delta_s",
        ]
        return {key: row.get(key) for key in keys}

    def _event(self, event, data):
        payload = {
            "schema": "gameuav.vins_monitor.event.v1",
            "wall_time": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "event": event,
            **data,
        }
        self.events_file.write(json.dumps(json_safe(payload), ensure_ascii=True, sort_keys=True) + "\n")
        self.events_file.flush()

    def run(self):
        rate = self.rospy.Rate(self.sample_rate)
        while not self.rospy.is_shutdown():
            row = self.build_row()
            self.writer.writerow({key: csv_value(value) for key, value in row.items()})
            self.rows_since_flush += 1
            if self.rows_since_flush >= self.sample_rate:
                self.csv_file.flush()
                self.rows_since_flush = 0
            rate.sleep()

    def close(self):
        try:
            self._event("monitor_stopped", {})
        finally:
            self.csv_file.flush()
            self.csv_file.close()
            self.events_file.close()


def main():
    import rospy
    from geometry_msgs.msg import PoseStamped
    from mavros_msgs.msg import State
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image, Imu, PointCloud

    rospy.init_node("vins_health_monitor")
    monitor = VinsHealthMonitor(
        rospy,
        {
            "State": State,
            "PoseStamped": PoseStamped,
            "Odometry": Odometry,
            "Imu": Imu,
            "Image": Image,
            "PointCloud": PointCloud,
        },
    )
    try:
        monitor.run()
    finally:
        monitor.close()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-only
"""Record VIO measurements and independent reference poses without publishing control."""
import json
import time
from pathlib import Path

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu


def stamp_ns(message):
    return message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec


def pose_values(pose):
    return {"position": [pose.position.x, pose.position.y, pose.position.z],
            "orientation_xyzw": [pose.orientation.x, pose.orientation.y,
                                 pose.orientation.z, pose.orientation.w]}


class VioObserver(Node):
    def __init__(self, config, output, record_images=False):
        super().__init__("vio_evaluation_observer")
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=False)
        self.records = (self.output / "samples.jsonl").open("w")
        self.topics = json.loads((Path(config) / "topics.json").read_text())
        self.counts = {}
        self.previous = {}
        self.last_received_wall = {}
        self.max_gap = {}
        self.invalid_stamps = {}
        self.regressions = {}
        self.last_imu = None
        self.last_image = None
        self.record_images = record_images
        self.last_saved_image_ns = 0
        self.first_image = None
        self.camera_info = None
        self.started = time.monotonic()
        sensor_history = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscriptions_owned = [
            self.create_subscription(Image, self.topics["image"], self.image, qos_profile_sensor_data),
            self.create_subscription(Imu, self.topics["imu"], self.imu, sensor_history),
            self.create_subscription(PoseStamped, self.topics["truth"], self.truth, sensor_history),
            self.create_subscription(Odometry, self.topics["odometry"], self.odometry, sensor_history),
            self.create_subscription(CameraInfo, self.topics["image"] + "/camera_info", self.info, qos_profile_sensor_data)]

    def record(self, kind, message, fields):
        stamp = stamp_ns(message)
        self.last_received_wall[kind] = time.monotonic()
        self.counts[kind] = self.counts.get(kind, 0) + 1
        if stamp <= 0:
            self.invalid_stamps[kind] = self.invalid_stamps.get(kind, 0) + 1
        previous = self.previous.get(kind)
        if previous is not None:
            delta = stamp - previous
            if delta <= 0:
                self.regressions[kind] = self.regressions.get(kind, 0) + 1
            self.max_gap[kind] = max(self.max_gap.get(kind, 0), delta)
        self.previous[kind] = stamp
        row = {"kind": kind, "stamp_ns": stamp, "receive_monotonic_ns": time.monotonic_ns(),
               "frame_id": message.header.frame_id, **fields}
        self.records.write(json.dumps(row, allow_nan=False) + "\n")

    def image(self, message):
        self.last_image = stamp_ns(message)
        self.record("image", message, {"width": message.width, "height": message.height,
                                        "encoding": message.encoding})
        if self.first_image is None:
            self.first_image = {"width": message.width, "height": message.height,
                                "step": message.step, "encoding": message.encoding}
            (self.output / "first-image.bin").write_bytes(bytes(message.data))
            (self.output / "first-image.json").write_text(json.dumps(self.first_image, indent=2))

        if self.record_images and self.last_image - self.last_saved_image_ns >= 1_000_000_000:
            frames = self.output / "frames"
            frames.mkdir(exist_ok=True)
            (frames / f"{self.last_image}.bin").write_bytes(bytes(message.data))
            self.last_saved_image_ns = self.last_image

    def imu(self, message):
        self.last_imu = stamp_ns(message)
        self.record("imu", message, {
            "acceleration": [message.linear_acceleration.x, message.linear_acceleration.y,
                             message.linear_acceleration.z],
            "angular_velocity": [message.angular_velocity.x, message.angular_velocity.y,
                                 message.angular_velocity.z]})

    def truth(self, message):
        self.record("truth", message, pose_values(message.pose))

    def odometry(self, message):
        self.record("odometry", message, {
            **pose_values(message.pose.pose), "child_frame_id": message.child_frame_id,
            "velocity": [message.twist.twist.linear.x, message.twist.twist.linear.y,
                         message.twist.twist.linear.z],
            "pose_covariance": list(message.pose.covariance),
            "latest_imu_ns": self.last_imu, "latest_image_ns": self.last_image})

    def info(self, message):
        self.camera_info = {"width": message.width, "height": message.height,
                            "k": list(message.k), "d": list(message.d),
                            "distortion_model": message.distortion_model}

    def event(self, name, **fields):
        self.records.write(json.dumps({"kind": "event", "name": name,
                                       "receive_monotonic_ns": time.monotonic_ns(), **fields}) + "\n")
        self.records.flush()

    def finish(self):
        self.records.close()
        summary = {"wall_seconds": time.monotonic() - self.started, "counts": self.counts,
                   "invalid_stamps": self.invalid_stamps, "nonincreasing_stamps": self.regressions,
                   "maximum_gap_seconds": {key: value / 1e9 for key, value in self.max_gap.items()},
                   "camera_info": self.camera_info}
        (self.output / "observation-summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        return summary

# SPDX-License-Identifier: AGPL-3.0-only
"""Run with camera capture stopped and both PX4 instances freshly disarmed."""

import json
import os
import time
import unittest
from pathlib import Path


@unittest.skipUnless(
    os.environ.get("RAVEN_SIM_STALE_INPUT") == "1",
    "requires explicit stale-input opt-in",
)
class StaleInputTests(unittest.TestCase):
    def test_stale_frames_are_not_detections(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        from vision_msgs.msg import Detection2DArray
        from diagnostic_msgs.msg import DiagnosticArray
        from px4_msgs.msg import VehicleStatus

        rclpy.init()
        node = Node(
            "stale_camera_input_validation",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        vehicles, seen, health, detections = {}, {}, [], []

        def status(message):
            vehicles[message.system_id] = message
            seen[message.system_id] = time.monotonic()

        def diagnostics(message):
            health.extend(s.message for s in message.status)

        def assert_disarmed():
            for identifier in (1, 2):
                self.assertIn(identifier, vehicles)
                self.assertLess(time.monotonic() - seen[identifier], 2)
                self.assertEqual(
                    vehicles[identifier].arming_state,
                    VehicleStatus.ARMING_STATE_DISARMED,
                )

        subscriptions = [
            node.create_subscription(
                VehicleStatus, topic, status, qos_profile_sensor_data
            )
            for topic in (
                "/fmu/out/vehicle_status_v4",
                "/simulation/fmu/out/vehicle_status_v4",
            )
        ]
        subscriptions += [
            node.create_subscription(
                DiagnosticArray, "/raven/vehicle_1/perception/health", diagnostics, 10
            ),
            node.create_subscription(
                Detection2DArray,
                "/raven/vehicle_1/perception/detections",
                detections.append,
                10,
            ),
        ]
        publisher = node.create_publisher(
            Image, "/raven/vehicle_1/cameras/front/image_raw", qos_profile_sensor_data
        )
        try:
            deadline = time.monotonic() + 12
            while len(vehicles) < 2 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            assert_disarmed()
            self.assertGreater(node.get_clock().now().nanoseconds, 5_000_000_000)
            image = Image()
            image.header.frame_id = "sim_camera_optical_frame"
            image.width, image.height, image.step, image.encoding = (
                640,
                360,
                1920,
                "rgb8",
            )
            image.data = bytes(image.step * image.height)
            published, last_publish = 0, 0.0
            deadline = time.monotonic() + 6
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                assert_disarmed()
                if time.monotonic() - last_publish >= 0.2:
                    stamp = node.get_clock().now().nanoseconds - 5_000_000_000
                    image.header.stamp.sec, image.header.stamp.nanosec = divmod(
                        stamp, 1_000_000_000
                    )
                    publisher.publish(image)
                    published += 1
                    last_publish = time.monotonic()
            self.assertGreaterEqual(published, 10)
            self.assertTrue(
                any("stale" in message.lower() for message in health), health[-10:]
            )
            self.assertEqual(len(detections), 0, "Expired images produced detections")
            report = {
                "injected_age_sim_seconds": 5,
                "frames_published": published,
                "detection_messages": len(detections),
                "stale_rejection_observed": True,
                "both_vehicles_remained_disarmed": True,
            }
            Path("/tmp/raven-simulation-stale-input.json").write_text(
                json.dumps(report, indent=2)
            )
            print("STALE_INPUT_RESULT", json.dumps(report), flush=True)
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()

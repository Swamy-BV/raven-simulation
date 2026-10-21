# SPDX-License-Identifier: AGPL-3.0-only
"""Run explicitly on the Raven board after starting the isolated AirSim/PX4 session."""

import os
import math
import unittest


@unittest.skipUnless(
    os.environ.get("RAVEN_SIM_FLIGHT") == "1", "requires explicit SITL opt-in"
)
class FlightTests(unittest.TestCase):
    def setUp(self):
        import rclpy
        from flight_session import FlightSession

        self.rclpy = rclpy
        rclpy.init()
        try:
            self.session = FlightSession()
        except Exception:
            rclpy.shutdown()
            raise

    def tearDown(self):
        try:
            self.session.close()
        finally:
            self.rclpy.shutdown()

    def test_hover(self):
        self.session.run_hover()

    def test_follow(self):
        self.session.run_follow()

    def test_turn(self):
        self.session.run_follow(40.0)

    def test_drive(self):
        from rcl_interfaces.srv import GetParameters
        request = GetParameters.Request()
        request.names = ["max_speed", "max_acceleration", "camera_stabilized",
                         "minimum_heading_speed"]
        reply = self.session.service(self.session.parameters, request)
        self.assertAlmostEqual(reply.values[0].double_value, 8.0)
        self.assertAlmostEqual(reply.values[1].double_value, 3.0)
        self.assertTrue(reply.values[2].bool_value)
        self.assertAlmostEqual(reply.values[3].double_value, 1.0)
        self.session.run_follow(
            40.0, selection_after_sim_seconds=55.0,
            recording_after_sim_seconds=65.0,
            braking_acceleration_mps2=3.0,
        )
        peak = max(math.hypot(*s["velocity_ned"][:2])
                   for s in self.session.samples)
        self.assertGreater(peak, 4.0, "Aircraft did not reach driving speed")

    def test_traffic(self):
        from rcl_interfaces.srv import GetParameters
        request = GetParameters.Request()
        request.names = ["max_speed", "max_acceleration", "align_with_motion",
                         "motion_bearing_offset", "camera_pitch", "ground_z",
                         "camera_stabilized", "follow_path", "follow_distance"]
        reply = self.session.service(self.session.parameters, request)
        self.assertEqual(reply.values[0].double_value, 20.0)
        self.assertEqual(reply.values[1].double_value, 6.0)
        self.assertTrue(reply.values[2].bool_value)
        self.assertAlmostEqual(reply.values[3].double_value, 0.0)
        self.assertAlmostEqual(reply.values[4].double_value, 0.6108652381980153)
        self.assertAlmostEqual(reply.values[5].double_value, 1.2)
        self.assertTrue(reply.values[6].bool_value)
        self.assertTrue(reply.values[7].bool_value)
        self.assertAlmostEqual(reply.values[8].double_value, 8.0)
        self.session.wait(lambda: self.session.camera_info is not None,
                          10, "scenario camera calibration")
        self.assertAlmostEqual(self.session.camera_info.k[0], 379.917386280537, places=4)
        print("PASS live 40 mph scenario limits and camera geometry", flush=True)
        request.names = ["class_agnostic"]
        reply = self.session.service(self.session.tracker_parameters, request)
        self.assertTrue(reply.values[0].bool_value)
        request.names = ["model"]
        reply = self.session.service(self.session.vision_parameters, request)
        self.assertEqual(reply.values[0].string_value, "yolov8m-neutron")
        print("PASS class-independent tracking and medium NPU model", flush=True)
        self.session.hover_yaw_rad = 1.5707963267948966
        self.session.run_follow(
            60.0, altitude_m=5.0, selection_after_sim_seconds=25.0,
            recording_after_sim_seconds=27.5, enable_after_sim_seconds=27.5,
            braking_acceleration_mps2=6.0,
        )
        samples = self.session.samples
        peak = max((s["velocity_ned"][0]**2 + s["velocity_ned"][1]**2)**0.5
                   for s in samples)
        distance = sum(
            ((b["ned"][0]-a["ned"][0])**2 + (b["ned"][1]-a["ned"][1])**2)**0.5
            for a,b in zip(samples,samples[1:]))
        self.assertGreater(peak, 15.0, "Aircraft never reached driving speed")
        self.assertGreater(distance, 300.0, "Insufficient driving-scene flight distance")


if __name__ == "__main__":
    unittest.main()

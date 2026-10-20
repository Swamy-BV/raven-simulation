# SPDX-License-Identifier: AGPL-3.0-only
"""Run explicitly on the Raven board after starting the isolated AirSim/PX4 session."""

import os
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


if __name__ == "__main__":
    unittest.main()

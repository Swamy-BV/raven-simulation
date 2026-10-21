# SPDX-License-Identifier: AGPL-3.0-only
"""Exercise Guidance on the board against the isolated PX4 SITL instance."""

import json
import math
import os
import time
import unittest


@unittest.skipUnless(os.environ.get("RAVEN_SIM_FLIGHT") == "1", "requires explicit SITL opt-in")
class GuidanceTests(unittest.TestCase):
    def setUp(self):
        import rclpy
        from rclpy.action import ActionClient
        from raven_interfaces.action import GoTo
        from flight_session import FlightSession

        self.rclpy = rclpy
        self.GoTo = GoTo
        rclpy.init()
        self.session = FlightSession()
        self.client = ActionClient(self.session.node, GoTo,
                                   "/raven/vehicle_1/autonomy/guidance/go_to")
        self.report = {"passed": False, "checks": []}
        self.assertTrue(self.client.wait_for_server(timeout_sec=10))

    def tearDown(self):
        try:
            self.session.close()
            self.report["real_m7_remained_disarmed"] = not self.session.m7_armed
            self.report["final_sim_arming_state"] = (
                self.session.status.arming_state if self.session.status else None)
        finally:
            (self.session.output_dir / "guidance-result.json").write_text(
                json.dumps(self.report, indent=2))
            self.client.destroy()
            self.rclpy.shutdown()

    def wait(self, predicate, seconds, label):
        def checked():
            s = self.session
            self.assertFalse(s.m7_armed)
            self.assertLess(time.monotonic() - s.m7_seen, 2, "M7 telemetry expired")
            self.assertEqual(s.status.nav_state, s.status.NAVIGATION_STATE_OFFBOARD)
            self.assertFalse(s.status.failsafe)
            return predicate()
        self.session.wait(checked, seconds, label)

    def goal(self, delta_ned):
        s = self.session
        goal = self.GoTo.Goal()
        goal.target.header.stamp = s.node.get_clock().now().to_msg()
        goal.target.header.frame_id = "px4_local_enu"
        target = [s.position.x + delta_ned[0], s.position.y + delta_ned[1],
                  s.position.z + delta_ned[2]]
        goal.target.point.x, goal.target.point.y, goal.target.point.z = (
            float(target[1]), float(target[0]), float(-target[2]))
        goal.yaw = math.pi/2 - s.hover_yaw_rad
        goal.max_speed = 0.6
        goal.position_tolerance = 0.3
        goal.timeout = 60.0
        goal.allow_unmapped_motion = True
        return goal, target

    def send(self, goal):
        future = self.client.send_goal_async(goal)
        self.wait(future.done, 8, "GoTo goal response")
        return future.result()

    def reject(self, goal, label, reason):
        self.assertFalse(self.send(goal).accepted, label)
        self.wait(lambda: reason in self.session.health.get("last_rejection", ""),
                  6, "rejection reason: " + label)
        self.report["checks"].append(label)
        print("PASS", label, flush=True)

    def test_go_to_and_cancel(self):
        from action_msgs.msg import GoalStatus
        from px4_msgs.msg import VehicleCommand
        from std_srvs.srv import SetBool
        s = self.session
        s.takeoff_to_hover()

        goal, _ = self.goal([3, 2, -0.5])
        goal.allow_unmapped_motion = False
        self.reject(goal, "unmapped motion requires explicit opt-in", "unmapped opt-in")
        goal, _ = self.goal([3, 2, -0.5])
        goal.target.header.frame_id = "unknown_frame"
        self.reject(goal, "unknown frame rejected", "configured local ENU")
        goal, _ = self.goal([3, 2, -0.5])
        goal.target.header.stamp.sec = 0
        goal.target.header.stamp.nanosec = 0
        self.reject(goal, "stale goal rejected", "fresh timestamp")
        goal, _ = self.goal([100, 0, 0])
        goal.timeout = 300.0
        self.reject(goal, "displacement limit rejected", "flight envelope")

        goal, target = self.goal([3, 2, -0.5])
        s.stream = False
        handle = self.send(goal)
        self.assertTrue(handle.accepted, s.health)
        result = handle.get_result_async()
        duplicate, _ = self.goal([1, 0, 0])
        self.reject(duplicate, "competing GoTo rejected", "busy")
        request = SetBool.Request()
        request.data = True
        response = s.service(s.enable, request)
        self.assertFalse(response.success, "following must not take GoTo ownership")
        self.assertIn("GoTo owns motion", response.message)
        self.report["checks"].append("following cannot take GoTo ownership")
        self.wait(result.done, 100, "measured GoTo completion")
        reply = result.result()
        self.assertEqual(reply.status, GoalStatus.STATUS_SUCCEEDED, reply.result.message)
        self.assertTrue(reply.result.success)
        error = math.dist(target, [s.position.x, s.position.y, s.position.z])
        self.assertLessEqual(error, goal.position_tolerance)
        self.report["go_to_error_m"] = error
        self.report["checks"].append("GoTo measured arrival")
        self.wait(lambda: s.health.get("state") == "holding", 6, "post-arrival hold")

        goal, _ = self.goal([6, 0, 0])
        handle = self.send(goal)
        self.assertTrue(handle.accepted)
        result = handle.get_result_async()
        self.wait(lambda: math.hypot(s.position.vx, s.position.vy) > 0.25,
                  30, "GoTo moving before cancellation")
        cancel = handle.cancel_goal_async()
        self.wait(cancel.done, 8, "GoTo cancellation acknowledged")
        self.assertTrue(cancel.result().goals_canceling)
        self.wait(result.done, 60, "GoTo cancellation after braking")
        reply = result.result()
        self.assertEqual(reply.status, GoalStatus.STATUS_CANCELED, reply.result.message)
        self.assertFalse(reply.result.success)
        speed = math.sqrt(s.position.vx**2 + s.position.vy**2 + s.position.vz**2)
        self.assertLessEqual(speed, 0.3)
        before = [s.position.x, s.position.y, s.position.z]
        end = time.monotonic() + 3
        self.wait(lambda: time.monotonic() >= end, 5, "settled cancelled hold")
        drift = math.dist(before, [s.position.x, s.position.y, s.position.z])
        self.assertLess(drift, 1.0)
        self.report["cancel_speed_mps"] = speed
        self.report["cancel_hold_drift_m"] = drift
        self.report["checks"].append("cancel brakes and holds")
        s.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        s.wait(lambda: s.land.landed and s.status.arming_state == 1,
               90, "automatic simulated landing and disarm")
        self.report["passed"] = True
        print("GUIDANCE_RESULT", json.dumps(self.report), flush=True)


if __name__ == "__main__":
    unittest.main()

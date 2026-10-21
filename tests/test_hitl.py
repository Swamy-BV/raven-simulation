# SPDX-License-Identifier: AGPL-3.0-only
"""Explicit M7 HITL acceptance; this never uses the SITL flight harness."""

import json
import math
import os
from pathlib import Path
import time
import unittest


@unittest.skipUnless(os.environ.get("RAVEN_M7_HITL") == "1", "requires explicit M7 HITL opt-in")
class HitlTests(unittest.TestCase):
    def test_hover_and_land(self):
        self.run_flight(False)

    def test_follow_and_land(self):
        self.run_flight(True)

    def test_yaw_and_land(self):
        self.run_flight(False, yaw_response=True)

    def run_flight(self, following, yaw_response=False):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from px4_msgs.msg import (VehicleStatus, VehicleLocalPosition,
                                  VehicleLandDetected, VehicleCommandAck,
                                  VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleAttitude,
                                  VehicleOdometry)

        from diagnostic_msgs.msg import DiagnosticArray
        from vision_msgs.msg import Detection2DArray
        from raven_interfaces.srv import SelectTarget
        from std_srvs.srv import SetBool, Trigger
        from rcl_interfaces.srv import GetParameters

        rclpy.init()
        node = rclpy.create_node("m7_hitl_hover_acceptance")
        latest = {}
        seen = {}
        records = []
        acks = []
        subs = []
        output = Path("/tmp/raven-hitl-results") / time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
        output.mkdir(parents=True, exist_ok=False)
        track_log = (output / "tracks.jsonl").open("w")
        health_log = (output / "health.jsonl").open("w")
        attitude_log = (output / "attitude.jsonl").open("w")
        observation_log = (output / "observation-health.jsonl").open("w")
        setpoint_log = (output / "setpoints.jsonl").open("w")
        yaw_log = (output / "yaw-response.jsonl").open("w")
        yaw_reference = None
        yaw_phases = []
        stream = False
        goal = None
        arm_requested = False
        follow_records = []
        recording_seconds = 61.0
        height_m = float(os.environ.get("RAVEN_HITL_HEIGHT_M", "4.5"))
        minimum_speed_mps = float(os.environ.get("RAVEN_HITL_CRUISE_MPS", "3.5"))
        self.assertTrue(2 <= height_m <= 12)
        self.assertTrue(0 < minimum_speed_mps < 30)
        selected = None
        follow_ready_ros_ns = None
        follow_start_ros_ns = None
        follow_end_ros_ns = None
        guidance = "/raven/vehicle_1/autonomy/guidance"
        enable = node.create_client(SetBool, guidance + "/set_following")
        select = node.create_client(SelectTarget, guidance + "/select_target")
        reset = node.create_client(Trigger, "/raven/vehicle_1/perception/reset_tracking")

        def receive(key, message):
            latest[key] = message
            seen[key] = time.monotonic()
            if key == "setpoint":
                nullable = lambda value: float(value) if math.isfinite(value) else None
                setpoint_log.write(json.dumps({
                    "received_ros_ns": node.get_clock().now().nanoseconds,
                    "timestamp_us": message.timestamp,
                    "position": [nullable(v) for v in message.position],
                    "velocity": [nullable(v) for v in message.velocity],
                    "acceleration": [nullable(v) for v in message.acceleration],
                    "yaw": nullable(message.yaw),
                    "yawspeed": nullable(message.yawspeed)}) + "\n")
            if key == "attitude":
                attitude_log.write(json.dumps({"source_ns": message.timestamp_sample * 1000,
                                               "q": [float(v) for v in message.q]}) + "\n")
            if key == "odometry":
                yaw_log.write(json.dumps({"kind": "odometry",
                    "received_ros_ns": node.get_clock().now().nanoseconds,
                    "source_ns": message.timestamp_sample * 1000,
                    "q": [float(v) for v in message.q],
                    "angular_velocity": [float(v) for v in message.angular_velocity]}) + "\n")
            if key == "ack":
                acks.append(message)
            if key == "health" and message.status:
                health_log.write(json.dumps({
                    "ros_ns": node.get_clock().now().nanoseconds,
                    "health": {v.key: v.value for v in message.status[0].values}}) + "\n")
            if key == "observation_health" and message.status:
                diagnostic = message.status[0]
                observation_log.write(json.dumps({
                    "source_ns": message.header.stamp.sec * 10**9 + message.header.stamp.nanosec,
                    "received_ros_ns": node.get_clock().now().nanoseconds,
                    "level": diagnostic.level[0] if isinstance(diagnostic.level, bytes) else int(diagnostic.level),
                    "reason": diagnostic.message,
                    "values": {v.key: v.value for v in diagnostic.values}}) + "\n")
            if key == "tracks":
                track_log.write(json.dumps({
                    "source_ns": message.header.stamp.sec * 10**9 + message.header.stamp.nanosec,
                    "tracks": [{"id": d.id,
                                "class_id": d.results[0].hypothesis.class_id if d.results else None,
                                "score": d.results[0].hypothesis.score if d.results else None,
                                "bbox": [d.bbox.center.position.x, d.bbox.center.position.y,
                                         d.bbox.size_x, d.bbox.size_y]} for d in message.detections]}) + "\n")
            if key == "position":
                records.append({"steady_ns": time.monotonic_ns(),
                                "px4_us": message.timestamp_sample,
                                "ned": [message.x, message.y, message.z],
                                "velocity_ned": [message.vx, message.vy, message.vz]})

        for key, cls, topic in [
            ("status", VehicleStatus, "vehicle_status_v4"),
            ("position", VehicleLocalPosition, "vehicle_local_position_v1"),
            ("land", VehicleLandDetected, "vehicle_land_detected"),
            ("ack", VehicleCommandAck, "vehicle_command_ack_v1"),
            ("attitude", VehicleAttitude, "vehicle_attitude"),
        ]:
            subs.append(node.create_subscription(
                cls, "/fmu/out/" + topic,
                lambda m, key=key: receive(key, m), qos_profile_sensor_data))
        if yaw_response:
            subs.append(node.create_subscription(
                VehicleOdometry, "/fmu/out/vehicle_odometry",
                lambda m: receive("odometry", m), qos_profile_sensor_data))
        if following:
            subs.append(node.create_subscription(
                DiagnosticArray, guidance + "/health",
                lambda m: receive("health", m), 10))
            subs.append(node.create_subscription(
                DiagnosticArray, guidance + "/observation_health",
                lambda m: receive("observation_health", m), qos_profile_sensor_data))
            subs.append(node.create_subscription(
                Detection2DArray, "/raven/vehicle_1/perception/tracks",
                lambda m: receive("tracks", m), 10))
        commands = node.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        modes = node.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        targets = node.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)

        def stamp():
            return node.get_clock().now().nanoseconds // 1000

        def publish():
            if not stream or goal is None:
                return
            mode = OffboardControlMode()
            mode.timestamp = stamp()
            mode.position = True
            modes.publish(mode)
            target = TrajectorySetpoint()
            target.timestamp = stamp()
            target.position = goal
            target.velocity = [math.nan] * 3
            target.acceleration = [math.nan] * 3
            target.jerk = [math.nan] * 3
            target.yaw = math.nan
            target.yawspeed = math.nan
            if yaw_reference is not None:
                phase, position, velocity, acceleration, yaw, rate = yaw_reference
                moving = phase in ("straight_fixed_yaw", "circle_fixed_yaw", "circle_tangent_yaw")
                target.position = [math.nan, math.nan, position[2]] if moving else position
                target.velocity = [velocity[0], velocity[1], math.nan] if moving else velocity
                target.acceleration = [acceleration[0], acceleration[1], math.nan] if moving else acceleration
                target.yaw = math.atan2(math.sin(yaw), math.cos(yaw))
                target.yawspeed = rate
                yaw_log.write(json.dumps({"kind": "reference", "ros_ns": target.timestamp * 1000,
                    "phase": phase, "expected_position_ned": position,
                    "velocity_ned": velocity, "acceleration_ned": acceleration,
                    "yaw": target.yaw, "yawspeed": rate,
                    "xy_control": "velocity" if moving else "position"}) + "\n")
            targets.publish(target)

        timer = node.create_timer(.05, publish)
        subs.append(node.create_subscription(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint",
            lambda m: receive("setpoint", m), qos_profile_sensor_data))

        def wait(predicate, seconds, label):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.02)
                if predicate():
                    print("PASS", label, flush=True)
                    return
            self.fail(label)

        def fresh():
            return all(k in latest and time.monotonic() - seen[k] < 2
                       for k in ("status", "position", "land"))

        def command(number, values=()):
            message = VehicleCommand()
            message.timestamp = stamp()
            message.command = number
            message.target_system = 1
            message.target_component = 1
            message.source_system = 246
            message.source_component = 191
            message.from_external = True
            for index, value in enumerate(values, 1):
                setattr(message, "param" + str(index), float(value))
            begin = len(acks)
            commands.publish(message)
            wait(lambda: any(a.command == number and a.target_system == 246
                             and a.target_component == 191 for a in acks[begin:]),
                 8, "M7 command acknowledgement " + str(number))
            ack = next(a for a in acks[begin:] if a.command == number
                       and a.target_system == 246 and a.target_component == 191)
            self.assertEqual(ack.result, VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED)

        def service(client, request):
            self.assertTrue(client.wait_for_service(timeout_sec=5), client.srv_name)
            future = client.call_async(request)
            wait(future.done, 8, client.srv_name)
            return future.result()

        def health():
            if "health" not in latest or not latest["health"].status:
                return {}
            return dict((v.key, v.value) for v in latest["health"].status[0].values)

        def visible_car():
            tracks = latest.get("tracks")
            if tracks is None:
                return None
            age = node.get_clock().now().nanoseconds / 1e9 - (
                tracks.header.stamp.sec + tracks.header.stamp.nanosec / 1e9)
            if not 0 <= age < .3:
                return None
            cars = [d for d in tracks.detections if d.results
                    and d.results[0].hypothesis.class_id == "2"
                    and d.results[0].hypothesis.score > .75
                    and 230 < d.bbox.center.position.x < 410
                    and 60 < d.bbox.center.position.y < 300]
            # The acquisition reticle selects the foreground vehicle rather than a
            # smaller parked car whose detector confidence happens to be higher.
            return max(cars, key=lambda d: d.bbox.size_x * d.bbox.size_y) if cars else None

        try:
            wait(lambda: fresh() and latest["position"].xy_valid
                 and latest["position"].z_valid, 45, "fresh M7 estimator and land telemetry")
            if following:
                for name in (guidance, "/raven/vehicle_1/perception/vision_engine",
                             "/raven/vehicle_1/perception/target_tracker",
                             "/raven/vehicle_1/cameras/front/media_capture"):
                    client = node.create_client(GetParameters, name + "/get_parameters")
                    request = GetParameters.Request()
                    request.names = ["use_sim_time"]
                    reply = service(client, request)
                    self.assertFalse(reply.values[0].bool_value,
                                     "M7 pipeline requires system time: " + name)
                    node.destroy_client(client)
                wait(lambda: "tracks" in latest
                     and 0 <= node.get_clock().now().nanoseconds / 1e9
                     - latest["tracks"].header.stamp.sec
                     - latest["tracks"].header.stamp.nanosec / 1e9 < .3,
                     15, "fresh real-time inference output before arming")
                wait(lambda: health().get("px4_namespace") == "", 5,
                     "guidance addresses the real M7 namespace")
            if yaw_response:
                self.assertEqual(node.count_publishers("/fmu/in/trajectory_setpoint"), 1,
                                 "Yaw diagnostic must be the only setpoint publisher before arming")
            status = latest["status"]
            self.assertEqual(status.system_id, 1)
            self.assertEqual(status.hil_state, VehicleStatus.HIL_STATE_ON)
            self.assertEqual(status.arming_state, VehicleStatus.ARMING_STATE_DISARMED)
            self.assertTrue(latest["land"].landed)
            p = latest["position"]
            origin = [float(p.x), float(p.y), float(p.z)]
            goal = [origin[0], origin[1], origin[2] - height_m]
            stream = True
            end = time.monotonic() + 3
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.02)
            command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, [1, 6])
            wait(lambda: latest["status"].nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD,
                 8, "M7 offboard mode")
            wait(lambda: latest["status"].pre_flight_checks_pass, 20, "normal HIL arming checks")
            arm_requested = True
            command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, [1, 0])
            wait(lambda: fresh() and latest["status"].arming_state == VehicleStatus.ARMING_STATE_ARMED,
                 5, "M7 armed in HIL mode")
            wait(lambda: fresh() and abs(latest["position"].z - goal[2]) < .3
                 and abs(latest["position"].vz) < .3, 40, f"M7 controlled takeoff to {height_m} m")
            hover_start_ros_ns = node.get_clock().now().nanoseconds
            start = time.monotonic()
            maximum_error = 0.0
            while time.monotonic() - start < 10:
                rclpy.spin_once(node, timeout_sec=.02)
                self.assertTrue(fresh(), "M7 telemetry expired")
                status = latest["status"]
                self.assertEqual(status.hil_state, VehicleStatus.HIL_STATE_ON)
                self.assertEqual(status.nav_state, VehicleStatus.NAVIGATION_STATE_OFFBOARD)
                self.assertFalse(status.failsafe)
                p = latest["position"]
                maximum_error = max(maximum_error, math.dist(goal, [p.x, p.y, p.z]))
                self.assertLess(maximum_error, .75, "hover position error")
            hover_end_ros_ns = node.get_clock().now().nanoseconds
            if yaw_response:
                from yaw_profile import yaw_response_sample
                wait(lambda: "odometry" in latest and time.monotonic() - seen["odometry"] < .25,
                     10, "fresh M7 body rates for yaw diagnostic")
                self.assertEqual(node.count_publishers("/fmu/in/trajectory_setpoint"), 1,
                                 "Yaw diagnostic must be the only setpoint publisher")
                q = latest["attitude"].q
                heading = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]),
                                     1 - 2 * (q[2] * q[2] + q[3] * q[3]))
                diagnostic_start = time.monotonic()
                previous_phase = None
                while True:
                    yaw_reference = yaw_response_sample(time.monotonic() - diagnostic_start, goal, heading)
                    if yaw_reference is None:
                        break
                    phase, expected, _, _, expected_yaw, _ = yaw_reference
                    if phase != previous_phase:
                        entry = {"phase": phase, "ros_ns": node.get_clock().now().nanoseconds}
                        yaw_phases.append(entry)
                        print("YAW_PHASE", json.dumps(entry), flush=True)
                        previous_phase = phase
                    rclpy.spin_once(node, timeout_sec=.02)
                    self.assertTrue(fresh(), "M7 telemetry expired")
                    self.assertLess(time.monotonic() - seen["attitude"], .25)
                    self.assertLess(time.monotonic() - seen["odometry"], .25)
                    self.assertEqual(latest["status"].hil_state, VehicleStatus.HIL_STATE_ON)
                    self.assertEqual(latest["status"].nav_state, VehicleStatus.NAVIGATION_STATE_OFFBOARD)
                    self.assertFalse(latest["status"].failsafe)
                    p = latest["position"]
                    self.assertLess(abs(p.z - goal[2]), 1.5, "yaw diagnostic altitude envelope")
                    self.assertLess(math.dist(expected[:2], [p.x, p.y]), 8,
                                    "yaw diagnostic path error envelope")
                    q = latest["attitude"].q
                    measured_yaw = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]),
                                              1 - 2 * (q[2] * q[2] + q[3] * q[3]))
                    difference = measured_yaw - expected_yaw
                    self.assertLess(abs(math.atan2(math.sin(difference), math.cos(difference))),
                                    math.radians(60), "yaw diagnostic heading envelope")
                p = latest["position"]
                goal = [float(p.x), float(p.y), goal[2]]
                print("PASS yaw diagnostic sequence completed; response metrics require offline analysis", flush=True)
            if following:
                # Acquire identity at the operating viewpoint. Ground-level
                # detections before takeoff do not define the selected object.
                self.assertTrue(service(reset, Trigger.Request()).success)
                latest.pop("tracks", None)
                wait(lambda: visible_car() is not None, 40, "visible NPU-tracked car")
                selected = visible_car().id
                request = SelectTarget.Request()
                request.track_id = selected
                reply = service(select, request)
                self.assertTrue(reply.success, reply.message)
                request = SetBool.Request()
                request.data = True
                stream = False  # Guidance becomes the sole setpoint publisher.
                reply = service(enable, request)
                self.assertTrue(reply.success, reply.message)
                wait(lambda: health().get("state") == "following", 5,
                     "M7 guidance authority handover")
                follow_ready_ros_ns = node.get_clock().now().nanoseconds
                print("FOLLOWING_READY", follow_ready_ros_ns, flush=True)
                # A minute means moving-target footage, excluding acquisition
                # and the scenario's stationary start delay.
                start = None
                stable_since = None
                warmup_deadline = time.monotonic() + 150
                last_report = 0
                while start is None or time.monotonic() - start < recording_seconds:
                    rclpy.spin_once(node, timeout_sec=.02)
                    self.assertTrue(fresh(), "M7 telemetry expired")
                    self.assertEqual(latest["status"].hil_state, VehicleStatus.HIL_STATE_ON)
                    self.assertEqual(latest["status"].nav_state, VehicleStatus.NAVIGATION_STATE_OFFBOARD)
                    self.assertFalse(latest["status"].failsafe)
                    h = health()
                    self.assertEqual(h.get("selected_track"), selected)
                    self.assertNotIn(h.get("state"), ("holding", "unavailable"), h)
                    p = latest["position"]
                    sample = {"ros_ns": node.get_clock().now().nanoseconds,
                              "ned": [p.x, p.y, p.z],
                              "velocity_ned": [p.vx, p.vy, p.vz], "health": h}
                    if start is None:
                        self.assertLess(time.monotonic(), warmup_deadline,
                                        "moving-target recording gate timed out")
                        moving = (h.get("state") == "following"
                                  and float(h.get("target_speed_mps", "0")) >= minimum_speed_mps)
                        if moving:
                            if stable_since is None:
                                stable_since = time.monotonic()
                            if time.monotonic() - stable_since >= 2:
                                start = time.monotonic()
                                follow_start_ros_ns = node.get_clock().now().nanoseconds
                                print("RECORDING_START", follow_start_ros_ns, flush=True)
                        else:
                            stable_since = None
                    if start is not None:
                        follow_records.append(sample)
                    if time.monotonic() - last_report > 3:
                        print("FOLLOW" if start is not None else "WARMUP", json.dumps(sample), flush=True)
                        last_report = time.monotonic()
                follow_end_ros_ns = node.get_clock().now().nanoseconds
                fraction = sum(s["health"].get("state") == "following"
                               for s in follow_records) / len(follow_records)
                self.assertGreater(fraction, .9, "sustained following fraction")
                displacement = math.dist(follow_records[0]["ned"][:2],
                                         follow_records[-1]["ned"][:2])
                self.assertGreater(displacement, 5, "actual M7 following displacement")
                service(reset, Trigger.Request())
                wait(lambda: health().get("state") == "holding", 5,
                     "identity loss latches hold")
                wait(lambda: math.hypot(latest["position"].vx, latest["position"].vy) < .15,
                     20, "M7 brakes to a stop")
                before = [latest["position"].x, latest["position"].y, latest["position"].z]
                end = time.monotonic() + 3
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=.02)
                p = latest["position"]
                self.assertLess(math.dist(before, [p.x, p.y, p.z]), 1, "latched hold drift")
                request.data = False
                self.assertTrue(service(enable, request).success)
            command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            stream = False
            wait(lambda: fresh() and latest["land"].landed
                 and latest["status"].arming_state == VehicleStatus.ARMING_STATE_DISARMED,
                 90, "M7 automatic landing and disarm")
            result = {"passed": True, "system_id": 1, "controller": "physical Cortex-M7",
                      "physics": "AirSim", "clock": "hardware / ROS system time",
                      "command_transport": "ROS DDS over RPMsg", "hover_seconds": 10,
                      "height_m": height_m, "minimum_target_speed_mps": minimum_speed_mps, "maximum_hover_error_m": maximum_error,
                      "normal_arming": True, "landed": True, "disarmed": True,
                      "hover_start_ros_ns": hover_start_ros_ns,
                      "hover_end_ros_ns": hover_end_ros_ns,
                      "origin_ned": origin,
                      "hover_goal_ned": [origin[0], origin[1], origin[2] - height_m],
                      "final_hold_goal_ned": goal,
                      "selected_track": selected,
                      "following_seconds": recording_seconds if following else 0,
                      "following_ready_ros_ns": follow_ready_ros_ns,
                      "following_start_ros_ns": follow_start_ros_ns,
                      "following_end_ros_ns": follow_end_ros_ns,
                      "yaw_diagnostic_completed": yaw_response, "yaw_phases": yaw_phases}
            (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
            print("RESULT", json.dumps(result), flush=True)
        except Exception as error:
            (output / "result.json").write_text(json.dumps({
                "passed": False, "system_id": 1, "controller": "physical Cortex-M7",
                "selected_track": selected, "error": str(error)}, indent=2) + "\n")
            raise
        finally:
            if arm_requested and "status" in latest and latest["status"].arming_state == VehicleStatus.ARMING_STATE_ARMED:
                stream = False
                try:
                    if following and enable.service_is_ready():
                        request = SetBool.Request()
                        request.data = False
                        service(enable, request)
                    command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                    wait(lambda: fresh() and latest["land"].landed
                         and latest["status"].arming_state == VehicleStatus.ARMING_STATE_DISARMED,
                         90, "failure cleanup landing and disarm")
                except Exception as error:
                    print("CLEANUP FAILED", str(error), flush=True)
            (output / "following.json").write_text(json.dumps(follow_records) + "\n")
            (output / "positions.json").write_text(json.dumps(records) + "\n")
            track_log.close()
            health_log.close()
            attitude_log.close()
            observation_log.close()
            setpoint_log.close()
            yaw_log.close()
            node.destroy_timer(timer)
            node.destroy_node()
            rclpy.shutdown()

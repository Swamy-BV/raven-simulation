# SPDX-License-Identifier: AGPL-3.0-only
import json, math, time, struct, zlib, shutil, os
from collections import OrderedDict
from pathlib import Path
from sensor_msgs.msg import Image, CameraInfo
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleStatus,
    VehicleAttitude,
    VehicleLocalPosition,
    VehicleLandDetected,
)
from vision_msgs.msg import Detection2DArray
from diagnostic_msgs.msg import DiagnosticArray
from raven_interfaces.srv import SelectTarget
from std_srvs.srv import SetBool, Trigger
from rcl_interfaces.srv import GetParameters


class FlightSession:
    def __init__(self, hover_yaw_rad=math.pi / 2, record_unselected=False):
        if not math.isfinite(hover_yaw_rad):
            raise ValueError("Hover yaw must be finite")
        self.hover_yaw_rad = hover_yaw_rad
        self.record_unselected = record_unselected
        results_root = Path("/tmp/raven-simulation-results")
        results_root.mkdir(parents=True, exist_ok=True)
        self.capture_snapshots = os.environ.get("RAVEN_SIM_SNAPSHOTS") == "1"
        reserve_mib = 512 if self.capture_snapshots else 128
        if shutil.disk_usage(results_root).free < reserve_mib * 1024 * 1024:
            raise RuntimeError(f"Need {reserve_mib} MiB free for flight evidence before arming")
        self.output_dir = results_root / time.strftime(
            "%Y%m%dT%H%M%SZ", time.gmtime()
        )
        self.output_dir.mkdir(parents=True, exist_ok=False)
        self.node = Node(
            "selected_target_following_acceptance",
            parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)],
        )
        self.status = None
        self.m7 = None
        self.position = None
        self.latest_setpoint = None
        self.land = None
        self.tracks = None
        self.camera_info = None
        self.health = {}
        self.acks = []
        self.stream = False
        self.goal = None
        self.samples = []
        self.last_clock = None
        self.clock_delivery = {"updates": 0, "max_step_ms": 0.0,
                               "max_wall_gap_ms": 0.0, "regressions": 0}
        self.m7_armed = False
        self.last_print = 0
        self.m7_seen = 0.0
        self.count = 0
        self.subs = []
        self.images = OrderedDict()
        self.selected = None
        self.detector_model = None
        self.snapshot_time = 0
        self.snapshot_index = 0
        for cls, topic, callback in [
            (VehicleStatus, "/simulation/fmu/out/vehicle_status_v4", self.on_status),
            (VehicleStatus, "/fmu/out/vehicle_status_v4", self.on_m7),
            (
                VehicleLocalPosition,
                "/simulation/fmu/out/vehicle_local_position_v1",
                self.on_position,
            ),
            (
                VehicleLandDetected,
                "/simulation/fmu/out/vehicle_land_detected",
                self.on_land,
            ),
            (
                VehicleCommandAck,
                "/simulation/fmu/out/vehicle_command_ack_v1",
                self.on_ack,
            ),
        ]:
            self.subs.append(
                self.node.create_subscription(
                    cls, topic, callback, qos_profile_sensor_data
                )
            )
        self.subs.append(
            self.node.create_subscription(
                Image,
                "/raven/vehicle_1/cameras/front/image_raw",
                self.on_image,
                qos_profile_sensor_data,
            )
        )
        self.subs.append(
            self.node.create_subscription(
                Detection2DArray,
                "/raven/vehicle_1/perception/tracks",
                self.on_tracks,
                10,
            )
        )
        self.raw = []
        self.raw_file = open(self.output_dir / "observations.jsonl", "w")
        self.sample_file = open(self.output_dir / "samples.jsonl", "w")
        self.subs.append(
            self.node.create_subscription(
                Detection2DArray,
                "/raven/vehicle_1/perception/detections",
                self.on_detections,
                10,
            )
        )
        self.subs.append(
            self.node.create_subscription(
                DiagnosticArray,
                "/raven/vehicle_1/autonomy/guidance/health",
                self.on_health,
                10,
            )
        )
        self.mode = self.node.create_publisher(
            OffboardControlMode,
            "/simulation/fmu/in/offboard_control_mode",
            qos_profile_sensor_data,
        )
        self.setpoint = self.node.create_publisher(
            TrajectorySetpoint,
            "/simulation/fmu/in/trajectory_setpoint",
            qos_profile_sensor_data,
        )
        self.commands = self.node.create_publisher(
            VehicleCommand,
            "/simulation/fmu/in/vehicle_command",
            qos_profile_sensor_data,
        )
        self.parameters = self.node.create_client(
            GetParameters, "/raven/vehicle_1/autonomy/guidance/get_parameters"
        )
        self.tracker_parameters = self.node.create_client(
            GetParameters, "/raven/vehicle_1/perception/target_tracker/get_parameters"
        )
        self.vision_parameters = self.node.create_client(
            GetParameters, "/raven/vehicle_1/perception/vision_engine/get_parameters"
        )
        self.subs.append(self.node.create_subscription(
            CameraInfo, "/raven/vehicle_1/cameras/front/camera_info",
            self.on_camera_info, qos_profile_sensor_data))
        self.select = self.node.create_client(
            SelectTarget, "/raven/vehicle_1/autonomy/guidance/select_target"
        )
        self.enable = self.node.create_client(
            SetBool, "/raven/vehicle_1/autonomy/guidance/set_following"
        )
        self.reset = self.node.create_client(
            Trigger, "/raven/vehicle_1/perception/reset_tracking"
        )
        self.subs.append(self.node.create_subscription(
            VehicleAttitude, "/simulation/fmu/out/vehicle_attitude",
            self.on_attitude, qos_profile_sensor_data))
        for suffix in ("target_observation", "target_position"):
            self.subs.append(self.node.create_subscription(
                PointStamped, "/raven/vehicle_1/autonomy/guidance/" + suffix,
                lambda m, kind=suffix: self.on_target_point(kind, m),
                qos_profile_sensor_data))
        self.subs.append(self.node.create_subscription(
            TrajectorySetpoint, "/simulation/fmu/in/trajectory_setpoint",
            self.on_setpoint, qos_profile_sensor_data))
        self.subs.append(self.node.create_subscription(
            Clock, "/clock", self.on_clock, qos_profile_sensor_data))
        self.timer = self.node.create_timer(0.05, self.publish_hold)

    def on_clock(self, message):
        stamp = message.clock.sec * 1000000000 + message.clock.nanosec
        steady = time.monotonic_ns()
        metrics = self.clock_delivery
        if self.last_clock is not None:
            previous_stamp, previous_steady = self.last_clock
            metrics["max_step_ms"] = max(metrics["max_step_ms"],
                                         (stamp - previous_stamp) / 1e6)
            metrics["max_wall_gap_ms"] = max(metrics["max_wall_gap_ms"],
                                             (steady - previous_steady) / 1e6)
            metrics["regressions"] += int(stamp < previous_stamp)
        metrics["updates"] += 1
        self.last_clock = stamp, steady
        self.raw_file.write(json.dumps({
            "kind": "clock", "stamp_ns": stamp, "received_steady_ns": steady,
        }) + "\n")

    def on_setpoint(self, m):
        self.latest_setpoint = m
        self.raw_file.write(json.dumps({
            "kind": "setpoint", "stamp_ns": m.timestamp * 1000,
            "received_ns": self.node.get_clock().now().nanoseconds,
            "received_steady_ns": time.monotonic_ns(),
            "velocity_ned": [float(v) if math.isfinite(v) else None for v in m.velocity],
            "acceleration_ned": [float(v) if math.isfinite(v) else None for v in m.acceleration],
            "yaw": float(m.yaw) if math.isfinite(m.yaw) else None,
        }) + "\n")

    def on_camera_info(self, message):
        self.camera_info = message

    def on_status(self, m):
        self.status = m
        self.raw_file.write(json.dumps({
            "kind": "status", "stamp_ns": m.timestamp * 1000,
            "received_ns": self.node.get_clock().now().nanoseconds,
            "received_steady_ns": time.monotonic_ns(),
            "system_id": m.system_id, "arming_state": m.arming_state,
            "nav_state": m.nav_state, "failsafe": m.failsafe,
        }) + "\n")

    def on_m7(self, m):
        self.m7_seen = time.monotonic()
        self.m7 = m
        self.m7_armed |= m.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def on_position(self, m):
        self.position = m
        self.raw_file.write(json.dumps({
            "kind": "position", "stamp_ns": m.timestamp_sample * 1000,
            "received_ns": self.node.get_clock().now().nanoseconds,
            "received_steady_ns": time.monotonic_ns(),
            "acceleration_ned": [m.ax, m.ay, m.az],
            "ned": [m.x, m.y, m.z], "velocity_ned": [m.vx, m.vy, m.vz],
        }) + "\n")

    def on_attitude(self, m):
        self.raw_file.write(json.dumps({
            "kind": "attitude", "stamp_ns": m.timestamp_sample * 1000,
            "received_ns": self.node.get_clock().now().nanoseconds,
            "received_steady_ns": time.monotonic_ns(),
            "q": [float(value) for value in m.q],
        }) + "\n")

    def on_target_point(self, kind, m):
        self.raw_file.write(json.dumps({
            "kind": kind, "frame": m.header.frame_id,
            "stamp_ns": m.header.stamp.sec * 1000000000 + m.header.stamp.nanosec,
            "received_ns": self.node.get_clock().now().nanoseconds,
            "point": [m.point.x, m.point.y, m.point.z],
        }) + "\n")

    def on_land(self, m):
        self.land = m

    def on_ack(self, m):
        if m.target_system == 246 and m.target_component == 191:
            self.acks.append(m)

    def on_image(self, m):
        key = m.header.stamp.sec * 1000000000 + m.header.stamp.nanosec
        self.images[key] = m
        while len(self.images) > 8:
            self.images.popitem(last=False)

    def on_tracks(self, m):
        self.tracks = m
        self.log_observations("tracks", m)
        if not self.capture_snapshots:
            return
        if (
            (not self.selected and not self.record_unselected)
            or self.position is None
            or self.health.get("state")
            not in (
                ["idle", "following", "waiting", "holding"]
                if self.record_unselected
                else ["following", "waiting", "holding"]
            )
            or time.monotonic() - self.snapshot_time
            < (0.2 if self.record_unselected else 0.05)
        ):
            return
        key = m.header.stamp.sec * 1000000000 + m.header.stamp.nanosec
        if key not in self.images:
            return
        d = next((d for d in m.detections if d.id == self.selected), None)
        image = self.images[key]
        data = bytes(image.data)

        def chunk(t, v):
            return (
                struct.pack(">I", len(v))
                + t
                + v
                + struct.pack(">I", zlib.crc32(t + v) & 0xFFFFFFFF)
            )

        rows = b"".join(
            b"\0" + data[y * image.step : y * image.step + image.width * 3]
            for y in range(image.height)
        )
        png = (
            b"\x89PNG\r\n\x1a\n"
            + chunk(
                b"IHDR",
                struct.pack(">IIBBBBB", image.width, image.height, 8, 2, 0, 0, 0),
            )
            + chunk(b"IDAT", zlib.compress(rows, 1))
            + chunk(b"IEND", b"")
        )
        folder = self.output_dir / "frames"
        folder.mkdir(parents=True, exist_ok=True)
        name = f"{self.snapshot_index:06d}"
        (folder / (name + ".png")).write_bytes(png)
        (folder / (name + ".json")).write_text(
            json.dumps(
                {
                    "stamp_ns": key,
                    "id": d.id if d else None,
                    "score": d.results[0].hypothesis.score if d else 0,
                    "state": self.health.get("state"),
                    "tracks": [
                        {
                            "id": x.id,
                            "class": x.results[0].hypothesis.class_id,
                            "score": x.results[0].hypothesis.score,
                        }
                        for x in m.detections
                    ],
                    "cx": d.bbox.center.position.x if d else None,
                    "cy": d.bbox.center.position.y if d else None,
                    "w": d.bbox.size_x if d else None,
                    "h": d.bbox.size_y if d else None,
                    "position_ned": [self.position.x, self.position.y, self.position.z],
                }
            )
        )
        self.snapshot_time = time.monotonic()
        self.snapshot_index += 1

    def log_observations(self, kind, m):
        self.raw_file.write(
            json.dumps(
                {
                    "kind": kind,
                    "received_ns": self.node.get_clock().now().nanoseconds,
                    "received_steady_ns": time.monotonic_ns(),
                    "stamp_ns": m.header.stamp.sec * 1000000000
                    + m.header.stamp.nanosec,
                    "selected": self.selected,
                    "state": self.health.get("state"),
                    "observations": [
                        {
                            "id": d.id,
                            "class": d.results[0].hypothesis.class_id,
                            "score": d.results[0].hypothesis.score,
                            "cx": d.bbox.center.position.x,
                            "cy": d.bbox.center.position.y,
                            "w": d.bbox.size_x,
                            "h": d.bbox.size_y,
                        }
                        for d in m.detections
                    ],
                }
            )
            + "\n"
        )
        self.raw_file.flush()

    def on_detections(self, m):
        self.log_observations("detections", m)

    def on_health(self, m):
        for s in m.status:
            self.health = {v.key: v.value for v in s.values}
            self.health["reason"] = s.message
            self.raw_file.write(json.dumps({
                "kind": "following", "stamp_ns": self.node.get_clock().now().nanoseconds,
                "received_steady_ns": time.monotonic_ns(),
                "health": self.health,
                "ned": [self.position.x,self.position.y,self.position.z] if self.position else None,
            }) + "\n")
            self.raw_file.flush()

    def timestamp(self):
        return self.node.get_clock().now().nanoseconds // 1000

    def publish_hold(self):
        if not self.stream or self.goal is None:
            return
        m = OffboardControlMode()
        m.timestamp = self.timestamp()
        m.position = True
        self.mode.publish(m)
        s = TrajectorySetpoint()
        s.timestamp = m.timestamp
        s.position = self.goal
        s.velocity = [math.nan] * 3
        s.acceleration = [math.nan] * 3
        s.jerk = [math.nan] * 3
        s.yaw = self.hover_yaw_rad
        s.yawspeed = math.nan
        self.setpoint.publish(s)
        self.count += 1

    def wait(self, predicate, seconds, label):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.m7_armed:
                raise RuntimeError("Unexpected M7 arming")
            if predicate():
                print("PASS", label, flush=True)
                return
        raise RuntimeError("Timeout: " + label)

    def service(self, client, request):
        assert client.wait_for_service(timeout_sec=5), "service unavailable"
        future = client.call_async(request)
        self.wait(future.done, 8, "service reply " + client.srv_name)
        return future.result()

    def command(self, command, params=()):
        assert self.status and self.status.system_id == 2, "wrong simulated vehicle"
        assert (
            self.m7
            and self.m7.system_id == 1
            and self.m7.arming_state == 1
            and time.monotonic() - self.m7_seen < 2
        ), "M7 status unavailable or armed"
        old = len(self.acks)
        m = VehicleCommand()
        m.timestamp = self.timestamp()
        m.command = command
        m.target_system = 2
        m.target_component = 1
        m.source_system = 246
        m.source_component = 191
        m.from_external = True
        for i, v in enumerate(params, 1):
            setattr(m, "param" + str(i), float(v))
        self.commands.publish(m)
        self.wait(
            lambda: len(self.acks) > old
            and any(a.command == command for a in self.acks[old:]),
            8,
            "command ack " + str(command),
        )
        a = next(a for a in self.acks[old:] if a.command == command)
        assert a.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED, (
            "command rejected",
            command,
            a.result,
        )

    def visible_car(self):
        if not self.tracks:
            return None
        t = self.tracks.header.stamp.sec + self.tracks.header.stamp.nanosec / 1e9
        if self.node.get_clock().now().nanoseconds / 1e9 - t > 0.3:
            return None
        cars = [
            d
            for d in self.tracks.detections
            if d.results
            and d.results[0].hypothesis.class_id == "2"
            and d.results[0].hypothesis.score > 0.75
            and 230 < d.bbox.center.position.x < 410
            and 60 < d.bbox.center.position.y < 300
        ]
        return max(cars, key=lambda d: d.results[0].hypothesis.score) if cars else None

    def hover_ready(self):
        error = math.dist(
            [self.position.x, self.position.y, self.position.z], self.goal
        )
        speed = math.sqrt(
            self.position.vx**2 + self.position.vy**2 + self.position.vz**2
        )
        if error < 0.2 and speed < 0.15:
            if not hasattr(self, "settled"):
                self.settled = time.monotonic()
            return time.monotonic() - self.settled > 1.5
        self.settled = time.monotonic()
        return False

    def takeoff_to_hover(self, altitude_m=4.5):
        self.wait(lambda: self.status and self.m7 and self.position and self.land
                  and self.position.xy_valid and self.position.z_valid,
                  25, "isolated SITL and M7 telemetry")
        assert (self.status.system_id == 2 and self.m7.system_id == 1
                and self.status.arming_state == 1 and self.land.landed)
        self.goal = [float(self.position.x), float(self.position.y),
                     float(self.position.z) - altitude_m]
        self.stream = True
        self.wait(lambda: self.count >= 50, 12, "20 Hz offboard warmup")
        self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, [1, 6])
        self.wait(lambda: self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD,
                  8, "simulated offboard mode")
        self.wait(lambda: self.status.pre_flight_checks_pass,
                  20, "normal offboard preflight checks")
        self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, [1, 0])
        self.wait(self.hover_ready, 60, "settled simulated hover")

    def run_follow(self, duration_sim_seconds=25.0, *, altitude_m=4.5,
                   selection_after_sim_seconds=65.0,
                   recording_after_sim_seconds=0.0,
                   enable_after_sim_seconds=0.0,
                   braking_acceleration_mps2=None):
        if not 25 <= duration_sim_seconds <= 60:
            raise ValueError("Following duration must be 25..60 simulated seconds")
        if not 2 <= altitude_m <= 10 or not 0 <= selection_after_sim_seconds <= 120:
            raise ValueError("Invalid following altitude or selection window")
        if (braking_acceleration_mps2 is not None
                and not 0 < braking_acceleration_mps2 <= 8):
            raise ValueError("Invalid braking acceleration")
        if not 0 <= recording_after_sim_seconds <= 180:
            raise ValueError("Invalid following recording start")
        self.wait(
            lambda: self.status
            and self.m7
            and self.position
            and self.land
            and self.position.xy_valid
            and self.position.z_valid,
            20,
            "both vehicle telemetry streams",
        )
        assert (
            self.status.system_id == 2
            and self.m7.system_id == 1
            and self.status.arming_state == 1
            and self.land.landed
        )
        model_request = GetParameters.Request()
        model_request.names = ["model"]
        model_reply = self.service(self.vision_parameters, model_request)
        self.detector_model = model_reply.values[0].string_value
        print("DETECTOR_MODEL", self.detector_model, flush=True)
        request = SelectTarget.Request()
        request.track_id = "not-a-visible-track"
        assert not self.service(self.select, request).success
        request = SetBool.Request()
        request.data = True
        assert not self.service(self.enable, request).success
        # Offboard mode checks apply after a valid setpoint stream is established.
        self.goal = [
            float(self.position.x),
            float(self.position.y),
            float(self.position.z) - altitude_m,
        ]
        self.stream = True
        self.wait(lambda: self.count >= 50, 10, "20 Hz offboard warmup")
        self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, [1, 6])
        self.wait(
            lambda: self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD,
            5,
            "simulated offboard mode",
        )
        self.wait(
            lambda: self.status.pre_flight_checks_pass,
            15,
            "normal offboard preflight checks",
        )
        self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, [1, 0])
        self.wait(self.hover_ready, 50, f"settled hover at {altitude_m} m")
        self.wait(
            lambda: self.node.get_clock().now().nanoseconds
            >= selection_after_sim_seconds * 1e9,
            100,
            "target selection window",
        )
        self.wait(
            lambda: self.visible_car() is not None,
            50,
            "visible tracked car from airborne camera",
        )
        car = self.visible_car()
        selected = car.id
        request = SelectTarget.Request()
        request.track_id = selected
        reply = self.service(self.select, request)
        assert reply.success, reply.message
        if enable_after_sim_seconds:
            self.wait(lambda: self.node.get_clock().now().nanoseconds
                      >= enable_after_sim_seconds * 1e9,
                      120, "moving-target engagement window")
        request = SetBool.Request()
        request.data = True
        enabled = False
        for attempt in range(10):
            self.stream = False
            reply = self.service(self.enable, request)
            if reply.success:
                enabled = True
                break
            self.stream = True
            print("ENABLE_NOT_READY", reply.message, flush=True)
            until = time.monotonic() + 0.2
            while time.monotonic() < until:
                rclpy.spin_once(self.node, timeout_sec=0.02)
        if not enabled:
            raise RuntimeError(reply.message)
        self.selected = selected
        self.wait(
            lambda: self.health.get("state") == "following",
            5,
            "following enabled for selected ID",
        )
        if recording_after_sim_seconds:
            def recording_ready():
                if self.health.get("state") == "holding":
                    raise RuntimeError("Following stopped before recording: "
                                       + self.health.get("reason", ""))
                return (self.node.get_clock().now().nanoseconds
                        >= recording_after_sim_seconds * 1e9
                        and self.health.get("state") == "following")
            self.wait(recording_ready, 120, "driving-scene recording start")
        start = time.monotonic()
        start_sim = self.node.get_clock().now().nanoseconds
        origin = [self.position.x, self.position.y, self.position.z]
        last = 0
        while (
            self.node.get_clock().now().nanoseconds - start_sim
            < duration_sim_seconds * 1e9
            and time.monotonic() - start < 180
        ):
            rclpy.spin_once(self.node, timeout_sec=0.05)
            assert (
                self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                and not self.status.failsafe
            ), "unexpected flight-mode loss"
            assert not self.m7_armed
            assert time.monotonic() - self.m7_seen < 2, "M7 telemetry expired"
            sample = {
                "t": round(time.monotonic() - start, 3),
                "ned": [self.position.x, self.position.y, self.position.z],
                "velocity_ned": [self.position.vx, self.position.vy, self.position.vz],
                "sim_ns": self.node.get_clock().now().nanoseconds,
                "following": dict(self.health),
            }
            self.samples.append(sample)
            self.sample_file.write(json.dumps(sample) + "\n")
            self.sample_file.flush()
            if time.monotonic() - last > 2:
                print("FOLLOW", json.dumps(sample), flush=True)
                last = time.monotonic()
            if self.health.get("state") == "holding":
                break
        duration = time.monotonic() - start
        duration_sim = (self.node.get_clock().now().nanoseconds - start_sim) / 1e9
        displacement = math.dist(origin[:2], [self.position.x, self.position.y])
        print(
            "FOLLOW_SUMMARY",
            json.dumps(
                {
                    "selected_id": selected,
                    "seconds": duration,
                    "displacement_m": displacement,
                    "health": self.health,
                }
            ),
            flush=True,
        )
        # An identity reset must not make the follower silently choose the new car ID.
        brake_origin = [self.position.x, self.position.y]
        brake_speed = math.hypot(self.position.vx, self.position.vy)
        brake_start = self.node.get_clock().now().nanoseconds
        self.service(self.reset, Trigger.Request())
        self.wait(
            lambda: self.health.get("state") == "holding",
            5,
            "target identity loss holds position",
        )
        stopping_distance = 0.0
        stopping_seconds = 0.0
        if braking_acceleration_mps2 is not None:
            # Observe the controller output directly; the 1 Hz diagnostics
            # report can arrive almost a second after braking has completed.
            def position_hold_ready():
                setpoint = self.latest_setpoint
                return (
                    setpoint is not None
                    and setpoint.timestamp * 1000 >= brake_start
                    and all(math.isfinite(v) for v in setpoint.position[:2])
                    and all(math.isnan(v) for v in setpoint.velocity[:2])
                    and math.hypot(self.position.vx, self.position.vy) < 0.5
                )

            self.wait(position_hold_ready, 30, "settled braking hold")
            stopping_seconds = (self.node.get_clock().now().nanoseconds
                                - brake_start) / 1e9
            stopping_distance = math.dist(
                brake_origin, [self.position.x, self.position.y])
            # Include one second for setpoint latency and jerk ramp, then
            # require the original tight drift bound once stationary.
            distance_budget = (brake_speed**2 / (2*braking_acceleration_mps2)
                               + brake_speed + 1.0)
            time_budget = brake_speed/braking_acceleration_mps2 + 2.0
            assert stopping_distance <= distance_budget, (
                "braking distance", stopping_distance, distance_budget)
            assert stopping_seconds <= time_budget, (
                "braking time", stopping_seconds, time_budget)
            print("PASS bounded stopping", stopping_seconds, stopping_distance,
                  flush=True)
        before = [self.position.x, self.position.y, self.position.z]
        end = time.monotonic() + 3
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        drift = math.dist(before, [self.position.x, self.position.y, self.position.z])
        assert drift < 1.0, ("hold drift", drift)
        assert (
            self.health.get("state") == "holding"
            and self.health.get("selected_track") == selected
        ), "unexpected target switch"
        request = SetBool.Request()
        request.data = False
        assert self.service(self.enable, request).success
        self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.wait(
            lambda: self.land.landed
            and self.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED,
            90,
            "automatic simulated landing and disarm",
        )
        assert not self.m7_armed
        report = {
            "detector_model": self.detector_model,
            "clock_delivery": dict(self.clock_delivery),
            "selected_id": selected,
            "following_seconds": duration,
            "following_sim_seconds": duration_sim,
            "required_following_sim_seconds": duration_sim_seconds,
            "horizontal_displacement_m": displacement,
            "hold_drift_m": drift,
            "stopping_distance_m": stopping_distance,
            "stopping_sim_seconds": stopping_seconds,
            "real_m7_remained_disarmed": True,
            "final_sim_arming_state": self.status.arming_state,
            "samples": self.samples,
        }
        with open(self.output_dir / "result.json", "w") as f:
            json.dump(report, f, indent=2)
        print(
            "RESULT",
            json.dumps({k: v for k, v in report.items() if k != "samples"}),
            flush=True,
        )
        following_fraction = sum(
            s["following"].get("state") == "following" for s in self.samples
        ) / len(self.samples)
        assert (
            duration_sim >= duration_sim_seconds
            and displacement > 1
            and following_fraction > 0.9
        ), "insufficient sustained closed-loop following"

    def run_hover(self, duration_wall_seconds=5.0, start_sim_seconds=0.0):
        if not 5 <= duration_wall_seconds <= 60 or not 0 <= start_sim_seconds <= 300:
            raise ValueError("Invalid hover recording duration or start time")
        self.wait(
            lambda: self.status
            and self.m7
            and self.position
            and self.land
            and self.position.xy_valid
            and self.position.z_valid,
            20,
            "isolated SITL and M7 telemetry",
        )
        assert (
            self.status.system_id == 2
            and self.m7.system_id == 1
            and self.status.arming_state == 1
        )
        self.goal = [
            float(self.position.x),
            float(self.position.y),
            float(self.position.z) - 4.5,
        ]
        self.stream = True
        self.wait(lambda: self.count >= 60, 8, "20 Hz offboard warmup")
        self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, [1, 6])
        self.wait(lambda: self.status.nav_state == 14, 5, "offboard mode")
        self.wait(
            lambda: self.status.pre_flight_checks_pass,
            15,
            "normal offboard preflight checks",
        )
        self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, [1, 0])
        self.wait(self.hover_ready, 40, "stable 4.5 m hover")
        if start_sim_seconds:
            self.wait(
                lambda: self.node.get_clock().now().nanoseconds
                >= start_sim_seconds * 1e9,
                180,
                "hover recording start time",
            )
        self.capture_start_sim_ns = self.node.get_clock().now().nanoseconds
        self.capture_origin = [self.position.x, self.position.y, self.position.z]
        self.hover_positions = []
        until = time.monotonic() + duration_wall_seconds
        while time.monotonic() < until:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            assert not self.m7_armed, "Unexpected M7 arming"
            assert (
                self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                and not self.status.failsafe
            ), "Unexpected hover flight-mode loss"
            self.hover_positions.append(
                [self.position.x, self.position.y, self.position.z]
            )
        self.capture_end_sim_ns = self.node.get_clock().now().nanoseconds
        self.stream = False
        self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.wait(
            lambda: self.land.landed and self.status.arming_state == 1,
            90,
            "landing and automatic disarm",
        )

    def close(self):
        try:
            if (
                self.status
                and self.status.system_id == 2
                and self.status.arming_state == 2
            ):
                self.stream = False
                self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.wait(
                    lambda: self.land
                    and self.land.landed
                    and self.status.arming_state == 1,
                    90,
                    "cleanup simulated landing",
                )
            if self.enable.service_is_ready():
                request = SetBool.Request()
                request.data = False
                self.service(self.enable, request)
        finally:
            self.raw_file.close()
            self.sample_file.close()
            self.node.destroy_node()

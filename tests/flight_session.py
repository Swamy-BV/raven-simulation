# SPDX-License-Identifier: AGPL-3.0-only
import json, math, time, struct, zlib
from collections import OrderedDict
from pathlib import Path
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleStatus,
    VehicleLocalPosition,
    VehicleLandDetected,
)
from vision_msgs.msg import Detection2DArray
from diagnostic_msgs.msg import DiagnosticArray
from raven_interfaces.srv import SelectTarget
from std_srvs.srv import SetBool, Trigger


class FlightSession:
    def __init__(self):
        self.output_dir = Path("/tmp/raven-simulation-results") / time.strftime(
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
        self.land = None
        self.tracks = None
        self.health = {}
        self.acks = []
        self.stream = False
        self.goal = None
        self.samples = []
        self.m7_armed = False
        self.last_print = 0
        self.m7_seen = 0.0
        self.count = 0
        self.subs = []
        self.images = OrderedDict()
        self.selected = None
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
                "/raven/vehicle_1/autonomy/following_health",
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
        self.select = self.node.create_client(
            SelectTarget, "/raven/vehicle_1/autonomy/select_target"
        )
        self.enable = self.node.create_client(
            SetBool, "/raven/vehicle_1/autonomy/set_following"
        )
        self.reset = self.node.create_client(
            Trigger, "/raven/vehicle_1/perception/reset_tracking"
        )
        self.timer = self.node.create_timer(0.05, self.publish_hold)

    def on_status(self, m):
        self.status = m

    def on_m7(self, m):
        self.m7_seen = time.monotonic()
        self.m7 = m
        self.m7_armed |= m.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def on_position(self, m):
        self.position = m

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
        if (
            not self.selected
            or self.health.get("state") not in ["following", "waiting", "holding"]
            or time.monotonic() - self.snapshot_time < 0.05
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
        s.yaw = 1.5707963267948966
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

    def run_follow(self):
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
            float(self.position.z) - 4.5,
        ]
        self.stream = True
        self.wait(lambda: self.count >= 50, 5, "20 Hz offboard warmup")
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
        self.wait(self.hover_ready, 40, "settled hover at 4.5 m")
        self.wait(
            lambda: self.node.get_clock().now().nanoseconds >= 65000000000,
            100,
            "car moving before target selection",
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
        start = time.monotonic()
        start_sim = self.node.get_clock().now().nanoseconds
        origin = [self.position.x, self.position.y, self.position.z]
        last = 0
        while (
            self.node.get_clock().now().nanoseconds - start_sim < 25000000000
            and time.monotonic() - start < 120
        ):
            rclpy.spin_once(self.node, timeout_sec=0.05)
            assert (
                self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                and not self.status.failsafe
            ), "unexpected flight-mode loss"
            assert not self.m7_armed
            sample = {
                "t": round(time.monotonic() - start, 3),
                "ned": [self.position.x, self.position.y, self.position.z],
                "following": dict(self.health),
            }
            self.samples.append(sample)
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
        self.service(self.reset, Trigger.Request())
        self.wait(
            lambda: self.health.get("state") == "holding",
            5,
            "target identity loss holds position",
        )
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
            "selected_id": selected,
            "following_seconds": duration,
            "following_sim_seconds": duration_sim,
            "horizontal_displacement_m": displacement,
            "hold_drift_m": drift,
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
            duration_sim >= 25 and displacement > 1 and following_fraction > 0.9
        ), "insufficient sustained closed-loop following"

    def run_hover(self):
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
        until = time.monotonic() + 5
        while time.monotonic() < until:
            rclpy.spin_once(self.node, timeout_sec=0.05)
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
            self.node.destroy_node()

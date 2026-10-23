#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-only
"""Evaluate VIO using only the simulator's internal flight controller."""
import argparse
import json
import shutil
import time
from pathlib import Path

import rclpy
from projectairsim_ros2_cpp.srv import Arm, Disarm, Land, MoveToPosition, Takeoff

from observe_vio import VioObserver


def spin_for(node, seconds):
    deadline = time.monotonic() + seconds
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


def call(node, service_type, robot, operation, values, timeout=45):
    client = node.create_client(service_type, f"/simulation/vio/{robot}/{operation}")
    try:
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError(f"Simulator service unavailable: {operation}")
        request = service_type.Request()
        for key, value in values.items():
            setattr(request, key, value)
        node.event("command", operation=operation, values=values)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done() or future.exception() or not future.result().success:
            raise RuntimeError(f"Simulator command failed or timed out: {operation}")
        node.event("command_complete", operation=operation)
    finally:
        node.destroy_client(client)


def estimator_resources(unit):
    group = Path("/sys/fs/cgroup/system.slice") / unit
    try:
        cpu = dict(line.split() for line in (group / "cpu.stat").read_text().splitlines())
        return {"usage_usec": int(cpu["usage_usec"]),
                "throttled_usec": int(cpu.get("throttled_usec", 0)),
                "memory_peak_bytes": int((group / "memory.peak").read_text()),
                "cpu_quota_available": (group / "cpu.max").exists()}
    except (OSError, ValueError, KeyError):
        return None


def run(args):
    profile = json.loads((args.config / "profile.json").read_text())
    robot_config = json.loads((args.config / "navigation-robot.jsonc").read_text())
    if robot_config["controller"]["type"] != "simple-flight-api":
        raise RuntimeError("This test supports only AirSim's internal controller")
    rclpy.init()
    observer = VioObserver(args.config, args.output, args.record_images)
    shutil.copytree(args.config, args.output / "config")
    resources_start = estimator_resources(args.estimator_unit)
    resources_time = time.monotonic()
    armed = False
    error = None
    try:
        spin_for(observer, 5)
        for required in ("image", "imu", "truth"):
            if observer.counts.get(required, 0) < 10:
                raise RuntimeError(f"Insufficient {required} samples before evaluation")
        if observer.invalid_stamps or observer.regressions:
            raise RuntimeError("Invalid or nonincreasing sensor timestamps")
        if not args.fly:
            spin_for(observer, args.seconds)
            return
        robot = profile["robot"]
        call(observer, Arm, robot, "arm", {"wait_on_last_task": True})
        armed = True
        call(observer, Takeoff, robot, "takeoff", {"wait_on_last_task": True})
        # OpenVINS completes visual initialization after the first motion update.
        # Only the internal simulator pilot takes off; VIO controls no aircraft.
        deadline = time.monotonic() + 5
        while observer.counts.get("odometry", 0) < 10 and time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=0.05)
        if observer.counts.get("odometry", 0) < 10:
            raise RuntimeError("No initialized VIO output after simulated takeoff")
        origin = profile["origin_ned_m"]
        observer.event("route_start")
        for waypoint in profile["flight"]["waypoints_relative_ned_m"]:
            if time.monotonic() - observer.last_received_wall.get("odometry", 0) > 1:
                raise RuntimeError("VIO output stopped during the route")
            target = [a + b for a, b in zip(origin, waypoint)]
            call(observer, MoveToPosition, robot, "move_to_position", {
                "x": target[0], "y": target[1], "z": target[2],
                "velocity": profile["flight"]["speed_mps"], "timeout_sec": 35.0,
                "drive_train_type": 0, "yaw_is_rate": False, "yaw": 0.0,
                "lookahead": -1.0, "adaptive_lookahead": 1.0, "wait_on_last_task": True})
            spin_for(observer, profile["flight"]["settle_seconds"])
        if time.monotonic() - observer.last_received_wall.get("odometry", 0) > 1:
            raise RuntimeError("VIO output stopped before route completion")
        observer.event("route_complete")
        spin_for(observer, 5)
    except Exception as exception:
        error = str(exception)
        observer.event("failure", error=error)
    finally:
        if armed:
            try:
                call(observer, Land, profile["robot"], "land", {"wait_on_last_task": True})
                call(observer, Disarm, profile["robot"], "disarm", {"wait_on_last_task": True})
                observer.event("landed_and_disarmed")
            except Exception as exception:
                error = f"{error or ''}; cleanup: {exception}"
        summary = observer.finish()
        resources_end = estimator_resources(args.estimator_unit)
        if resources_start is not None and resources_end is not None:
            elapsed = time.monotonic() - resources_time
            summary["estimator_resources"] = {
                "wall_seconds": elapsed,
                "cpu_percent_one_core": (resources_end["usage_usec"] - resources_start["usage_usec"]) / (elapsed * 10000),
                "cpu_quota_available": resources_end["cpu_quota_available"],
                "throttled_seconds": ((resources_end["throttled_usec"] - resources_start["throttled_usec"]) / 1e6
                                      if resources_end["cpu_quota_available"] else None),
                "memory_peak_bytes": resources_end["memory_peak_bytes"]}
        summary["flight_requested"] = args.fly
        summary["error"] = error
        (args.output / "run-result.json").write_text(json.dumps(summary, indent=2) + "\n")
        observer.destroy_node()
        rclpy.shutdown()
    if error:
        raise RuntimeError(error)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=10)
    parser.add_argument("--estimator-unit", default="visual-odometry.service", help="Systemd unit for optional resource measurements")
    parser.add_argument("--record-images", action="store_true", help="Save one raw frame per second for geometry diagnostics")
    parser.add_argument("--fly", action="store_true", help="Move only the AirSim internal-controller aircraft")
    run(parser.parse_args())


if __name__ == "__main__":
    main()

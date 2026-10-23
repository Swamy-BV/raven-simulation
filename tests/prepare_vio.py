#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-only
"""Prepare a fixed-camera, SimpleFlight-only VIO evaluation from upstream templates."""
import argparse
import json
import math
from pathlib import Path

import commentjson
import yaml


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2) + "\n")


class NoAliases(yaml.SafeDumper):
    def increase_indent(self, flow=False, indentless=False):
        return super().increase_indent(flow, False)

    def ignore_aliases(self, data):
        return True


def write_yaml(path, value):
    path.write_text("%YAML:1.0\n---\n" + yaml.dump(
        value, Dumper=NoAliases, sort_keys=False, default_flow_style=None))


def prepare(profile, robot, estimator, output):
    camera = profile["camera"]
    imu = profile["imu"]
    width, height = camera["width"], camera["height"]
    if not (160 <= width <= 1920 and 120 <= height <= 1080):
        raise ValueError("Navigation image dimensions are outside evaluation limits")
    if not 5 <= camera["rate_hz"] <= 60:
        raise ValueError("Navigation camera rate must be 5..60 Hz")
    if not 0 < profile["physics_step_ns"] <= 10000000:
        raise ValueError("Physics step must be positive and at most 10 ms")
    if robot["controller"]["type"] != "simple-flight-api":
        raise ValueError("VIO shadow evaluation requires the simulator's internal controller")
    output.mkdir(parents=True, exist_ok=False)
    capture = {"image-type": 0, "width": width, "height": height,
               "fov-degrees": camera["horizontal_fov_deg"],
               "capture-enabled": True, "streaming-enabled": True,
               "pixels-as-float": False, "compress": False}
    robot["sensors"] = [
        {"id": camera["id"], "type": "camera", "enabled": True,
         "parent-link": "Frame", "capture-interval": 1 / camera["rate_hz"],
         "capture-settings": [capture],
         "origin": {"xyz": " ".join(map(str, camera["position_frd_m"])),
                    "rpy-deg": f"0 {-camera['pitch_down_deg']} 0"}},
        {"id": imu["id"], "type": "imu", "enabled": True, "parent-link": "Frame",
         "accelerometer": {"velocity-random-walk": imu["accelerometer_noise_density"],
                           "bias-stability": imu["accelerometer_bias_stability"],
                           "tau": imu["accelerometer_tau_s"], "turn-on-bias": "0 0 0"},
         "gyroscope": {"angle-random-walk": imu["gyroscope_noise_density"],
                       "bias-stability": imu["gyroscope_bias_stability"],
                       "tau": imu["gyroscope_tau_s"], "turn-on-bias": "0 0 0"}}]
    scene = {"id": profile["id"], "scene-type": "UnrealNative",
             "actors": [{"type": "robot", "name": profile["robot"],
                         "origin": {"xyz": " ".join(map(str, profile["origin_ned_m"])),
                                    "rpy-deg": "0 0 0"},
                         "robot-config": "navigation-robot.jsonc", "start-landed": True}],
             "clock": {"type": "real-time", "step-ns": profile["physics_step_ns"],
                       "real-time-update-rate": profile["physics_step_ns"], "pause-on-start": False},
             "home-geo-point": {"latitude": 47.641468, "longitude": -122.140165, "altitude": 122.0}}
    prefix = f"/ProjectAirsim/{profile['id']}/robots/{profile['robot']}"
    topics = {"image": f"{prefix}/sensors/{camera['id']}/scene_camera",
              "imu": f"{prefix}/sensors/{imu['id']}/imu", "truth": f"{prefix}/actual_pose",
              "odometry": "/localization/vio/odometry"}
    # The upstream ROS bridge converts AirSim body FRD to FLU.
    # Camera pixels retain OpenCV right/down/forward optical axes.
    angle = math.radians(camera["pitch_down_deg"])
    c, s = math.cos(angle), math.sin(angle)
    x, y, z = camera["position_frd_m"]
    rotation = [[0.0, -s, c, x], [-1.0, 0.0, 0.0, -y], [0.0, -c, -s, -z], [0.0, 0.0, 0.0, 1.0]]
    focal = width / (2 * math.tan(math.radians(camera["horizontal_fov_deg"]) / 2))
    camera_chain = {"cam0": {"T_imu_cam": rotation, "cam_overlaps": [],
                              "camera_model": "pinhole", "distortion_model": "radtan",
                              "distortion_coeffs": [0.0] * 4,
                              "intrinsics": [focal, focal, width / 2, height / 2],
                              "resolution": [width, height], "rostopic": topics["image"]}}
    identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    imu_chain = {"imu0": {"T_i_b": [[1., 0., 0., 0.], [0., 1., 0., 0.],
                                    [0., 0., 1., 0.], [0., 0., 0., 1.]],
                          "accelerometer_noise_density": imu["accelerometer_noise_density"],
                          "accelerometer_random_walk": imu["accelerometer_bias_stability"] / math.sqrt(imu["accelerometer_tau_s"]),
                          "gyroscope_noise_density": imu["gyroscope_noise_density"],
                          "gyroscope_random_walk": imu["gyroscope_bias_stability"] / math.sqrt(imu["gyroscope_tau_s"]),
                          "rostopic": topics["imu"], "time_offset": 0.0,
                          "update_rate": 1e9 / profile["physics_step_ns"], "model": "kalibr",
                          "Tw": identity, "Ta": identity, "R_IMUtoGYRO": identity,
                          "R_IMUtoACC": identity, "Tg": [[0.0] * 3 for _ in range(3)]}}
    estimator.update({"max_cameras": 1, "use_stereo": False, "use_aruco": False,
                      "try_zupt": True, "zupt_only_at_beginning": True,
                      "zupt_chi2_multipler": 0.5, "zupt_noise_multiplier": 1.0,
                      "zupt_max_disparity": 0.1,
                      "calib_cam_extrinsics": False, "calib_cam_intrinsics": False,
                      "calib_cam_timeoffset": False, "num_opencv_threads": 2,
                      "init_dyn_mle_max_threads": 2, "track_frequency": camera["rate_hz"],
                      "relative_config_imu": "kalibr_imu_chain.yaml",
                      "relative_config_imucam": "kalibr_imucam_chain.yaml",
                      "record_timing_information": False, "save_total_state": False})
    write_json(output / "scene.jsonc", scene)
    write_json(output / "navigation-robot.jsonc", robot)
    write_json(output / "profile.json", profile)
    write_json(output / "topics.json", topics)
    write_yaml(output / "estimator_config.yaml", estimator)
    write_yaml(output / "kalibr_imucam_chain.yaml", camera_chain)
    write_yaml(output / "kalibr_imu_chain.yaml", imu_chain)
    return topics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--robot-template", type=Path, required=True)
    parser.add_argument("--estimator-template", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    robot = commentjson.loads(args.robot_template.read_text())
    estimator = yaml.safe_load(args.estimator_template.read_text().replace("%YAML:1.0", ""))
    topics = prepare(json.loads(args.profile.read_text()), robot, estimator, args.output)
    print(json.dumps(topics, indent=2))


if __name__ == "__main__":
    main()

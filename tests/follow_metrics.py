# SPDX-License-Identifier: AGPL-3.0-only
"""Offline evaluation only: simulator truth never feeds a controller."""

import argparse
import bisect
import json
import math
import statistics
from pathlib import Path


def load_lines(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line]


class Series:
    def __init__(self, rows):
        ordered = sorted(dict(rows).items())
        self.times = [t for t, _ in ordered]
        self.values = [v for _, v in ordered]

    def at(self, stamp):
        i = bisect.bisect_left(self.times, stamp)
        if i < len(self.times) and self.times[i] == stamp:
            return self.values[i]
        if i == 0 or i == len(self.times):
            return None
        before, after = self.times[i-1], self.times[i]
        if max(stamp-before, after-stamp) > 100_000_000:
            return None
        ratio = (stamp-before)/(after-before)
        return [a+(b-a)*ratio for a, b in zip(self.values[i-1], self.values[i])]

    def nearest_age_ms(self, stamp):
        i = bisect.bisect_left(self.times, stamp)
        candidates = [self.times[j] for j in (i-1, i) if 0 <= j < len(self.times)]
        return min(abs(stamp-t) for t in candidates)/1e6 if candidates else None


def distribution(values):
    if not values:
        return None
    ordered = sorted(values)
    return {"mean": statistics.mean(values), "rms": math.sqrt(
        statistics.mean(v*v for v in values)), "p95": ordered[
            math.ceil(.95*len(ordered))-1], "min": ordered[0], "max": ordered[-1]}


def translation(pose):
    return [pose["translation"][axis] for axis in ("x", "y", "z")]


def formation_error(delta, direction):
    along = sum(a*b for a, b in zip(delta, direction))
    cross = abs(delta[0]*direction[1]-delta[1]*direction[0])
    return along < 0, cross, math.degrees(math.atan2(cross, -along))


def projection_metrics(rows, truth, offset, start, end):
    errors, latencies = [], []
    for row in rows:
        if not start <= row["stamp_ns"] <= end:
            continue
        if row["frame"] != "px4_local_ned":
            raise ValueError("Ground estimate must be in the expected local NED frame")
        target = truth.at(row["stamp_ns"])
        if target is None:
            continue
        errors.append(math.dist([row["point"][i]+offset[i] for i in range(2)],
                                target[:2]))
        latencies.append((row["received_ns"]-row["stamp_ns"])/1e6)
    return {"matched_samples": len(errors), "error_to_actor_origin_m": distribution(errors),
            "capture_to_ros_receipt_ms": distribution(latencies)}


def evaluate(run, observations, result):
    poses = load_lines(Path(run)/"poses.jsonl")
    records = load_lines(observations)
    report = json.loads(Path(result).read_text())
    required_duration = report.get("required_following_sim_seconds", 25)
    if not 25 <= required_duration <= 60:
        raise ValueError("Invalid required following duration")
    samples = report["samples"]
    if not samples:
        raise ValueError("Following samples are missing")
    start, end = samples[0]["sim_ns"], samples[-1]["sim_ns"]
    vehicle = Series((r["stamp_ns"], r["ned"]) for r in records
                     if r["kind"] == "position")
    actual = Series((r["sim_ns"], translation(r["aircraft"])) for r in poses)
    truth = Series((r["sim_ns"], translation(r["target"])) for r in poses)
    attitude = Series((r["stamp_ns"], r["q"]) for r in records
                      if r["kind"] == "attitude")
    if not vehicle.times or not attitude.times:
        raise ValueError("Capture-time position and attitude recordings are required")
    stamps = [r["stamp_ns"] for r in records if r["kind"] == "target_observation"
              and start <= r["stamp_ns"] <= end]
    offsets = []
    for row in poses:
        if not start <= row["sim_ns"] <= min(end, start+2_000_000_000):
            continue
        local = vehicle.at(row["sim_ns"])
        if local is not None:
            offsets.append([a-b for a, b in zip(translation(row["aircraft"]), local)])
    if len(offsets) < 10:
        raise ValueError("Insufficient synchronized pose samples to align evaluation frames")
    offset = [statistics.median(v[i] for v in offsets) for i in range(3)]
    ranges, behind, lateral, trailing_angles = [], [], [], []
    response_delay = None
    forward_motion_threshold = 0.1
    for sample in samples:
        stamp = sample["sim_ns"]
        drone, target = actual.at(stamp), truth.at(stamp)
        if drone is None or target is None:
            continue
        delta = [drone[i]-target[i] for i in range(2)]
        ranges.append(math.hypot(*delta))
        before, after = truth.at(stamp-200_000_000), truth.at(stamp+200_000_000)
        if before is None or after is None:
            continue
        velocity = [(after[i]-before[i])/.4 for i in range(2)]
        speed = math.hypot(*velocity)
        if speed < .05:
            continue
        direction = [v/speed for v in velocity]
        forward_speed = sum(a*b for a, b in zip(sample["velocity_ned"][:2], direction))
        if response_delay is None and forward_speed >= forward_motion_threshold:
            response_delay = (stamp-start)/1e9
        is_behind, cross, angle = formation_error(delta, direction)
        behind.append(is_behind)
        lateral.append(cross)
        trailing_angles.append(angle)
    fraction = sum(s["following"].get("state") == "following" for s in samples)/len(samples)
    trailing_error = distribution(trailing_angles)
    maximum_trailing_angle = 30.0
    acceptance = {"rear_formation": bool(trailing_error) and
                  trailing_error["p95"] <= maximum_trailing_angle,"duration": report["following_sim_seconds"] >= required_duration,
                  "movement": report["horizontal_displacement_m"] > 1,
                  "following_fraction": fraction > .9,
                  "hold_drift": report["hold_drift_m"] < 1,
                  "m7_disarmed": report["real_m7_remained_disarmed"],
                  "sim_disarmed": report["final_sim_arming_state"] == 1}
    return {"passed": all(acceptance.values()),
            "detector_model": report.get("detector_model"), "acceptance": acceptance,
            "engagement_to_forward_motion_sim_seconds": response_delay,
            "forward_motion_threshold_mps": forward_motion_threshold,
            "required_following_sim_seconds": required_duration,
            "following_fraction": fraction, "following_sim_seconds": report["following_sim_seconds"],
            "horizontal_displacement_m": report["horizontal_displacement_m"],
            "hold_drift_m": report["hold_drift_m"],
            "evaluation_world_minus_local_ned_m": offset,
            "reference": "Scripted actor origin; box footpoint estimates a visible ground contact, not exact vehicle center",
            "pose_alignment_ms": {"position": distribution([vehicle.nearest_age_ms(t) for t in stamps]),
                                  "attitude": distribution([attitude.nearest_age_ms(t) for t in stamps])},
            "stand_off_m": distribution(ranges), "behind_moving_target_fraction":
            statistics.mean(behind) if behind else None,
            "cross_track_m": distribution(lateral),
            "trailing_bearing_error_deg": trailing_error,
            "max_trailing_bearing_error_p95_deg": maximum_trailing_angle,
            "raw_projection": projection_metrics([r for r in records if r["kind"] == "target_observation"], truth, offset, start, end),
            "filtered_projection": projection_metrics([r for r in records if r["kind"] == "target_position"], truth, offset, start, end)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("observations", type=Path)
    parser.add_argument("result", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    report = evaluate(args.run, args.observations, args.result)
    args.output.write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()

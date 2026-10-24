#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-only
"""Compare shadow VIO to independent truth without fitting scale or trajectory."""
import argparse
import json
from pathlib import Path

import numpy as np


def rotation_xyzw(quaternion):
    q = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("Invalid orientation quaternion")
    x, y, z, w = q / norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def statistics(values):
    values = np.asarray(values, dtype=float)
    return {"mean": float(np.mean(values)), "p95": float(np.percentile(values, 95)),
            "maximum": float(np.max(values))}


def compare(rows):
    streams = {kind: [r for r in rows if r["kind"] == kind]
               for kind in ("truth", "odometry", "imu", "image")}
    result = {"alignment": "first matched body orientation and position; rigid; no scale fitting",
              "counts": {k: len(v) for k, v in streams.items()}}
    result["rates_hz"] = {}
    result["maximum_stamp_gap_seconds"] = {}
    for kind, stream in streams.items():
        if len(stream) < 2:
            continue
        stamps = np.array([r["stamp_ns"] for r in stream], dtype=np.int64)
        if np.any(stamps <= 0):
            raise ValueError(f"Invalid {kind} timestamp")
        if np.any(np.diff(stamps) <= 0):
            raise ValueError(f"Nonincreasing {kind} timestamps")
        result["rates_hz"][kind] = float((len(stamps)-1) / ((stamps[-1]-stamps[0])/1e9))
        result["maximum_stamp_gap_seconds"][kind] = float(np.max(np.diff(stamps))/1e9)
    truth = streams["truth"]
    odometry = streams["odometry"]
    if len(truth) < 2 or len(odometry) < 2:
        result["comparable"] = False
        result["reason"] = "Insufficient reference or VIO samples"
        return result, None
    t0 = truth[0]["stamp_ns"]
    gt_times = np.array([(r["stamp_ns"]-t0)/1e9 for r in truth])
    gt_positions = np.array([r["position"] for r in truth])
    odometry = [r for r in odometry if truth[0]["stamp_ns"] <= r["stamp_ns"] <= truth[-1]["stamp_ns"]]
    if len(odometry) < 2:
        result["comparable"] = False
        result["reason"] = "No common acquisition-time interval"
        return result, None
    times = np.array([(r["stamp_ns"]-t0)/1e9 for r in odometry])
    reference = np.column_stack([np.interp(times, gt_times, gt_positions[:, i]) for i in range(3)])
    nearest = int(np.argmin(np.abs(gt_times-times[0])))
    # Both poses describe the same IMU/body FLU frame. Use only the first pair
    # to remove VIO's arbitrary world origin/yaw; never fit subsequent motion.
    alignment = rotation_xyzw(truth[nearest]["orientation_xyzw"]) @ rotation_xyzw(odometry[0]["orientation_xyzw"]).T
    positions = np.array([r["position"] for r in odometry])
    aligned = (alignment @ (positions-positions[0]).T).T + reference[0]
    errors = np.linalg.norm(aligned-reference, axis=1)
    reference_path = float(np.linalg.norm(np.diff(reference, axis=0), axis=1).sum())
    result.update({"comparable": True, "compared_seconds": float(times[-1]-times[0]),
                   "reference_duration_seconds": float(gt_times[-1]-gt_times[0]),
                   "odometry_time_coverage_fraction": float((times[-1]-times[0])/(gt_times[-1]-gt_times[0])),
                   "unobserved_reference_tail_seconds": float(gt_times[-1]-times[-1]),
                   "initialization_seconds_from_observer_start": float(times[0]),
                   "reference_path_m": reference_path,
                   "position_rmse_m": float(np.sqrt(np.mean(errors**2))),
                   "position_error_m": statistics(errors),
                   "end_position_error_m": float(errors[-1]),
                   "end_drift_percent_of_path": float(100*errors[-1]/reference_path) if reference_path > 0 else None,
                   "initial_alignment_truth_delta_ms": float(abs(gt_times[nearest]-times[0])*1000)})
    lags = [(r["latest_imu_ns"]-r["stamp_ns"])/1e6 for r in odometry if r.get("latest_imu_ns")]
    if lags:
        result["latest_received_imu_minus_odometry_ms"] = statistics(lags)
    # A route failure or lack of odometry must remain visible in the report.
    events = [r for r in rows if r["kind"] == "event"]
    result["route_complete"] = any(r["name"] == "route_complete" for r in events)
    result["landed_and_disarmed"] = any(r["name"] == "landed_and_disarmed" for r in events)
    result["failures"] = [r for r in events if r["name"] == "failure"]
    receive_start = min(r["receive_monotonic_ns"] for r in rows if "receive_monotonic_ns" in r)
    result["initialization_wall_seconds"] = (odometry[0]["receive_monotonic_ns"]-receive_start)/1e9
    return result, {"time_s": times, "reference": reference, "vio": aligned, "error_m": errors}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("samples", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rows = [json.loads(line) for line in args.samples.read_text().splitlines() if line]
    result, trajectory = compare(rows)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    if trajectory is not None:
        np.savez_compressed(args.output.with_suffix(".npz"), **trajectory)
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()

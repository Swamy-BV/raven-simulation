# SPDX-License-Identifier: AGPL-3.0-only
"""Offline HITL rear-formation and observed-identity measurements.

Fixture commands are evaluation references, never controller inputs. Their
poses do not establish exact render-tick placement or physical sensor truth.
"""

import argparse
import json
import math
from pathlib import Path
import re

from follow_metrics import distribution, formation_error


def rows(path):
    with path.open() as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


def identity_evidence(source, selected_track):
    visible_indices = [index for index, row in enumerate(source)
                       if any(track["id"] == selected_track for track in row["tracks"])]
    seen = [source[index]["source_ns"] for index in visible_indices]
    recovered = []
    for first, last in zip(visible_indices, visible_indices[1:]):
        gap = (source[last]["source_ns"] - source[first]["source_ns"]) / 1e9
        # A missing input stream is not evidence of visual occlusion recovery.
        # Require intervening tracker outputs and continuous acquisition time.
        if gap >= .3 and last - first >= 3 and all(
                source[i + 1]["source_ns"] - source[i]["source_ns"] <= 250_000_000
                for i in range(first, last)):
            recovered.append(gap)
    return seen, recovered


def evaluate(run):
    result = json.loads((run / "board/result.json").read_text())
    log = (run / "flight-test.log").read_text()
    ready = int(re.search(r"^FOLLOWING_READY (\d+)", log, re.M)[1])
    health = list(rows(run / "board/health.jsonl"))
    active = [row for row in health if row["ros_ns"] >= ready
              and row["health"].get("selected_track") == result["selected_track"]]
    end = result.get("following_end_ros_ns")
    if end is None:
        end = next((row["ros_ns"] for row in active
                    if row["health"].get("state") == "holding"), active[-1]["ros_ns"])
    first = next(rows(run / "frames.jsonl"))
    offset = first["capture_ns"] - first["source_sim_ns"]
    ranges, angles, cross_track, behind, speeds = [], [], [], [], []
    previous = None
    last_sample = 0
    for row in rows(run / "poses.jsonl"):
        stamp = row["sim_ns"] + offset
        if not ready <= stamp <= end or stamp - last_sample < 100_000_000:
            continue
        last_sample = stamp
        aircraft = row["aircraft"]["translation"]
        target = row["target"]["translation"]
        delta = [aircraft[axis] - target[axis] for axis in ("x", "y")]
        ranges.append(math.hypot(*delta))
        if previous:
            dt = (stamp - previous[0]) / 1e9
            speed = math.hypot(*(target[axis] - previous[1][axis]
                                 for axis in ("x", "y"))) / dt
            speeds.append(speed)
            if speed >= 2:
                q = row["target"]["rotation"]
                yaw = math.atan2(2 * (q["w"] * q["z"] + q["x"] * q["y"]),
                                 1 - 2 * (q["y"] ** 2 + q["z"] ** 2))
                rear, cross, angle = formation_error(delta, [math.cos(yaw), math.sin(yaw)])
                behind.append(rear)
                cross_track.append(cross)
                angles.append(angle)
        previous = stamp, target
    source = [row for row in rows(run / "board/tracks.jsonl")
              if ready <= row["source_ns"] <= end]
    seen, recovered = identity_evidence(source, result["selected_track"])
    gaps = [(b - a) / 1e9 for a, b in zip(seen, seen[1:])]
    following = [row for row in active if row["ros_ns"] <= end]
    return {
        "flight_test_passed": result["passed"],
        "reviewed_seconds_after_handover": (end - ready) / 1e9,
        "selected_visible_fraction_of_tracker_frames": len(seen) / len(source) if source else None,
        "largest_selected_observation_gap_seconds": max(gaps) if gaps else None,
        "terminal_selected_missing_seconds": (end - seen[-1]) / 1e9 if seen else None,
        "recovered_gaps_above_300ms": recovered,
        "following_fraction_of_health_samples": sum(
            row["health"].get("state") == "following" for row in following) / len(following),
        "rear_formation_when_target_above_2mps": {
            "behind_fraction": sum(behind) / len(behind) if behind else None,
            "bearing_error_degrees": distribution(angles),
            "cross_track_m": distribution(cross_track),
        },
        "stand_off_m": distribution(ranges),
        "fixture_speed_mps": distribution(speeds),
        "reference": "Elapsed-time fixture commands and asynchronously sampled aircraft pose; evaluation only",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--require-rear-follow", action="store_true")
    parser.add_argument("--require-occlusion", action="store_true")
    args = parser.parse_args()
    report = evaluate(args.run)
    rear = report["rear_formation_when_target_above_2mps"]
    gates = {"flight": report["flight_test_passed"]}
    if args.require_rear_follow:
        gates["rear_formation"] = bool(rear["bearing_error_degrees"]) and (
            rear["behind_fraction"] >= .99 and rear["bearing_error_degrees"]["p95"] <= 30)
        gates["stand_off"] = bool(report["stand_off_m"]) and report["stand_off_m"]["min"] >= 6
    if args.require_occlusion:
        gates["same_identity_recovered"] = bool(report["recovered_gaps_above_300ms"])
    report["acceptance"] = gates
    report["passed"] = all(gates.values())
    (args.run / "rear-following-metrics.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["passed"] else 1)


if __name__ == "__main__":
    main()

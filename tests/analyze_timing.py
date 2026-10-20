# SPDX-License-Identifier: AGPL-3.0-only
"""Summarize debug timing lines from one board boot and simulation clock session."""

import argparse
import json
import math
import re
from collections import defaultdict
from pathlib import Path

PATTERN = re.compile(r"timing stage=(\w+) source_ns=(\d+) ros_ns=(\d+) steady_ns=(\d+)")


def distribution(values):
    if not values:
        return None
    values = sorted(values)
    return {
        "count": len(values),
        "p50_ms": values[math.ceil(0.5 * len(values)) - 1],
        "p95_ms": values[math.ceil(0.95 * len(values)) - 1],
        "max_ms": values[-1],
        "min_ms": values[0],
    }


def summarize(text):
    samples = defaultdict(dict)
    ages = defaultdict(list)
    moving_command_ages = []
    durations = defaultdict(list)
    seen = set()
    nonmonotonic = 0
    last = {}
    for match in PATTERN.finditer(text):
        stage, source, ros, steady = match.groups()
        source, ros, steady = int(source), int(ros), int(steady)
        if not source or (stage, source, steady) in seen:
            continue
        seen.add((stage, source, steady))
        if stage != "setpoint" and stage in last and source < last[stage]:
            nonmonotonic += 1
        last[stage] = source
        if stage == "setpoint":
            # A nonzero source identifies a perception-based moving command.
            # Holds are traced with source 0 and excluded above.
            moving_command_ages.append((ros - source) / 1e6)
        if stage not in samples[source]:
            ages[stage].append((ros - source) / 1e6)
            samples[source][stage] = steady
    for stages in samples.values():
        for first, second in [
            ("capture", "inference"),
            ("inference", "tracking"),
            ("tracking", "setpoint"),
            ("capture", "setpoint"),
        ]:
            if first in stages and second in stages:
                elapsed = stages[second] - stages[first]
                if elapsed < 0:
                    raise ValueError("Stage order or board clock is invalid")
                durations[first + "_to_" + second].append(elapsed / 1e6)
    return {
        "clock_policy": "source age uses ROS time; same-board stage durations use steady wall time",
        "source_age_sim_ms": {key: distribution(value) for key, value in ages.items()},
        "board_stage_wall_ms": {
            key: distribution(value) for key, value in durations.items()
        },
        "moving_command_age_sim_ms": distribution(moving_command_ages),
        "unique_source_frames": len(samples),
        "nonmonotonic_source_observations": nonmonotonic,
        "matched_frame_counts": {key: len(value) for key, value in durations.items()},
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument(
        "--sender-run",
        type=Path,
        help="Bridge run containing frames.jsonl and encoded.jsonl",
    )
    options = parser.parse_args()
    report = summarize(options.trace.read_text())
    if not report["unique_source_frames"]:
        parser.error("No timing samples; enable node debug logging")
    if options.sender_run:
        frames = [
            json.loads(line)
            for line in (options.sender_run / "frames.jsonl").read_text().splitlines()
        ]
        encoded = [
            json.loads(line)
            for line in (options.sender_run / "encoded.jsonl").read_text().splitlines()
        ]
        inputs = {frame["source_sim_ns"]: frame for frame in frames}
        latencies = [
            (
                frame["encoded_steady_ns"]
                - inputs[frame["source_sim_ns"]]["receive_steady_ns"]
            )
            / 1e6
            for frame in encoded
            if frame["source_sim_ns"] in inputs
        ]
        if any(value < 0 for value in latencies):
            raise ValueError("Invalid sender trace clock ordering")
        span = (
            (frames[-1]["receive_steady_ns"] - frames[0]["receive_steady_ns"]) / 1e9
            if len(frames) > 1
            else 0
        )
        report["sender"] = {
            "sdk_receive_to_encoded_wall_ms": distribution(latencies),
            "received_frames": len(frames),
            "encoded_frames": len(encoded),
            "source_wall_fps": (len(frames) - 1) / span if span else None,
            "max_appsrc_queued_bytes": max(
                (frame["queued_bytes"] for frame in frames), default=0
            ),
            "cross_host_wall_latency": "not inferred from unsynchronized host steady clocks",
        }
    print(json.dumps(report, indent=2))

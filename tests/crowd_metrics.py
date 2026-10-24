# SPDX-License-Identifier: AGPL-3.0-only
"""Post-flight cyclist identity checks; fixture poses never feed a controller.

The observed box footpoint is intersected with the simulated road using the
recorded render-camera pose. Near neighbours or uncertain projections remain
ambiguous. This is a geometric reference check, not a ReID certification.
"""

import argparse
import bisect
import json
import math
from pathlib import Path
import re

from follow_metrics import Series
from hitl_metrics import rows


def ground_point(box, camera_pose, focal, ground_z):
    x, y, _, height = box
    ray = [1, (x - 320) / focal, (y + height / 2 - 180) / focal]
    q = camera_pose[3:]
    norm = math.sqrt(sum(value * value for value in q))
    if norm < 1e-9:
        return None
    w, a, b, c = [value / norm for value in q]
    cross = [b * ray[2] - c * ray[1], c * ray[0] - a * ray[2],
             a * ray[1] - b * ray[0]]
    direction = [
        ray[0] + 2 * (w * cross[0] + b * cross[2] - c * cross[1]),
        ray[1] + 2 * (w * cross[1] + c * cross[0] - a * cross[2]),
        ray[2] + 2 * (w * cross[2] + a * cross[1] - b * cross[0]),
    ]
    if direction[2] <= .05:
        return None
    scale = (ground_z - camera_pose[2]) / direction[2]
    if scale <= 0:
        return None
    return [camera_pose[i] + scale * direction[i] for i in range(2)]


def classify(point, actors, expected):
    ranked = sorted((math.dist(point, position), name)
                    for name, position in actors.items())
    residual, nearest = ranked[0]
    margin = ranked[1][0] - residual if len(ranked) > 1 else math.inf
    if residual > 2.0 or margin < .5:
        return "ambiguous", nearest, residual
    return ("correct" if nearest == expected else "other"), nearest, residual


def evaluate(run):
    bindings = json.loads((run / "config/bindings.json").read_text())
    result = json.loads((run / "board/result.json").read_text())
    ready = int(re.search(r"^FOLLOWING_READY (\d+)",
                          (run / "flight-test.log").read_text(), re.M)[1])
    end = result.get("following_end_ros_ns")
    if end is None:
        end = next(row["ros_ns"] for row in rows(run / "board/health.jsonl")
                   if row["ros_ns"] > ready and row["health"].get("state") == "holding")
    expected = bindings["target"]["name"]
    names = [expected] + [actor["name"] for actor in bindings["traffic"]
                          if actor.get("asset") == "Cyclist"]
    poses = list(rows(run / "poses.jsonl"))
    series = {name: [] for name in names}
    for pose in poses:
        target = pose["target"]["translation"]
        series[expected].append((pose["sim_ns"], [target["x"], target["y"]]))
        for actor in pose["traffic"]:
            if actor["name"] in series:
                position = actor["pose"]["translation"]
                series[actor["name"]].append((pose["sim_ns"], [position["x"], position["y"]]))
    series = {name: Series(samples) for name, samples in series.items()}
    frames = list(rows(run / "frames.jsonl"))
    times = [frame["capture_ns"] for frame in frames]
    calibration = json.loads((run / "config/camera.yaml").read_text())
    focal = calibration["camera_matrix"]["data"][0]
    counts = dict(correct=0, other=0, ambiguous=0, unavailable=0, missing=0)
    residuals = []
    wrong_start = None
    wrong_actor = None
    longest_wrong = 0.0
    total = 0
    for row in rows(run / "board/tracks.jsonl"):
        if not ready <= row["source_ns"] <= end:
            continue
        total += 1
        track = next((track for track in row["tracks"]
                      if track["id"] == result["selected_track"]), None)
        if track is None:
            counts["missing"] += 1
            wrong_start = None
            continue
        index = bisect.bisect_left(times, row["source_ns"])
        candidates = [i for i in (index - 1, index) if 0 <= i < len(frames)]
        frame = frames[min(candidates, key=lambda i: abs(times[i] - row["source_ns"]))]
        actors = {name: value.at(frame["source_sim_ns"]) for name, value in series.items()}
        point = ground_point(track["bbox"], frame["render_camera_pose"], focal,
                             bindings["road_mesh"]["surface_z_m"])
        if (abs(frame["capture_ns"] - row["source_ns"]) > 100_000_000
                or point is None or any(value is None for value in actors.values())):
            counts["unavailable"] += 1
            wrong_start = None
            continue
        state, nearest, residual = classify(point, actors, expected)
        counts[state] += 1
        residuals.append(residual)
        if state == "other":
            if wrong_start is None or nearest != wrong_actor:
                wrong_start = row["source_ns"]
            wrong_actor = nearest
            longest_wrong = max(longest_wrong, (row["source_ns"] - wrong_start) / 1e9)
        else:
            wrong_start = None
    observed = total - counts["missing"]
    gates = dict(flight=result["passed"],
                 correct_fixture_fraction=counts["correct"] / max(1, observed) >= .95,
                 other_fixture_fraction=counts["other"] / max(1, observed) <= .01,
                 no_sustained_wrong_fixture=longest_wrong < .3,
                 selected_visible=observed / max(1, total) >= .9)
    return dict(passed=all(gates.values()), acceptance=gates, cyclist_count=len(names),
                expected_fixture=expected, counts=counts, tracker_frames=total,
                correct_fraction_of_observed=counts["correct"] / max(1, observed),
                longest_wrong_fixture_seconds=longest_wrong,
                maximum_nearest_fixture_residual_m=max(residuals, default=None),
                reference="Recorded render camera and asynchronous fixture poses; evaluation only",
                limits="2m residual, 0.5m nearest-neighbour margin; partial boxes can be ambiguous")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    report = evaluate(args.run)
    (args.run / "crowd-identity-metrics.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["passed"] else 1)


if __name__ == "__main__":
    main()

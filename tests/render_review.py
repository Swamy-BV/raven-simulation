# SPDX-License-Identifier: AGPL-3.0-only
"""Render recorded ROS tracks onto original footage using Windows NVDEC/NVENC.

This offline review tool does not publish observations or command an aircraft.
Boxes retain their last observed coordinates for at most 200 ms. Playback retains the recorded wall-time cadence, including slow simulation.
"""

import argparse
import bisect
import json
import math
from pathlib import Path


def read_rows(path):
    with path.open() as stream:
        return [json.loads(line) for line in stream if line.strip()]


class Timeline:
    def __init__(self, rows):
        self.rows = sorted(rows, key=lambda row: row["stamp_ns"])
        self.times = [row["stamp_ns"] for row in self.rows]

    def previous(self, stamp, maximum_age_ns):
        index = bisect.bisect_right(self.times, stamp) - 1
        if index < 0 or stamp - self.times[index] > maximum_age_ns:
            return None
        return self.rows[index]


def load_review(observations, result, include_ready=False):
    if result.get("system_id") != 1:
        samples = result["samples"]
        rows = read_rows(observations)
        return rows, samples[0]["sim_ns"] - 3000000000, samples[-1]["sim_ns"] + 3000000000, "source_sim_ns", "PX4 SITL"
    directory = observations.parent
    samples = json.loads((directory / "following.json").read_text())
    positions = json.loads((directory / "positions.json").read_text())
    health_file = directory / "health.jsonl"
    states = read_rows(health_file) if health_file.exists() else samples
    rows = [dict(row, kind="following", stamp_ns=row["ros_ns"])
            for row in states]
    rows.extend(dict(row, kind="position", stamp_ns=row["px4_us"] * 1000)
                for row in positions)
    for row in read_rows(observations):
        rows.append({"kind": "tracks", "stamp_ns": row["source_ns"],
                     "observations": [dict(id=box["id"],
                                           cx=box["bbox"][0], cy=box["bbox"][1],
                                           w=box["bbox"][2], h=box["bbox"][3],
                                           score=box["score"], **{"class": box["class_id"]})
                                      for box in row["tracks"]]})
    start_key = "following_ready_ros_ns" if include_ready else "following_start_ros_ns"
    return rows, result[start_key], result["following_end_ros_ns"], "capture_ns", "Actual M7 PX4 | HITL"



def finite_number(value):
    try:
        value = float(value)
        return value if math.isfinite(value) else None
    except (TypeError, ValueError):
        return None


def planar_speed(values):
    if not values or len(values) < 2:
        return None
    x, y = (finite_number(v) for v in values[:2])
    return math.hypot(x, y) if x is not None and y is not None else None


def yaw_degrees_error(command, attitude):
    if command is None or attitude is None:
        return None
    w, x, y, z = attitude["q"]
    measured = math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
    return math.degrees(math.remainder(command - measured, 2 * math.pi))


def draw_telemetry(context, stamp, state, pose, commands, attitudes, positions, text):
    command = commands.previous(stamp, 250000000)
    attitude = attitudes.previous(stamp, 250000000)
    values = state["health"] if state else {}
    actual = planar_speed(pose["velocity_ned"]) if pose else None
    requested = planar_speed(command["velocity"]) if command else None
    yaw = finite_number(command["yaw"]) if command else None
    error = yaw_degrees_error(yaw, attitude)
    rate = finite_number(command["yawspeed"]) if command else None
    fmt = lambda value, digits=1: f"{value:.{digits}f}" if value is not None else "--"
    field = lambda key: finite_number(values.get(key))
    mph = lambda value: value * 2.2369362921 if value is not None else None
    step = field("control_step_ms")
    lines = [
        "RECORDED GUIDANCE / PX4",
        f"Target est.  {fmt(mph(field('target_speed_mps')))} mph",
        f"Range est.   {fmt(field('target_range_m'))} m",
        f"Speed cmd/PX4  {fmt(mph(requested))} / {fmt(mph(actual))} mph",
        f"Yaw error  {fmt(error)} deg",
        f"Yaw command  {fmt(math.degrees(rate) if rate is not None else None)} deg/s",
        f"Input age  {fmt(field('source_age_ms'), 0)} ms",
        f"Control  {fmt(1000 / step if step and step > 0 else None, 0)} Hz | Plan {fmt(field('planning_duration_ms'), 2)} ms",
    ]
    context.set_source_rgba(0, 0, 0, .76)
    context.rectangle(430, 231, 204, 116)
    context.fill()
    for i, line in enumerate(lines):
        text(context, 438, 245 + 13 * i, line, size=10 if i else 10.5)

    # Plot commands actually sent to PX4 against measured PX4 speed. No fixture
    # trajectory is used, and missing intervals are left disconnected.
    x, y, width, height = 12, 281, 200, 46
    context.set_source_rgba(0, 0, 0, .76)
    context.rectangle(x-5, y-22, width+10, height+28)
    context.fill()
    text(context, x, y-9, "Speed 0-50 mph | last 6 s", size=10)
    context.set_source_rgba(1, 1, 1, .25)
    context.set_line_width(.6)
    for limit in (0, 25, 50):
        ordinate = y + height * (1-limit/50)
        context.move_to(x, ordinate)
        context.line_to(x+width, ordinate)
    context.stroke()
    for timeline, key, color in ((commands, "velocity", (.2, .8, 1)),
                                 (positions, "velocity_ned", (1, .75, .15))):
        left = bisect.bisect_left(timeline.times, stamp-6000000000)
        right = bisect.bisect_right(timeline.times, stamp)
        points = timeline.rows[left:right]
        stride = max(1, len(points)//300)
        context.set_source_rgb(*color)
        context.set_line_width(1.2)
        previous = None
        for row in points[::stride]:
            value = planar_speed(row[key])
            if value is None:
                previous = None
                continue
            px = x + width * (row["stamp_ns"] - stamp + 6000000000) / 6000000000
            py = y + height * (1-min(50, value*2.2369362921)/50)
            if previous is None or row["stamp_ns"]-previous > 250000000:
                context.move_to(px, py)
            else:
                context.line_to(px, py)
            previous = row["stamp_ns"]
        context.stroke()
    context.set_source_rgb(.2, .8, 1)
    context.rectangle(15, 332, 8, 2)
    context.fill()
    text(context, 27, 337, "Command", size=9)
    context.set_source_rgb(1, .75, .15)
    context.rectangle(92, 332, 8, 2)
    context.fill()
    text(context, 104, 337, "PX4 measured", size=9)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("observations", type=Path)
    parser.add_argument("result", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--telemetry", action="store_true",
                        help="Show recorded guidance, PX4 response and tracking identity/confidence")
    parser.add_argument("--include-ready", action="store_true",
                        help="Include the stationary lead-in after following readiness")
    parser.add_argument("--start-seconds", type=float,
                        help="Optional start in recorded video seconds")
    parser.add_argument("--duration-seconds", type=float,
                        help="Optional wall-time duration in seconds")
    args = parser.parse_args()
    if args.start_seconds is not None and args.start_seconds < 0:
        parser.error("Start must be nonnegative")
    if args.duration_seconds is not None and args.duration_seconds <= 0:
        parser.error("Duration must be positive")
    if args.output.exists():
        parser.error("Output already exists")
    result = json.loads(args.result.read_text())
    rows, start, stop, clock_key, controller_label = load_review(args.observations, result, args.include_ready)
    if args.telemetry and result.get("system_id") != 1:
        parser.error("Telemetry view currently requires M7 HITL logs")
    commands = Timeline([])
    attitudes = Timeline([])
    if args.telemetry:
        commands = Timeline([dict(row, stamp_ns=row["received_ros_ns"])
                             for row in read_rows(args.observations.parent / "setpoints.jsonl")])
        attitudes = Timeline([dict(row, stamp_ns=row["source_ns"])
                              for row in read_rows(args.observations.parent / "attitude.jsonl")])
    selected_ids = list(dict.fromkeys(row["health"].get("selected_track") for row in rows
                                     if row["kind"] == "following" and row["health"].get("selected_track"))) if args.telemetry else []
    aliases = {identity: f"{i+1:02d}" for i, identity in enumerate(selected_ids)}
    frames = read_rows(args.run / "frames.jsonl")
    encoded = read_rows(args.run / "encoded.jsonl")
    by_source = {row["source_sim_ns"]: row for row in frames}
    # The sender uses zero B-frames. MP4 rebases its first encoded PTS to zero.
    origin_pts = by_source[encoded[0]["source_sim_ns"]]["pts_ns"]
    encoded_frames = [by_source[row["source_sim_ns"]] for row in encoded]
    pts = [row["pts_ns"] - origin_pts for row in encoded_frames]
    tracks = Timeline([row for row in rows if row["kind"] == "tracks"])
    position = Timeline([row for row in rows if row["kind"] == "position"])
    health = Timeline([row for row in rows if row["kind"] == "following"])
    included = [row for row in encoded_frames
                if start <= row[clock_key] <= stop]
    if not included:
        raise RuntimeError("No recorded video frames cover the flight")
    begin = included[0]["pts_ns"] - origin_pts
    end = included[-1]["pts_ns"] - origin_pts
    if args.start_seconds is not None:
        begin = max(begin, int(args.start_seconds * 1e9))
    if args.duration_seconds is not None:
        end = min(end, begin + int(args.duration_seconds * 1e9))
    if end <= begin:
        raise RuntimeError("Requested clip does not overlap recorded flight")
    # Accurate seeks may clamp a partial first frame's PTS to the seek time.
    # Seek at a recorded frame boundary to preserve source-frame identity.
    begin_index = bisect.bisect_left(pts, begin)
    end_index = bisect.bisect_right(pts, end) - 1
    if begin_index >= end_index:
        raise RuntimeError("Requested clip contains too few source frames")
    begin, end = pts[begin_index], pts[end_index]

    import gi
    gi.require_version("Gst", "1.0")
    gi.require_foreign("cairo")
    from gi.repository import Gst
    Gst.init(None)
    required = ["nvh264dec", "nvh264enc", "cairooverlay"]
    if any(Gst.ElementFactory.find(name) is None for name in required):
        raise RuntimeError("Windows GStreamer NVDEC, NVENC and Cairo are required")
    # Seek only the decoder. Seeking a prerolling encoder/muxer can leave an
    # invalid MP4; the output graph receives a fresh zero-based time segment.
    decoder = Gst.parse_launch(
        "filesrc name=input ! qtdemux ! h264parse ! nvh264dec "
        "! videoconvert ! video/x-raw,format=BGRA "
        "! appsink name=frames sync=false max-buffers=4 drop=false")
    decoder.get_by_name("input").set_property("location", str(args.run / "camera.mp4"))
    encoder = Gst.parse_launch(
        "appsrc name=frames format=time block=true is-live=false "
        "! cairooverlay name=overlay ! videoconvert ! video/x-raw,format=NV12 "
        "! nvh264enc preset=p4 tune=ultra-low-latency rc-mode=cbr "
        "bitrate=8000 gop-size=30 bframes=0 zerolatency=true "
        "! h264parse ! mp4mux faststart=true ! filesink name=output")
    encoder.get_by_name("output").set_property("location", str(args.output))
    first_pts = None
    output_size = (640, 360)
    counts = {"frames": 0, "with_tracks": 0, "unmatched_frames": 0}

    def text(context, x, y, value, size=13):
        context.set_source_rgb(1, 1, 1)
        context.set_font_size(size)
        context.move_to(x, y)
        context.show_text(value)

    def draw(_overlay, context, timestamp, _duration):
        timestamp += first_pts
        index = bisect.bisect_left(pts, timestamp)
        candidates = [i for i in [index - 1, index] if 0 <= i < len(pts)]
        index = min(candidates, key=lambda i: abs(pts[i] - timestamp))
        counts["frames"] += 1
        if abs(pts[index] - timestamp) > 2000000:
            counts["unmatched_frames"] += 1
            return
        context.save()
        context.scale(output_size[0] / 640, output_size[1] / 360)
        stamp = encoded_frames[index][clock_key]
        track = tracks.previous(stamp, 200000000)
        state = health.previous(stamp, 1500000000)
        pose = position.previous(stamp, 200000000)
        context.set_source_rgba(0, 0, 0, .72)
        context.rectangle(0, 0, 640, 70)
        context.fill()
        hardware = result.get("system_id") == 1
        title = "Raven | NXP i.MX95" if hardware else "Raven | Visual Tracking"
        details = ("PX4 / Cortex-M7 | Linux / Cortex-A55 | NPU vision | ROS 2 DDS"
                   if hardware else "PX4 SITL | Linux vision | ROS 2 DDS")
        text(context, 10, 18, title)
        text(context, 10, 37, details, size=11)
        speed = math.hypot(*pose["velocity_ned"][:2]) if pose else None
        speed_label = f"Drone {speed * 2.2369362921:.0f} mph" if speed is not None else "Speed unavailable"
        state_label = state["health"].get("state", "unknown") if state else "unknown"
        height_label = ""
        if pose and result.get("system_id") == 1:
            height = result["origin_ned"][2] - pose["ned"][2]
            height_label = f" | Alt {height:.1f} m"
        text(context, 10, 59, f"{state_label.capitalize()} | {speed_label}{height_label}", size=12)
        text(context, 10, 348, "AirSim | Hardware-in-the-loop" if hardware
             else "AirSim | Software-in-the-loop", size=10)
        if args.telemetry:
            draw_telemetry(context, stamp, state, pose, commands, attitudes, position, text)
        if track and track["observations"]:
            counts["with_tracks"] += 1
            selected = track.get("selected")
            if state and result.get("system_id") == 1:
                selected = state["health"].get("selected_track")
            for box in track["observations"]:
                active = box["id"] == selected
                context.set_source_rgb(*( (.1, 1, .3) if active else (1, .8, .1)))
                context.set_line_width(2)
                x = box["cx"] - box["w"] / 2
                y = box["cy"] - box["h"] / 2
                context.rectangle(x, y, box["w"], box["h"])
                context.stroke()
                if active:
                    class_label = {"0": "person", "2": "car"}.get(str(box["class"]), "target")
                    label = f"Selected {class_label}"
                    if args.telemetry:
                        label += f" | ID {aliases.get(box['id'], '--')} | det {box['score']:.0%}"
                    text(context, max(2, x), max(78 if args.telemetry else 68, y - 5), label, size=12)
        context.restore()

    encoder.get_by_name("overlay").connect("draw", draw)
    try:
        decoder.set_state(Gst.State.PAUSED)
        if decoder.get_state(15000000000)[0] == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("Review decoder could not preroll")
        if not decoder.seek(1.0, Gst.Format.TIME,
                            Gst.SeekFlags.FLUSH | Gst.SeekFlags.ACCURATE,
                            Gst.SeekType.SET, begin, Gst.SeekType.SET, end):
            raise RuntimeError("Review seek failed")
        sink = decoder.get_by_name("frames")
        app = encoder.get_by_name("frames")
        decoder.set_state(Gst.State.PLAYING)
        while True:
            sample = sink.emit("try-pull-sample", 15000000000)
            if sample is None:
                error = decoder.get_bus().pop_filtered(Gst.MessageType.ERROR)
                if error is not None:
                    raise RuntimeError(str(error.parse_error()))
                if not sink.get_property("eos"):
                    raise RuntimeError("Review decoder stalled")
                break
            if first_pts is None:
                first_pts = sample.get_buffer().pts
                caps = sample.get_caps().get_structure(0)
                output_size = (caps.get_value("width"), caps.get_value("height"))
                app.set_property("caps", sample.get_caps())
                encoder.set_state(Gst.State.PLAYING)
            buffer = sample.get_buffer().copy_deep()
            buffer.pts -= first_pts
            buffer.dts = Gst.CLOCK_TIME_NONE
            if app.emit("push-buffer", buffer) != Gst.FlowReturn.OK:
                raise RuntimeError("Review encoder rejected a frame")
        app.emit("end-of-stream")
        message = encoder.get_bus().timed_pop_filtered(
            30000000000, Gst.MessageType.EOS | Gst.MessageType.ERROR)
        if message is None:
            raise RuntimeError("Review encoding timed out")
        if message.type == Gst.MessageType.ERROR:
            raise RuntimeError(str(message.parse_error()))
    finally:
        decoder.set_state(Gst.State.NULL)
        encoder.set_state(Gst.State.NULL)
    if counts["unmatched_frames"] or not counts["with_tracks"]:
        raise RuntimeError(f"Review source alignment failed: {counts}")
    report = dict(counts, controller=controller_label, clock=clock_key,
                  width=output_size[0], height=output_size[1],
                  start_seconds=begin / 1e9, end_seconds=end / 1e9,
                  decoder="nvh264dec", encoder="nvh264enc",
                  overlay="Recorded ROS boxes held at most 200 ms; selected object labelled",
                  telemetry=("Recorded control-time input age; ground-plane range and target-speed estimates; command/PX4 speed and yaw error" if args.telemetry else "disabled"),
                  playback="Original wall-time cadence; no speed-up")
    args.output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report))


if __name__ == "__main__":
    main()

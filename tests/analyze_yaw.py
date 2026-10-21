# SPDX-License-Identifier: AGPL-3.0-only
"""Analyze isolated HITL yaw response from recorded references and telemetry."""
import argparse
import bisect
import json
import math
from pathlib import Path
import statistics


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def yaw(q):
    return math.atan2(2 * (q[0] * q[3] + q[1] * q[2]),
                      1 - 2 * (q[2] * q[2] + q[3] * q[3]))


def angle_error(actual, requested):
    difference = actual - requested
    return math.atan2(math.sin(difference), math.cos(difference))


def percentile(values, fraction):
    return sorted(values)[int((len(values) - 1) * fraction)] if values else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    run = args.run
    data = rows(run / 'board/yaw-response.jsonl')
    references = [r for r in data if r['kind'] == 'reference']
    odometry = [r for r in data if r['kind'] == 'odometry']
    assert references and odometry
    stamps = [r['ros_ns'] for r in references]
    phase_names = list(dict.fromkeys(r['phase'] for r in references))
    motors = [r for r in rows(run / 'mavlink.jsonl')
              if r['system_id'] == 1 and r['component_id'] == 1
              and r['message']['mavpackettype'] == 'HIL_ACTUATOR_CONTROLS']
    frames = rows(run / 'frames.jsonl')
    offset = frames[0]['capture_ns'] - frames[0]['source_sim_ns']
    poses = rows(run / 'poses.jsonl')
    samples = []
    for row in odometry:
        index = bisect.bisect_right(stamps, row['source_ns']) - 1
        if index < 0 or row['source_ns'] - stamps[index] > 150000000:
            continue
        reference = references[index]
        samples.append({'ros_ns': row['source_ns'], 'phase': reference['phase'],
                        'requested_yaw': reference['yaw'], 'actual_yaw': yaw(row['q']),
                        'error_deg': math.degrees(angle_error(yaw(row['q']), reference['yaw'])),
                        'body_z_rate_dps': math.degrees(row['angular_velocity'][2]),
                        'requested_yaw_rate_dps': math.degrees(reference['yawspeed'])})
    positions = json.loads((run / 'board/positions.json').read_text())
    phases = []
    for name in phase_names:
        reference = [r for r in references if r['phase'] == name]
        begin, end = reference[0]['ros_ns'], reference[-1]['ros_ns']
        subset = [r for r in samples if r['phase'] == name]
        if not subset:
            continue
        errors = [abs(r['error_deg']) for r in subset]
        requested = [r['requested_yaw'] for r in subset]
        actual = [r['actual_yaw'] for r in subset]
        # Angles are relative to the first requested heading to handle +/-pi.
        heading = requested[0]
        requested = [math.degrees(angle_error(v, heading)) for v in requested]
        actual = [math.degrees(angle_error(v, heading)) for v in actual]
        tail = [r for r in subset if r['ros_ns'] >= begin + 3e9]
        controls = [r['message']['controls'][:4] for r in motors
                    if begin <= r['received_ros_ns'] <= end]
        truth = [r for r in poses if begin <= r['sample_steady_ns'] + offset <= end]
        truth_errors = []
        for row in truth:
            stamp = row['sample_steady_ns'] + offset
            index = bisect.bisect_right(stamps, stamp) - 1
            if index < 0 or stamp - stamps[index] > 150000000:
                continue
            rotation = row['aircraft']['rotation']
            q = [rotation[k] for k in ('w', 'x', 'y', 'z')]
            truth_errors.append(math.degrees(angle_error(yaw(q), references[index]['yaw'])))
        phase = {'phase': name, 'seconds': (end - begin) / 1e9, 'samples': len(subset),
                 'yaw_error_rms_deg': math.sqrt(statistics.mean(r['error_deg']**2 for r in subset)),
                 'yaw_error_p95_deg': percentile(errors, .95),
                 'yaw_error_max_deg': max(errors),
                 'body_z_rate_abs_p95_dps': percentile([abs(r['body_z_rate_dps']) for r in subset], .95),
                 'requested_yaw_rate_abs_max_dps': max(abs(r['requested_yaw_rate_dps']) for r in subset),
                 'tail_after_3s_error_rms_deg': math.sqrt(statistics.mean(r['error_deg']**2 for r in tail)) if tail else None,
                 'motor_samples': len(controls),
                 'motor_min': min(min(c) for c in controls) if controls else None,
                 'motor_max': max(max(c) for c in controls) if controls else None,
                 'motor_near_max_fraction': sum(any(v >= .99 for v in c) for c in controls) / len(controls) if controls else None,
                 'truth_yaw_error_rms_deg': math.sqrt(statistics.mean(v*v for v in truth_errors)) if truth_errors else None}
        if name not in ('circle_tangent_yaw',):
            phase['actual_yaw_peak_to_peak_deg'] = max(actual) - min(actual)
            phase['requested_yaw_peak_to_peak_deg'] = max(requested) - min(requested)
        if name.startswith('yaw_step_'):
            prior = references[max(0, bisect.bisect_left(stamps, begin) - 1)]['yaw']
            change = angle_error(reference[0]['yaw'], prior)
            if abs(change) > .001:
                direction = math.copysign(1, change)
                overshoot = max(0.0, max(direction * r['error_deg'] for r in subset))
                phase['step_degrees'] = math.degrees(change)
                phase['overshoot_degrees'] = overshoot
                phase['overshoot_percent'] = 100 * overshoot / abs(math.degrees(change))
                last_outside = max((r['ros_ns'] for r in subset if abs(r['error_deg']) > 2), default=begin)
                phase['settling_time_within_2deg_s'] = ((last_outside - begin) / 1e9
                    if end - last_outside >= 1e9 else None)
        position_errors = []
        speeds = []
        for row in positions:
            stamp = row['px4_us'] * 1000
            if not begin <= stamp <= end:
                continue
            index = bisect.bisect_right(stamps, stamp) - 1
            if index < 0 or stamp - stamps[index] > 150000000:
                continue
            expected = references[index]['expected_position_ned']
            position_errors.append(math.dist(expected[:2], row['ned'][:2]))
            speeds.append(math.hypot(*row['velocity_ned'][:2]))
        phase['horizontal_path_error_p95_m'] = percentile(position_errors, .95)
        phase['horizontal_speed_max_mps'] = max(speeds) if speeds else None
        phases.append(phase)
    summary = {'diagnostic_completed': json.loads((run / 'board/result.json').read_text()).get('yaw_diagnostic_completed', False),
               'vision_controls_used': False, 'phases': phases,
               'motor_interarrival_p95_ms': percentile([(b['received_steady_ns']-a['received_steady_ns'])/1e6 for a,b in zip(motors,motors[1:])], .95),
               'motor_interarrival_max_ms': max((b['received_steady_ns']-a['received_steady_ns'])/1e6 for a,b in zip(motors,motors[1:]))}
    (run / 'yaw-analysis.json').write_text(json.dumps(summary, indent=2) + '\n')
    (run / 'yaw-samples.json').write_text(json.dumps(samples) + '\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()

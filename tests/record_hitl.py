# SPDX-License-Identifier: AGPL-3.0-only
"""Read-only M7 HITL telemetry recorder; never requests streams or commands."""
import argparse
from collections import Counter
import json
import math
import os
from pathlib import Path
import time


def finite(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: finite(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite(v) for v in value]
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('board')
    parser.add_argument('output', type=Path)
    parser.add_argument('--seconds', type=float, default=600)
    parser.add_argument('--stop-file', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('RAVEN_M7_HITL') != '1' or not 0 < args.seconds <= 1800:
        raise RuntimeError('Explicit HITL opt-in and bounded duration required')
    from pymavlink import mavutil
    link = mavutil.mavlink_connection('tcp:' + args.board + ':5760',
                                      source_system=247, source_component=191)
    selected = {'HEARTBEAT', 'HIL_SENSOR', 'HIL_GPS', 'HIL_ACTUATOR_CONTROLS',
                'ATTITUDE', 'ATTITUDE_QUATERNION', 'ATTITUDE_TARGET',
                'LOCAL_POSITION_NED', 'ACTUATOR_CONTROL_TARGET', 'STATUSTEXT'}
    counts = Counter()
    begin = time.monotonic()
    try:
        with args.output.open('x') as stream:
            while time.monotonic() - begin < args.seconds and not args.stop_file.exists():
                message = link.recv_match(blocking=True, timeout=.2)
                if message is None or message.get_type() not in selected:
                    continue
                # Include simulator sensors plus M7 outputs; retain source IDs
                # rather than assuming they share a clock or component.
                kind = message.get_type()
                counts[kind] += 1
                row = {'received_ros_ns': time.time_ns(),
                       'received_steady_ns': time.monotonic_ns(),
                       'system_id': message.get_srcSystem(),
                       'component_id': message.get_srcComponent(),
                       'message': message.to_dict()}
                stream.write(json.dumps(finite(row), allow_nan=False) + '\n')
    finally:
        link.close()
        summary = {'seconds': time.monotonic() - begin, 'counts': dict(counts),
                   'commands_sent': 0, 'stream_rates_changed': False}
        args.output.with_suffix('.summary.json').write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps(summary), flush=True)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-only
"""Snapshot and restore only the aiding parameters for an explicit M7 VIO test."""
import argparse
import json
import os
from pathlib import Path

from pymavlink import mavutil
from hitl_parameters import gate, read, write


DESIRED = {'EKF2_EV_CTRL': 1, 'EKF2_HGT_REF': 0, 'EKF2_GPS_CTRL': 0}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('apply', 'restore'))
    parser.add_argument('board')
    parser.add_argument('snapshot', type=Path)
    args = parser.parse_args()
    if os.environ.get('RAVEN_M7_HITL') != '1':
        raise RuntimeError('Explicit M7 HITL opt-in required')
    link = mavutil.mavlink_connection('tcp:'+args.board+':5760', source_system=245,
                                      source_component=191)
    try:
        gate(link)
        if args.mode == 'restore':
            snapshot = json.loads(args.snapshot.read_text())
            if snapshot.get('schema') != 1 or set(snapshot['parameters']) != set(DESIRED):
                raise RuntimeError('Unexpected external-vision parameter snapshot')
            for name in reversed(DESIRED):
                item = snapshot['parameters'][name]
                write(link, name, item['value'], item['type'])
        else:
            snapshot = {'schema':1, 'parameters':{}}
            for name in DESIRED:
                original, kind = read(link, name)
                snapshot['parameters'][name] = {'value':original,'type':kind}
            # Preserve originals before the first write, including partial failure.
            with args.snapshot.open('x') as stream:
                json.dump(snapshot, stream, indent=2)
                stream.write('\n')
            for name, desired in DESIRED.items():
                write(link, name, desired, snapshot['parameters'][name]['type'])
        print('Restart EKF2 while disarmed to apply the height-reference change.',flush=True)
    finally:
        link.close()


if __name__ == '__main__':
    main()

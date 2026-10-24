#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-only
"""Measure a short-run Windows AirSim QPC to Linux clock epoch translation."""
import argparse
import ipaddress
import json
import os
from pathlib import Path
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('board')
    parser.add_argument('--wsl', default='Ubuntu-22.04')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.name != 'nt':
        raise RuntimeError('Run on the Windows simulator host: WSL has a different monotonic clock epoch')
    ipaddress.IPv4Address(args.board)
    code = 'import sys,time\nfor line in sys.stdin:\n print(time.time_ns(),flush=True)'
    process = subprocess.Popen(['wsl','-d',args.wsl,'--','ssh','-T','-o','BatchMode=yes',
                                '-o','ConnectTimeout=10','-o','ServerAliveInterval=2',
                                '-o','ServerAliveCountMax=3','root@'+args.board,
                                "python3 -u -c '"+code+"'"], stdin=subprocess.PIPE,
                               stdout=subprocess.PIPE, text=True, bufsize=1)
    samples = []
    try:
        for _ in range(60):
            first = time.perf_counter_ns()
            process.stdin.write('clock\n')
            process.stdin.flush()
            board_ns = int(process.stdout.readline())
            last = time.perf_counter_ns()
            samples.append({'round_trip_ns':last-first,
                            'offset_ns':board_ns-(first+last)//2})
            time.sleep(.02)
    finally:
        process.stdin.close()
        process.wait(timeout=10)
    best = min(samples, key=lambda sample:sample['round_trip_ns'])
    uncertainty = best['round_trip_ns']//2
    if uncertainty > 2000000:
        raise RuntimeError('Clock measurement uncertainty exceeds 2 ms; no valid epoch offset written')
    result = {'method':'persistent SSH midpoint; minimum observed round trip',
              'timestamp_offset_ns':best['offset_ns'],
              'round_trip_uncertainty_ns':uncertainty, 'samples':samples}
    # This does not discipline either clock. Re-measure before short bench runs;
    # the hardware adapter independently rejects stale/future measurements.
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({key:value for key,value in result.items() if key != 'samples'}))


if __name__ == '__main__':
    main()

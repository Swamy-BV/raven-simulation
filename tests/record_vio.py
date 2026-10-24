#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-only
"""Passively record VIO, IMU, images and evaluation-only simulator reference poses."""
import argparse
from pathlib import Path
import time

import rclpy
from observe_vio import VioObserver


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=120)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 900:
        raise ValueError('Recording must be 1..900 seconds')
    rclpy.init()
    observer = VioObserver(args.config, args.output)
    try:
        end = time.monotonic()+args.seconds
        while time.monotonic() < end:
            rclpy.spin_once(observer, timeout_sec=.05)
    finally:
        print(observer.finish(), flush=True)
        observer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

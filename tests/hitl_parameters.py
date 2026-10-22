#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-only
"""Apply generated MPC parameters to a disarmed HIL bench, then restore them."""
import argparse
import json
import math
import os
from pathlib import Path
import re
import struct
import time

from pymavlink import mavutil


def value(message):
    if message.param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
        return struct.unpack("<i", struct.pack("<f", message.param_value))[0]
    if message.param_type == mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
        return float(message.param_value)
    raise RuntimeError("Unsupported parameter type")


def parameter(link, name, expected=None):
    end = time.monotonic() + 5
    while time.monotonic() < end:
        message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=.2)
        if (message and message.get_srcSystem() == 1 and message.get_srcComponent() == 1
                and message.param_id == name):
            actual = value(message)
            if expected is None or math.isclose(actual, expected, rel_tol=1e-5, abs_tol=1e-5):
                return actual, message.param_type
    raise RuntimeError(f"Parameter {name} acknowledgement missing or mismatched")


def read(link, name):
    link.mav.param_request_read_send(1, 1, name.encode(), -1)
    return parameter(link, name)


def write(link, name, desired, kind):
    encoded = (struct.unpack("<f", struct.pack("<i", int(desired)))[0]
               if kind == mavutil.mavlink.MAV_PARAM_TYPE_INT32 else float(desired))
    link.mav.param_set_send(1, 1, name.encode(), encoded, kind)
    parameter(link, name, desired)
    print(f"HIL parameter {name}={desired}", flush=True)


def gate(link):
    actual, _ = read(link, "SYS_HITL")
    if actual != 1:
        raise RuntimeError("Expected SYS_HITL=1")
    end = time.monotonic() + 5
    while time.monotonic() < end:
        heartbeat = link.recv_match(type="HEARTBEAT", blocking=True, timeout=.2)
        if not heartbeat or heartbeat.get_srcSystem() != 1 or heartbeat.get_srcComponent() != 1:
            continue
        if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            raise RuntimeError("M7 must be disarmed for HIL parameter changes")
        if not heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_HIL_ENABLED:
            raise RuntimeError("M7 heartbeat is not in HIL mode")
        return
    raise RuntimeError("Fresh M7 heartbeat missing")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("apply", "restore"))
    parser.add_argument("board")
    parser.add_argument("snapshot", type=Path)
    parser.add_argument("--config", type=Path)
    args = parser.parse_args()
    if os.environ.get("RAVEN_M7_HITL") != "1":
        raise RuntimeError("Explicit M7 HITL opt-in required")
    link = mavutil.mavlink_connection(f"tcp:{args.board}:5760", source_system=245, source_component=191)
    try:
        gate(link)
        if args.mode == "restore":
            snapshot = json.loads(args.snapshot.read_text())
            if snapshot.get("schema") != 1:
                raise RuntimeError("Unsupported parameter snapshot")
            for name, entry in snapshot["parameters"].items():
                if not re.fullmatch(r"MPC_[A-Z0-9_]+", name):
                    raise RuntimeError("Snapshot contains a non-MPC parameter")
                write(link, name, entry["value"], entry["type"])
            return
        if args.snapshot.exists():
            raise RuntimeError("Refusing to overwrite original parameter snapshot")
        desired = {}
        for line in (args.config / "px4.env").read_text().splitlines():
            name, text = line.removeprefix("PX4_PARAM_").split("=", 1)
            number = float(text)
            if not re.fullmatch(r"MPC_[A-Z0-9_]+", name) or not math.isfinite(number):
                raise RuntimeError("Invalid generated HIL parameter")
            desired[name] = number
        snapshot = {"schema": 1, "parameters": {}}
        for name in desired:
            original, kind = read(link, name)
            snapshot["parameters"][name] = {"value": original, "type": kind}
        temporary = args.snapshot.with_suffix(".tmp")
        temporary.write_text(json.dumps(snapshot, indent=2) + "\n")
        temporary.replace(args.snapshot)
        # Persist the originals before the first write, including partial failure.
        for name, number in desired.items():
            write(link, name, number, snapshot["parameters"][name]["type"])
    finally:
        link.close()


if __name__ == "__main__":
    main()

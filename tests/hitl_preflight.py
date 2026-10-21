# SPDX-License-Identifier: AGPL-3.0-only
"""Disarmed transport gate for the separately authorized M7 HITL test."""

import argparse
import json
import os
import struct
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("board")
    parser.add_argument("--serial-hitl", action="store_true",
                        help="Simulator sensors arrive over the dedicated M7 UART")
    args = parser.parse_args()
    if os.environ.get("RAVEN_M7_HITL") != "1":
        raise RuntimeError("Explicit M7 HITL opt-in required")
    from pymavlink import mavutil

    link = mavutil.mavlink_connection("tcp:" + args.board + ":5760",
                                      source_system=246, source_component=191)
    counts = {"HIL_SENSOR": 0, "HIL_GPS": 0, "HIL_ACTUATOR_CONTROLS": 0}
    imu_times = []
    heartbeat = None
    hitl_parameter = None
    try:
        link.mav.param_request_read_send(1, 1, b"SYS_HITL", -1)
        next_request = time.monotonic() + 1
        end = time.monotonic() + 8
        while time.monotonic() < end:
            if time.monotonic() >= next_request and hitl_parameter is None:
                link.mav.param_request_read_send(1, 1, b"SYS_HITL", -1)
                next_request = time.monotonic() + 1
            message = link.recv_match(blocking=True, timeout=.2)
            if message is None:
                continue
            kind = message.get_type()
            if kind in ("HIL_SENSOR", "HIL_GPS"):
                counts[kind] += 1
            if kind == "HIL_SENSOR" and (not imu_times or message.time_usec > imu_times[-1]):
                imu_times.append(message.time_usec)
            if message.get_srcSystem() != 1 or message.get_srcComponent() != 1:
                continue
            if kind == "HEARTBEAT":
                heartbeat = message
                if message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    raise RuntimeError("M7 must be disarmed before HITL acceptance")
                if not message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_HIL_ENABLED:
                    raise RuntimeError("M7 heartbeat does not identify HIL mode")
            if kind == "PARAM_VALUE" and message.param_id == "SYS_HITL":
                if message.param_type != mavutil.mavlink.MAV_PARAM_TYPE_INT32:
                    raise RuntimeError("Unexpected SYS_HITL parameter type")
                hitl_parameter = struct.unpack("<i", struct.pack("<f", message.param_value))[0]
            if kind == "HIL_ACTUATOR_CONTROLS":
                if not message.mode & mavutil.mavlink.MAV_MODE_FLAG_HIL_ENABLED:
                    raise RuntimeError("Actuator output is not marked HIL")
                counts[kind] += 1
        if heartbeat is None or hitl_parameter != 1:
            raise RuntimeError(f"Expected actual M7 and SYS_HITL=1, got {hitl_parameter}")
        if counts["HIL_ACTUATOR_CONTROLS"] < 10:
            raise RuntimeError("Missing M7 HIL actuator output: " + str(counts))
        if not args.serial_hitl and (counts["HIL_SENSOR"] < 100 or counts["HIL_GPS"] < 10):
            raise RuntimeError("Incomplete AirSim/M7 HIL transport: " + str(counts))
        imu_rate_hz = ((len(imu_times) - 1) * 1e6 / (imu_times[-1] - imu_times[0])
                       if len(imu_times) > 1 else 0)
        # These HITL profiles request 250 Hz. A packet-count-only check also
        # accepts the Windows hidden-window failure at approximately 64 Hz.
        if not args.serial_hitl and imu_rate_hz < 200:
            raise RuntimeError(f"HITL IMU cadence too slow: {imu_rate_hz:.1f} Hz; expected about 250 Hz")
        def console(command):
            payload = (command + "\n").encode()
            link.mav.serial_control_send(
                10, 6, 0, 0, len(payload), list(payload) + [0] * (70 - len(payload)))
            output = ""
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                message = link.recv_match(type="SERIAL_CONTROL", blocking=True, timeout=.2)
                if message and message.get_srcSystem() == 1 and message.get_srcComponent() == 1:
                    output += bytes(message.data[:message.count]).decode(errors="replace")
                    if output.rstrip().endswith("nsh> \x1b[K"):
                        return output
            raise RuntimeError("No complete M7 console response for " + command)

        # Wait for the shell greeting before issuing the first actual command.
        console("")
        version = console("ver all")
        required_variant = "Build variant: hitl-serial" if args.serial_hitl else "Build variant: hitl"
        if "HW arch: RAVEN_FRDM" not in version or required_variant not in version:
            raise RuntimeError("Dedicated simulated-output M7 firmware required")
        simulated_outputs = console("pwm_out_sim status")
        if "Param prefix: HIL_ACT" not in simulated_outputs:
            raise RuntimeError("HIL actuator simulator is not running")
        for index in range(4):
            if "func: " + str(101 + index) not in simulated_outputs:
                raise RuntimeError("Missing simulated motor function")
        physical_outputs = console("pwm_out status")
        if "command not found" not in physical_outputs:
            raise RuntimeError("Physical PWM backend must be excluded from this firmware")
        if args.serial_hitl:
            import re
            for topic in ("sensor_combined", "sensor_gps"):
                samples = []
                for _ in range(2):
                    report = console("listener " + topic + " 1")
                    match = re.search(r"timestamp: (\d+) \(([0-9.]+) seconds ago\)", report)
                    if not match or float(match.group(2)) >= 2:
                        raise RuntimeError("Missing fresh serial HIL sensor: " + report)
                    samples.append(int(match.group(1)))
                    time.sleep(.2)
                if samples[1] <= samples[0]:
                    raise RuntimeError("Serial HIL sensor stopped: " + topic)
        print(json.dumps({"system_id": 1, "SYS_HITL": hitl_parameter,
                          "disarmed": True, "simulated_outputs_only": True,
                          "network_imu_rate_hz": imu_rate_hz if not args.serial_hitl else None,
                          "sensor_transport": "serial" if args.serial_hitl else "network", "message_counts_8s": counts}), flush=True)
    finally:
        link.close()


if __name__ == "__main__":
    main()

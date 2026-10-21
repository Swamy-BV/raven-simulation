#!/bin/bash
# SPDX-License-Identifier: AGPL-3.0-only
# The explicit test argument authorizes arming only isolated SITL system ID 2.
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
board=${1:?Provide board IPv4 address}
test=${2:?Choose hover, follow or traffic}
case "$test" in hover|follow|traffic) ;; *) echo 'Choose hover, follow or traffic' >&2; exit 2;; esac
[[ "$board" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]] || { echo 'Expected IPv4 address' >&2; exit 2; }
args=(-o BatchMode=yes -o ConnectTimeout=10 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 -o IPQoS=none)
ssh "${args[@]}" root@"$board" 'mkdir -p /run/raven-simulation/tests'
scp "${args[@]}" "$root/tests/flight_session.py" "$root/tests/test_flight.py" root@"$board":/run/raven-simulation/tests/
ssh "${args[@]}" root@"$board" /bin/sh -s -- "$test" <<'REMOTE'
set -e
cd /run/raven-simulation/tests
export ROS_LOG_DIR=/tmp/raven-simulation-log
. /opt/ros/jazzy/setup.sh
export RAVEN_SIM_FLIGHT=1
python3 -m unittest -v "test_flight.FlightTests.test_$1"
REMOTE

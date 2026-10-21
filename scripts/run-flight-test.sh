#!/bin/bash
# SPDX-License-Identifier: AGPL-3.0-only
# The explicit test argument authorizes arming only isolated SITL system ID 2.
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
board=${1:?Provide board IPv4 address}
test=${2:?Choose hover, follow, turn, drive, traffic or guidance}
case "$test" in hover|follow|turn|drive|traffic|guidance) ;; *) echo 'Choose hover, follow, turn, drive, traffic or guidance' >&2; exit 2;; esac
[[ "$board" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]] || { echo 'Expected IPv4 address' >&2; exit 2; }
snapshots=${RAVEN_SIM_SNAPSHOTS:-0}
case "$snapshots" in 0|1) ;; *) echo "RAVEN_SIM_SNAPSHOTS must be 0 or 1" >&2; exit 2;; esac
runtime=${RAVEN_SIM_ROS_PREFIX:-}
if [ -n "$runtime" ]; then
  [[ "$runtime" =~ ^/[-/A-Za-z0-9_.]+$ ]] || { echo 'Expected target runtime prefix' >&2; exit 2; }
fi
args=(-o BatchMode=yes -o ConnectTimeout=10 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 -o IPQoS=none)
if [ -n "${RAVEN_SIM_SSH_SOCKET:-}" ]; then
  [[ "$RAVEN_SIM_SSH_SOCKET" =~ ^/[-/A-Za-z0-9_.%]+$ ]] || { echo 'Expected SSH socket path' >&2; exit 2; }
  args+=(-o ControlPath="$RAVEN_SIM_SSH_SOCKET" -o ControlMaster=auto -o ControlPersist=300)
fi
ssh "${args[@]}" root@"$board" 'mkdir -p /run/raven-simulation/tests'
tar -C "$root/tests" -cf - flight_session.py test_flight.py test_guidance.py |
  ssh "${args[@]}" root@"$board" 'tar -xf - -C /run/raven-simulation/tests'
ssh "${args[@]}" root@"$board" /bin/sh -s -- "$test" "$runtime" "$snapshots" <<'REMOTE'
set -e
cd /run/raven-simulation/tests
export ROS_LOG_DIR=/tmp/raven-simulation-log
set -a
. /etc/ros/ros2.env
set +a
. /opt/ros/jazzy/setup.sh
if [ -n "$2" ]; then
  version=$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
  export AMENT_PREFIX_PATH="$2:${AMENT_PREFIX_PATH:-/opt/ros/jazzy}"
  export LD_LIBRARY_PATH="$2/lib:${LD_LIBRARY_PATH:-/opt/ros/jazzy/lib}"
  export PYTHONPATH="$2/lib/python$version/site-packages${PYTHONPATH:+:$PYTHONPATH}"
fi
export RAVEN_SIM_FLIGHT=1 RAVEN_SIM_SNAPSHOTS="$3"
case "$1" in
  guidance) target=test_guidance.GuidanceTests.test_go_to_and_cancel ;;
  *) target="test_flight.FlightTests.test_$1" ;;
esac
python3 -u -m unittest -v "$target"
REMOTE

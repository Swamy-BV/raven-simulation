#!/bin/bash
# SPDX-License-Identifier: AGPL-3.0-only
set -euo pipefail
[[ "${RAVEN_M7_HITL:-}" = 1 ]] || { echo 'Set RAVEN_M7_HITL=1 for the isolated M7 HITL bench' >&2; exit 2; }
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
board=${1:?Provide board IPv4 address}
case_name=${2:-hover}
case "$case_name" in hover|follow|yaw) ;; *) echo 'Choose hover, follow or yaw' >&2; exit 2;; esac
run_dir=${RAVEN_SIM_RUN_DIR:-}
if [ -n "$run_dir" ] && { [ "$case_name" != follow ] || [ ! -f "$run_dir/config/bindings.json" ]; }; then
  echo 'RAVEN_SIM_RUN_DIR must name the active follow bridge run directory' >&2
  exit 2
fi
mirror_output() {
  while IFS= read -r line; do
    printf '%s\n' "$line"
    if [ -n "$run_dir" ] && [[ "$line" == FOLLOWING_READY\ * ]]; then
      # Signal the environment fixture only after guidance accepts authority.
      touch -- "$run_dir/start-target"
    fi
  done
}
runtime=${RAVEN_SIM_ROS_PREFIX:-}
[[ -z "$runtime" || "$runtime" =~ ^/[-/A-Za-z0-9_.]+$ ]] || exit 2
[[ "$board" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]] || exit 2
args=(-o BatchMode=yes -o ConnectTimeout=10 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 -o IPQoS=none)
if [ -n "${RAVEN_SIM_SSH_SOCKET:-}" ]; then
  args+=(-o ControlPath="$RAVEN_SIM_SSH_SOCKET")
fi
height=4.5
cruise=3.5
follow_seconds=61
reticle_x=320
reticle_y=135
require_reid=0
require_ground=0
if [ -n "$run_dir" ]; then
  read -r height cruise follow_seconds reticle_x reticle_y < <(python3 - "$run_dir/config/bindings.json" <<'CONFIG'
import json, sys
settings = json.load(open(sys.argv[1])).get("acceptance", {})
height = float(settings.get("height_m", 4.5))
cruise = float(settings.get("minimum_target_speed_mps", 3.5))
assert 2 <= height <= 12 and 0 < cruise < 30
seconds = float(settings.get("following_seconds", 61))
assert 61 <= seconds <= 180
reticle = settings.get("acquisition_reticle", [320, 135])
assert len(reticle) == 2 and 0 <= reticle[0] <= 640 and 0 <= reticle[1] <= 360
print(height, cruise, seconds, *reticle)
CONFIG
)
fi
if [ -n "$run_dir" ] && [ -r "$run_dir/config/tracking.yaml" ]; then
  read -r require_reid require_ground < <(python3 - "$run_dir/config/tracking.yaml" <<'TRACKING'
import json, sys
parameters = json.load(open(sys.argv[1]))["/raven/vehicle_1/perception/target_tracker"]["ros__parameters"]
print(int(bool(parameters.get("vehicle_reid_model"))), int(bool(parameters.get("ground_association"))))
TRACKING
)
fi
python3 "$root/tests/hitl_preflight.py" "$board"
parameter_snapshot=
restore_parameters() {
  result=$?
  if [ -n "$parameter_snapshot" ] && [ -f "$parameter_snapshot" ]; then
    python3 "$root/tests/hitl_parameters.py" restore "$board" "$parameter_snapshot" || result=1
  fi
  exit "$result"
}
trap restore_parameters EXIT
if [ -n "$run_dir" ]; then
  parameter_snapshot="$run_dir/hitl-parameters-before.json"
  python3 "$root/tests/hitl_parameters.py" apply "$board" "$parameter_snapshot" --config "$run_dir/config"
fi
ssh "${args[@]}" root@"$board" 'mkdir -p /run/raven-hitl/tests; cat > /run/raven-hitl/tests/test_hitl.py' < "$root/tests/test_hitl.py"
ssh "${args[@]}" root@"$board" 'cat > /run/raven-hitl/tests/yaw_profile.py' < "$root/tests/yaw_profile.py"
ssh "${args[@]}" root@"$board" /bin/sh -s -- "$case_name" "$runtime" "$height" "$cruise" "$follow_seconds" "$reticle_x" "$reticle_y" "$require_reid" "$require_ground" <<'REMOTE' | mirror_output
set -e
cd /run/raven-hitl/tests
export ROS_LOG_DIR=/tmp/raven-hitl-log
set -a
. /etc/ros/ros2.env
set +a
. /opt/ros/jazzy/setup.sh
if [ -n "$2" ]; then
  export AMENT_PREFIX_PATH="$2:${AMENT_PREFIX_PATH:-/opt/ros/jazzy}"
  export LD_LIBRARY_PATH="$2/lib:${LD_LIBRARY_PATH:-/opt/ros/jazzy/lib}"
  version=$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
  export PYTHONPATH="$2/lib/python$version/site-packages${PYTHONPATH:+:$PYTHONPATH}"
fi
export RAVEN_M7_HITL=1 RAVEN_HITL_HEIGHT_M="$3" RAVEN_HITL_CRUISE_MPS="$4" RAVEN_HITL_FOLLOW_SECONDS="$5"
export RAVEN_HITL_RETICLE_X="$6" RAVEN_HITL_RETICLE_Y="$7"
export RAVEN_HITL_REQUIRE_REID="$8" RAVEN_HITL_REQUIRE_GROUND="$9"
python3 -u -m unittest -v "test_hitl.HitlTests.test_${1}_and_land"
REMOTE

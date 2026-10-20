#!/bin/bash
# SPDX-License-Identifier: AGPL-3.0-only
# Start after the bridge reports a loaded scene; keep this terminal running.
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
# shellcheck source=../versions.env
source "$root/versions.env"
px4=$(realpath -- "${1:?Provide the firmware-matched PX4 checkout}")
host=${2:?Provide the Windows WSL gateway IPv4 address}
board=${3:?Provide the board IPv4 address}
work=$(realpath -m -- "${4:-$root/run/px4}")
for address in "$host" "$board"; do
  [[ "$address" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]] || { echo 'Expected IPv4 addresses' >&2; exit 2; }
done
if [ "$(git -C "$px4" rev-parse HEAD)" != "$PX4_COMMIT" ]; then
  echo 'PX4 must match the firmware/px4_msgs revision in versions.env.' >&2
  exit 1
fi
[[ -x "$px4/build/px4_sitl_default/bin/px4" ]] || { echo 'Build px4_sitl_default first.' >&2; exit 1; }
IFS=. read -r first second third fourth <<< "$board"
for octet in "$first" "$second" "$third" "$fourth"; do
  ((10#$octet <= 255)) || { echo 'Invalid IPv4 octet' >&2; exit 2; }
done
agent_ip=$((10#$first * 16777216 + 10#$second * 65536 + 10#$third * 256 + 10#$fourth))
mkdir -p "$work"
exec 9>"$work/run.lock"
flock -n 9 || { echo 'This PX4 working directory is already in use.' >&2; exit 1; }
export PX4_SYS_AUTOSTART=10016 PX4_SIMULATOR=none PX4_SIM_HOST_ADDR=127.0.0.1
export PX4_UXRCE_DDS_NS=simulation PX4_UXRCE_DDS_PORT=8889
export PX4_PARAM_UXRCE_DDS_AG_IP="$agent_ip"
export PX4_PARAM_UXRCE_DDS_PTCFG=1 PX4_PARAM_UXRCE_DDS_SYNCT=0 PX4_PARAM_UXRCE_DDS_DOM_ID=0
# Instance 1 keeps system ID 2; relay to the scoped Windows simulator port.
socat TCP-LISTEN:4561,bind=127.0.0.1,reuseaddr,fork TCP:"$host":4560 &
relay=$!
trap 'kill "$relay" 2>/dev/null || true; wait "$relay" 2>/dev/null || true' EXIT
"$px4/build/px4_sitl_default/bin/px4" -d "$px4/build/px4_sitl_default/etc" -i 1 -w "$work"

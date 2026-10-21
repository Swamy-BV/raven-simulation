#!/bin/bash
# SPDX-License-Identifier: AGPL-3.0-only
# Configure only the temporary simulator route; never arm or reflash M7.
set -euo pipefail
mode=${1:?Choose start or stop}
board=${2:?Provide board IPv4 address}
simulator=${3:-}
case "$mode" in start|stop) ;; *) exit 2;; esac
for address in "$board" ${simulator:+"$simulator"}; do
  [[ "$address" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]] || { echo 'Expected IPv4 address' >&2; exit 2; }
done
if [ "$mode" = start ] && [ -z "$simulator" ]; then
  echo 'Provide the Windows simulator LAN address' >&2
  exit 2
fi
args=(-o BatchMode=yes -o ConnectTimeout=10 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 -o IPQoS=none)
if [ -n "${RAVEN_SIM_SSH_SOCKET:-}" ]; then
  args+=(-o ControlPath="$RAVEN_SIM_SSH_SOCKET")
fi
runtime=0
if [ "$mode" = start ] && [ -n "${RAVEN_HITL_ROUTER_BINARY:-}" ]; then
  test -r "$RAVEN_HITL_ROUTER_BINARY"
  ssh "${args[@]}" root@"$board" 'mkdir -p /run/raven-hitl; cat > /run/raven-hitl/mavlink-routerd.new && chmod 0755 /run/raven-hitl/mavlink-routerd.new && mv /run/raven-hitl/mavlink-routerd.new /run/raven-hitl/mavlink-routerd' < "$RAVEN_HITL_ROUTER_BINARY"
  runtime=1
fi
ssh "${args[@]}" root@"$board" /bin/sh -s -- "$mode" "$simulator" "$runtime" <<'REMOTE'
set -e
if [ "$1" = start ]; then
  mkdir -p /run/raven-hitl /run/systemd/system/mavlink-router.service.d
  cat /etc/mavlink-router/main.conf > /run/raven-hitl/main.conf
  cat >> /run/raven-hitl/main.conf <<CONF

[TcpEndpoint airsim-hitl]
Address = $2
Port = 4560
RetryTimeout = 5
# AirSim must observe the flight controller, not camera-component heartbeats.
AllowSrcCompOut = 1
AllowSrcSysOut = 1

[UdpEndpoint airsim-control]
Mode = Server
Address = 0.0.0.0
Port = 14581
AllowSrcCompOut = 1
AllowSrcSysOut = 1
CONF
  cat > /run/systemd/system/mavlink-router.service.d/90-hitl.conf <<'CONF'
[Service]
ExecStart=
ExecStart=/usr/bin/mavlink-routerd -c /run/raven-hitl/main.conf
CONF
  if [ "$3" = 1 ]; then
    sed -i 's|/usr/bin/mavlink-routerd|/run/raven-hitl/mavlink-routerd|' /run/systemd/system/mavlink-router.service.d/90-hitl.conf
  fi
else
  rm -f /run/systemd/system/mavlink-router.service.d/90-hitl.conf
fi
systemctl daemon-reload
systemctl restart mavlink-router
systemctl is-active mavlink-router
REMOTE

#!/bin/bash
# SPDX-License-Identifier: AGPL-3.0-only
# Temporary /run overrides only. Land the simulated aircraft before stop.
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
mode=${1:?Choose start or stop}
trace=${RAVEN_SIM_TRACE:-0}
case "$trace" in 0|1) ;; *) echo "RAVEN_SIM_TRACE must be 0 or 1" >&2; exit 2;; esac
board=${2:?Provide board IPv4 address}
case "$mode" in start|stop) ;; *) echo 'Choose start or stop' >&2; exit 2;; esac
[[ "$board" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]] || { echo 'Expected IPv4 address' >&2; exit 2; }
ssh_args=(-o BatchMode=yes -o ConnectTimeout=10 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 -o IPQoS=none)
if [ "$mode" = start ]; then
  ssh "${ssh_args[@]}" root@"$board" 'mkdir -p /run/raven-simulation; cat > /run/raven-simulation/camera.yaml' < "$root/config/camera.yaml"
fi
ssh "${ssh_args[@]}" root@"$board" /bin/sh -s -- "$mode" "$trace" <<'REMOTE'
set -e
mode=$1
trace=$2
systemctl stop target-follower
for service in media-capture vision-engine target-tracker target-follower; do
  mkdir -p /run/systemd/system/$service.service.d
  override=/run/systemd/system/$service.service.d/90-airsim.conf
  if [ "$mode" = stop ]; then
    rm -f "$override"
    continue
  fi
  package=$(printf '%s' "$service" | tr '-' '_')
  cat > "$override" <<CONF
[Service]
ExecStart=
ExecStart=/opt/ros/jazzy/lib/$package/$service --ros-args --params-file /etc/$service.yaml -p use_sim_time:=true
CONF
  if [ "$trace" = 1 ]; then
    case "$service" in
      media-capture) logger=raven.vehicle_1.cameras.front.media_capture ;;
      target-follower) logger=raven.vehicle_1.autonomy.target_follower ;;
      *) logger=raven.vehicle_1.perception.$package ;;
    esac
    sed -i "/^ExecStart=\/opt/s|\$| --log-level $logger:=debug|" "$override"
  fi
  if [ "$service" = media-capture ]; then
    sed -i '/^ExecStart=\/opt/s|$| -p source:=rtp -p rtp_capture_timestamps:=true -p input_uri:=udp://0.0.0.0:5602 -p source_width:=640 -p source_height:=360 -p photo_width:=640 -p photo_height:=360 -p ros_fps:=30 -p optical_frame:=sim_camera_optical_frame -p camera_info_url:=file:///run/raven-simulation/camera.yaml|' "$override"
  elif [ "$service" = target-follower ]; then
    sed -i '/^ExecStart=\/opt/s|$| -p ground_z:=0.2 -p camera_pitch:=0.6108652381980153 -p camera_forward:=0.5 -p camera_right:=0.0 -p camera_down:=0.2|' "$override"
  fi
done
if [ "$mode" = start ]; then
  cat > /run/systemd/system/px4-simulation-agent.service <<'UNIT'
[Unit]
Description=Temporary DDS agent for isolated PX4 SITL
After=network.target
[Service]
Type=simple
User=px4-dds
Group=px4-dds
Environment=LD_LIBRARY_PATH=/opt/ros/jazzy/lib
Environment=FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ExecStart=/opt/ros/jazzy/bin/MicroXRCEAgent udp4 -p 8889 -v 3
Restart=on-failure
RestartSec=5
NoNewPrivileges=yes
PrivateTmp=yes
ProtectSystem=strict
ProtectHome=yes
UNIT
else
  systemctl stop px4-simulation-agent.service
  rm -f /run/systemd/system/px4-simulation-agent.service
fi
systemctl daemon-reload
systemctl restart media-capture vision-engine target-tracker
if [ "$mode" = start ]; then
  systemctl start px4-simulation-agent.service target-follower
fi
systemctl is-active media-capture vision-engine target-tracker
REMOTE

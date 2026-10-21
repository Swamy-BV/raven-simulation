#!/bin/bash
# SPDX-License-Identifier: AGPL-3.0-only
# Temporary /run overrides only. Land the simulated aircraft before stop.
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
mode=${1:?Choose start or stop}
trace=${RAVEN_SIM_TRACE:-0}
case "$trace" in 0|1) ;; *) echo "RAVEN_SIM_TRACE must be 0 or 1" >&2; exit 2;; esac
board=${2:?Provide board IPv4 address}
scenario_config=${3:-}
follower_binary=${RAVEN_SIM_FOLLOWER_BINARY:-}
case "$mode" in start|stop) ;; *) echo 'Choose start or stop' >&2; exit 2;; esac
[[ "$board" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]] || { echo 'Expected IPv4 address' >&2; exit 2; }
ssh_args=(-o BatchMode=yes -o ConnectTimeout=10 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 -o IPQoS=none)
profile=0
binary=0
if [ "$mode" = start ]; then
  camera="$root/config/camera.yaml"
  if [ -n "$scenario_config" ]; then
    test -r "$scenario_config/camera.yaml"
    test -r "$scenario_config/following.yaml"
    camera="$scenario_config/camera.yaml"
    profile=1
    ssh "${ssh_args[@]}" root@"$board" 'mkdir -p /run/raven-simulation; cat > /run/raven-simulation/following.yaml' < "$scenario_config/following.yaml"
  fi
  ssh "${ssh_args[@]}" root@"$board" 'mkdir -p /run/raven-simulation; cat > /run/raven-simulation/camera.yaml' < "$camera"
  if [ -n "$follower_binary" ]; then
    test -r "$follower_binary"
    ssh "${ssh_args[@]}" root@"$board" 'systemctl stop target-follower; cat > /run/raven-simulation/target-follower; chmod 0755 /run/raven-simulation/target-follower' < "$follower_binary"
    binary=1
  fi
fi
ssh "${ssh_args[@]}" root@"$board" /bin/sh -s -- "$mode" "$trace" "$profile" "$binary" <<'REMOTE'
set -e
mode=$1
trace=$2
profile=$3
binary=$4
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
    sed -i '/^ExecStart=./s|$| -p source:=rtp -p rtp_capture_timestamps:=true -p input_uri:=udp://0.0.0.0:5602 -p source_width:=640 -p source_height:=360 -p photo_width:=640 -p photo_height:=360 -p ros_fps:=30 -p optical_frame:=sim_camera_optical_frame -p camera_info_url:=file:///run/raven-simulation/camera.yaml|' "$override"
  elif [ "$service" = target-follower ]; then
    if [ "$binary" = 1 ]; then
      sed -i 's|^ExecStart=/opt/ros/jazzy/lib/target_follower/target-follower |ExecStart=/run/raven-simulation/target-follower |' "$override"
    fi
    if [ "$profile" = 1 ]; then
      sed -i '/^ExecStart=./s|$| --params-file /run/raven-simulation/following.yaml|' "$override"
    else
      sed -i '/^ExecStart=./s|$| -p ground_z:=0.2 -p camera_pitch:=0.6108652381980153 -p camera_forward:=0.5 -p camera_right:=0.0 -p camera_down:=0.2|' "$override"
    fi
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

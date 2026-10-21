#!/bin/bash
# SPDX-License-Identifier: AGPL-3.0-only
# Temporary /run overrides only. Land the simulated aircraft before stop.
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
mode=${1:?Choose start or stop}
trace=${RAVEN_SIM_TRACE:-0}
controller=${RAVEN_SIM_CONTROLLER:-sitl}
case "$controller" in
  sitl) ;;
  hitl) [[ "${RAVEN_M7_HITL:-}" = 1 ]] || { echo 'M7 HITL requires RAVEN_M7_HITL=1' >&2; exit 2; } ;;
  *) echo 'Choose sitl or hitl controller' >&2; exit 2;;
esac
case "$trace" in 0|1) ;; *) echo "RAVEN_SIM_TRACE must be 0 or 1" >&2; exit 2;; esac
board=${2:?Provide board IPv4 address}
scenario_config=${3:-}
follower_binary=${RAVEN_SIM_GUIDANCE_BINARY:-}
guidance_config=${RAVEN_SIM_GUIDANCE_CONFIG:-}
vision_model=${RAVEN_SIM_VISION_MODEL:-}
vision_binary=${RAVEN_SIM_VISION_BINARY:-}
vision_config=${RAVEN_SIM_VISION_CONFIG:-}
vision_override=0
[[ "$vision_model" =~ ^[A-Za-z0-9_-]*$ ]] || { echo "Invalid model identifier" >&2; exit 2; }
case "$mode" in start|stop) ;; *) echo 'Choose start or stop' >&2; exit 2;; esac
[[ "$board" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]] || { echo 'Expected IPv4 address' >&2; exit 2; }
ssh_args=(-o BatchMode=yes -o ConnectTimeout=10 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 -o IPQoS=none)
if [ -n "${RAVEN_SIM_SSH_SOCKET:-}" ]; then
  [[ "$RAVEN_SIM_SSH_SOCKET" =~ ^/[-/A-Za-z0-9_.%]+$ ]] || { echo 'Expected SSH socket path' >&2; exit 2; }
  ssh_args+=(-o ControlPath="$RAVEN_SIM_SSH_SOCKET" -o ControlMaster=auto -o ControlPersist=300)
fi
profile=0
binary=0
config=0
if [ "$mode" = start ]; then
  if [ "$controller" = hitl ]; then
    python3 "$root/tests/hitl_preflight.py" "$board"
  fi
  if [ -n "${RAVEN_SIM_GUIDANCE_UNIT:-}" ]; then
    ssh "${ssh_args[@]}" root@"$board" 'cat > /run/systemd/system/guidance.service; systemctl daemon-reload' < "$RAVEN_SIM_GUIDANCE_UNIT"
  fi
  if [ -n "$guidance_config" ]; then
    test -r "$guidance_config"
    ssh "${ssh_args[@]}" root@"$board" 'mkdir -p /run/raven-simulation; cat > /run/raven-simulation/guidance.yaml' < "$guidance_config"
    config=1
  fi
  if [ -n "$vision_binary" ] || [ -n "$vision_config" ]; then
    test -r "$vision_binary"
    test -r "$vision_config"
    ssh "${ssh_args[@]}" root@"$board" 'mkdir -p /run/raven-simulation; systemctl stop vision-engine; cat > /run/raven-simulation/vision-engine; chmod 0755 /run/raven-simulation/vision-engine' < "$vision_binary"
    ssh "${ssh_args[@]}" root@"$board" 'cat > /run/raven-simulation/vision.json' < "$vision_config"
    vision_override=1
  fi
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
    ssh "${ssh_args[@]}" root@"$board" 'systemctl stop guidance; cat > /run/raven-simulation/guidance; chmod 0755 /run/raven-simulation/guidance' < "$follower_binary"
    binary=1
  fi
fi
ssh "${ssh_args[@]}" root@"$board" /bin/sh -s -- "$mode" "$trace" "$profile" "$binary" "$config" "${vision_model:--}" "$controller" "$vision_override" <<'REMOTE'
set -e
mode=$1
trace=$2
profile=$3
binary=$4
config=$5
vision_model=$6
[ "$vision_model" = - ] && vision_model=
controller=$7
vision_override=$8
sim_time=true
[ "$controller" = hitl ] && sim_time=false
if systemctl cat guidance >/dev/null 2>&1; then systemctl stop guidance; fi
for service in media-capture vision-engine target-tracker guidance; do
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
ExecStart=/opt/ros/jazzy/lib/$package/$service --ros-args --params-file /etc/$service.yaml -p use_sim_time:=$sim_time
CONF
  if [ "$trace" = 1 ]; then
    case "$service" in
      media-capture) logger=raven.vehicle_1.cameras.front.media_capture ;;
      guidance) logger=raven.vehicle_1.autonomy.guidance ;;
      *) logger=raven.vehicle_1.perception.$package ;;
    esac
    sed -i "/^ExecStart=\/opt/s|\$| --log-level $logger:=debug|" "$override"
  fi
  if [ "$service" = media-capture ]; then
    sed -i '/^ExecStart=./s|$| -p source:=rtp -p rtp_capture_timestamps:=true -p input_uri:=udp://0.0.0.0:5602 -p source_width:=640 -p source_height:=360 -p photo_width:=640 -p photo_height:=360 -p ros_fps:=30 -p optical_frame:=sim_camera_optical_frame -p camera_info_url:=file:///run/raven-simulation/camera.yaml|' "$override"
  elif [ "$service" = vision-engine ]; then
    if [ -n "$vision_model" ]; then
      sed -i "/^ExecStart=./s|\$| -p model:=$vision_model|" "$override"
    fi
    if [ "$vision_override" = 1 ]; then
      sed -i 's|^ExecStart=/opt/ros/jazzy/lib/vision_engine/vision-engine |ExecStart=/run/raven-simulation/vision-engine |; /^ExecStart=./s|$| -p config_file:=/run/raven-simulation/vision.json|' "$override"
      cat >> "$override" <<CONF
RuntimeDirectory=vision-engine-validation
RuntimeDirectoryMode=0700
ReadWritePaths=/run/vision-engine-validation
CONF
    fi
  elif [ "$service" = guidance ]; then
    if [ "$config" = 1 ]; then
      sed -i 's|/etc/guidance.yaml|/run/raven-simulation/guidance.yaml|' "$override"
    fi
    sed -i '/^ExecStart=./s|$| -p allow_unmapped_motion:=true|' "$override"
    if [ "$binary" = 1 ]; then
      sed -i 's|^ExecStart=/opt/ros/jazzy/lib/guidance/guidance |ExecStart=/run/raven-simulation/guidance |' "$override"
    fi
    if [ "$profile" = 1 ]; then
      sed -i '/^ExecStart=./s|$| --params-file /run/raven-simulation/following.yaml|' "$override"
    else
      sed -i '/^ExecStart=./s|$| -p ground_z:=0.2 -p camera_pitch:=0.6108652381980153 -p camera_forward:=0.5 -p camera_right:=0.0 -p camera_down:=0.2|' "$override"
    fi
  fi
done
if [ "$mode" = start ] && [ "$controller" = sitl ]; then
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
  if systemctl cat px4-simulation-agent.service >/dev/null 2>&1; then systemctl stop px4-simulation-agent.service; fi
  rm -f /run/systemd/system/px4-simulation-agent.service
fi
if [ "$mode" = stop ]; then
  rm -f /run/systemd/system/guidance.service
fi
systemctl daemon-reload
systemctl restart media-capture vision-engine target-tracker
if [ "$mode" = start ]; then
  if [ "$controller" = sitl ]; then systemctl start px4-simulation-agent.service; fi
  systemctl start guidance
fi
systemctl is-active media-capture vision-engine target-tracker
REMOTE

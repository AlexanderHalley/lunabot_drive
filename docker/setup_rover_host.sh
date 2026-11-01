#!/usr/bin/env bash
#
# One-time host setup for the rover (Jetson Orin, JetPack 6), so it can run the
# robot image: Docker, the DDS receive buffer, CAN at boot, the data tree.
# docs/DEPLOYMENT.md "The rover, once" is the same list, explained.
#
#   sudo docker/setup_rover_host.sh [--can native|slcan|none] [--user NAME]
#
# Idempotent: run it again after a JetPack reflash, or to change --can.
# --can native  the Orin's own CAN controller (can0), via systemd-networkd
# --can slcan   the 2026 USB-to-CAN adapter on /dev/ttyACM0, via a systemd unit
# --can none    leave CAN alone (bench, or set up by hand)

set -euo pipefail

CAN=native
TARGET_USER=${SUDO_USER:-}
while [[ $# -gt 0 ]]; do
  case "$1" in
    --can) CAN=$2; shift 2 ;;
    --user) TARGET_USER=$2; shift 2 ;;
    -h|--help) sed -n '2,15p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
[[ $EUID -eq 0 ]] || { echo "run with sudo" >&2; exit 1; }
[[ -n "$TARGET_USER" ]] || { echo "which user runs the rover? --user NAME" >&2; exit 1; }
case "$CAN" in native|slcan|none) ;; *) echo "--can native|slcan|none" >&2; exit 2 ;; esac
HOME_DIR=$(getent passwd "$TARGET_USER" | cut -d: -f6)

say() { echo "== $*"; }

say "host: $(uname -m), $(. /etc/os-release && echo "$PRETTY_NAME")"
[[ -f /etc/nv_tegra_release ]] && say "L4T: $(head -1 /etc/nv_tegra_release)"
[[ "$(uname -m)" == aarch64 ]] || echo "   note: not aarch64 -- fine for testing, but the rover is an Orin"

# --- Docker -------------------------------------------------------------------
if ! command -v docker >/dev/null || ! docker compose version >/dev/null 2>&1; then
  say "installing Docker"
  apt-get update
  apt-get install -y docker.io docker-compose-v2
fi
systemctl enable --now docker
usermod -aG docker "$TARGET_USER"
say "docker $(docker --version | cut -d' ' -f3 | tr -d ,), compose $(docker compose version --short); $TARGET_USER in group docker (log in again)"

# --- DDS receive buffer (docker/cyclonedds.xml asks for 10 MB) ----------------
echo 'net.core.rmem_max=10485760' > /etc/sysctl.d/60-lunabot-dds.conf
sysctl -q --system
say "net.core.rmem_max = $(sysctl -n net.core.rmem_max)"

# --- CAN at boot, 1 Mbit/s (docs/HARDWARE_CAN.md) ------------------------------
case "$CAN" in
  native)
    # The Orin's CAN controller is the mttcan driver.
    modprobe mttcan 2>/dev/null || echo "   note: no mttcan module -- is this an Orin with CAN pins?"
    echo mttcan > /etc/modules-load.d/lunabot-can.conf
    cat > /etc/systemd/network/80-lunabot-can0.network <<'EOF'
[Match]
Name=can0

[CAN]
BitRate=1M
RestartSec=100ms
EOF
    rm -f /etc/systemd/system/lunabot-slcan.service
    systemctl enable --now systemd-networkd
    systemctl restart systemd-networkd
    ;;
  slcan)
    cat > /etc/systemd/system/lunabot-slcan.service <<'EOF'
[Unit]
Description=Lunabot USB-to-CAN adapter as can0 (slcan, 1 Mbit/s)
After=dev-ttyACM0.device
BindsTo=dev-ttyACM0.device

[Service]
Type=forking
ExecStart=/usr/bin/slcand -o -s8 /dev/ttyACM0 can0
ExecStartPost=/bin/sleep 1
ExecStartPost=/usr/sbin/ip link set can0 up type can
ExecStop=/usr/sbin/ip link set can0 down
Restart=on-failure

[Install]
WantedBy=multi-user.target
EOF
    command -v slcand >/dev/null || apt-get install -y can-utils
    rm -f /etc/systemd/network/80-lunabot-can0.network
    systemctl daemon-reload
    systemctl enable --now lunabot-slcan.service || echo "   note: adapter not plugged in? it starts when it is"
    ;;
  none) ;;
esac
if [[ "$CAN" != none ]]; then
  if ip link show can0 >/dev/null 2>&1; then
    say "can0: $(ip -details link show can0 | grep -oE 'state [A-Z-]+|bitrate [0-9]+' | tr '\n' ' ')"
  else
    echo "   note: can0 not present yet"
  fi
fi

# --- The data tree (bags and ROS logs from the containers) ----------------------
install -d -o "$TARGET_USER" -g "$TARGET_USER" \
  "$HOME_DIR/lunabotics/data/bags" "$HOME_DIR/lunabotics/data/logs/ros"
say "data: $HOME_DIR/lunabotics/data/{bags,logs/ros}"

cat <<EOF

Done. Next, as $TARGET_USER (after logging in again):
  docker login ghcr.io -u <github user>        # only if the image package is private
  docker/deploy.sh                             # pull, start, check
EOF

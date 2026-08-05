#!/usr/bin/env bash
#
# Launch Isaac Sim with the Lunabot scene.
#
#   src/lunabot_sim/scripts/run_isaac_sim.sh --scene default --seed 7
#
# This script is the seam between two Python environments that must not mix:
#
#   1. Expand lunabot.urdf.xacro using the ROS environment's xacro.
#   2. exec Isaac's OWN python on run_sim.py, passing the expanded URDF.
#
# Isaac's embedded Python is not the ROS distro's Python. Trying to import
# xacro or rclpy inside Isaac is a losing fight; passing a plain file across
# the boundary is not. All ROS traffic goes through the bridge extension,
# which links its own DDS.
#
# Then, in a second terminal:
#
#   ros2 launch lunabot_bringup robot.launch.py hw:=sim use_sim_time:=true
#
# ORDER MATTERS: start this first. /clock must be publishing before the ROS
# graph comes up, or every node with use_sim_time blocks at time zero -- with
# no error and no log line.

set -euo pipefail

SCENE="default"
SEED=0
HEADLESS=""
DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
CACHE_DIR="${LUNABOT_SIM_CACHE:-${HOME}/.cache/lunabot_sim}"

usage() {
  sed -n '2,30p' "$0" | sed 's/^# \?//'
  exit "${1:-0}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --scene)     SCENE="$2"; shift 2 ;;
    --seed)      SEED="$2"; shift 2 ;;
    --domain-id) DOMAIN_ID="$2"; shift 2 ;;
    --headless)  HEADLESS="--headless"; shift ;;
    -h|--help)   usage 0 ;;
    *) echo "unknown argument: $1" >&2; usage 1 ;;
  esac
done

# ---------------------------------------------------------------------------
# 1. Expand the xacro, in the ROS environment
# ---------------------------------------------------------------------------
if ! command -v xacro >/dev/null 2>&1; then
  echo "error: xacro not found. Source the ROS workspace first:" >&2
  echo "  source /opt/ros/jazzy/setup.bash && source install/setup.bash" >&2
  exit 1
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "error: ros2 not found. Source the workspace first." >&2
  exit 1
fi

DESCRIPTION_SHARE="$(ros2 pkg prefix lunabot_description)/share/lunabot_description"
XACRO_FILE="${DESCRIPTION_SHARE}/urdf/lunabot.urdf.xacro"

if [[ ! -f "${XACRO_FILE}" ]]; then
  echo "error: ${XACRO_FILE} not found. Build the workspace first." >&2
  exit 1
fi

mkdir -p "${CACHE_DIR}"
URDF_FILE="${CACHE_DIR}/lunabot_sim.urdf"

echo "expanding ${XACRO_FILE} -> ${URDF_FILE}"
# hardware:=sim selects topic_based_ros2_control. The <ros2_control> block is
# not read by Isaac at all -- it is there so the SAME file describes the robot
# for the ROS side, which is the whole point of one xacro for three targets.
xacro "${XACRO_FILE}" hardware:=sim > "${URDF_FILE}"

# ---------------------------------------------------------------------------
# 2. Hand off to Isaac's Python
# ---------------------------------------------------------------------------
# ISAAC_SIM_PATH must point at the install root -- the directory containing
# python.sh. Common locations differ by install method, so check a few.
if [[ -z "${ISAAC_SIM_PATH:-}" ]]; then
  for candidate in \
    "${HOME}/isaacsim" \
    "${HOME}/.local/share/ov/pkg/isaac-sim-"* \
    "${HOME}/.local/share/ov/pkg/isaac_sim-"* \
    "/isaac-sim"
  do
    if [[ -x "${candidate}/python.sh" ]]; then
      ISAAC_SIM_PATH="${candidate}"
      break
    fi
  done
fi

if [[ -z "${ISAAC_SIM_PATH:-}" || ! -x "${ISAAC_SIM_PATH}/python.sh" ]]; then
  echo "error: could not find Isaac Sim's python.sh." >&2
  echo "Set ISAAC_SIM_PATH to the install root, e.g.:" >&2
  echo "  export ISAAC_SIM_PATH=\$HOME/isaacsim" >&2
  exit 1
fi

echo "using Isaac Sim at ${ISAAC_SIM_PATH}"

# lunabot_sim must be importable by ISAAC's python, which knows nothing about
# the colcon install tree. Point PYTHONPATH at the package source directly.
PACKAGE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${PACKAGE_ROOT}:${PYTHONPATH:-}"

# The bridge reads this to pick a domain when useDomainIDEnvVar is set. The
# graphs set the id explicitly, so this is belt and braces.
export ROS_DOMAIN_ID="${DOMAIN_ID}"

exec "${ISAAC_SIM_PATH}/python.sh" -m lunabot_sim.run_sim \
  --urdf "${URDF_FILE}" \
  --scene "${SCENE}" \
  --seed "${SEED}" \
  --domain-id "${DOMAIN_ID}" \
  ${HEADLESS}

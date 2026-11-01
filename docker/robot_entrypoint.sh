#!/usr/bin/env bash
#
# The rover image's entrypoint: source ROS and the workspace, then either run
# the command given, or launch $LUNABOT_LAUNCH (compose.robot.yml sets it).
#
# `set -e` deliberately comes AFTER the sourcing: ROS setup scripts reference
# unset variables and return non-zero in situations that are not errors.

# shellcheck disable=SC1090,SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /opt/lunabot/setup.bash

set -e
echo "lunabot image ${LUNABOT_GIT_SHA:-unknown}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" >&2

if [[ $# -gt 0 ]]; then
  exec "$@"
fi
# Word-split on purpose: LUNABOT_LAUNCH is "<file> arg:=value ...".
# shellcheck disable=SC2086
exec ros2 launch lunabot_bringup ${LUNABOT_LAUNCH}

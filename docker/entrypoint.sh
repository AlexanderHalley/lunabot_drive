#!/usr/bin/env bash
#
# Sources the ROS environment and, if the workspace has been built, its
# overlay. Then execs whatever was asked for.
#
# `set -e` deliberately comes AFTER the sourcing: ROS setup scripts reference
# unset variables under `set -u` and return non-zero in situations that are
# not errors, so guarding them would break every container start.

# shellcheck disable=SC1090,SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ -f /workspace/install/setup.bash ]]; then
  source /workspace/install/setup.bash
else
  echo "note: /workspace/install not found -- run 'colcon build' first." >&2
fi

# Large image topics need a bigger receive buffer than the CycloneDDS
# default, or frames drop under load in a way that looks like a camera
# problem. Same configuration as the robot and the offboard machine.
if [[ ! -f "${HOME}/.ros/cyclonedds.xml" ]]; then
  mkdir -p "${HOME}/.ros"
  cat > "${HOME}/.ros/cyclonedds.xml" <<'XML'
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS>
  <Domain>
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
    </General>
    <Internal>
      <SocketReceiveBufferSize min="10MB"/>
    </Internal>
  </Domain>
</CycloneDDS>
XML
fi
export CYCLONEDDS_URI="file://${HOME}/.ros/cyclonedds.xml"

set -e
exec "$@"

# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
#
# Shared setup for the scripts that hand off to Isaac's Python. Sourced, not
# executed:
#
#   source "$(dirname "${BASH_SOURCE[0]}")/isaac_env.sh"
#   lunabot_sim_find_isaac
#   lunabot_sim_export_pythonpath
#
# It exists because two scripts now cross the same seam -- run_isaac_sim.sh and
# probe_isaac_api.sh -- and the install-location search is the part most likely
# to need editing on a new machine. One copy of it, or the probe finds an Isaac
# the simulator does not.

# Locate Isaac Sim's own python.sh and export ISAAC_SIM_PATH.
#
# ISAAC_SIM_PATH must point at the install ROOT: the directory containing
# python.sh, not the python.sh itself. An already-set value is trusted and
# only checked, so a machine with an unusual layout is one export away.
lunabot_sim_find_isaac() {
  if [[ -z "${ISAAC_SIM_PATH:-}" ]]; then
    local candidate
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
    return 1
  fi

  export ISAAC_SIM_PATH
  echo "using Isaac Sim at ${ISAAC_SIM_PATH}"
}

# Put the lunabot_sim SOURCE directory on PYTHONPATH.
#
# The source directory, deliberately, not the colcon install tree: these
# scripts are run by Isaac's embedded Python, which knows nothing about
# AMENT_PREFIX_PATH and cannot find an ament_python install.
lunabot_sim_export_pythonpath() {
  local package_root
  package_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
  export PYTHONPATH="${package_root}:${PYTHONPATH:-}"
}

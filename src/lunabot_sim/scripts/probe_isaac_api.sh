#!/usr/bin/env bash
#
# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
#
# Resolve every VERIFY in lunabot_sim against a real Isaac install.
#
#   src/lunabot_sim/scripts/probe_isaac_api.sh
#   src/lunabot_sim/scripts/probe_isaac_api.sh --json /tmp/isaac_probe.json
#
# THE FIRST THING TO RUN on the simulation machine. It starts Isaac headless,
# asks it which extension ids, OmniGraph node types, sensor classes and URDF
# importer entry points actually exist, prints the answers next to what
# compat.py currently guesses, and exits non-zero if anything is missing.
#
# It builds no scene, imports no robot and publishes no ROS topics, so it is
# safe to run against a fresh install before anything else has been tried.
#
# Unlike run_isaac_sim.sh this needs no ROS environment: nothing is expanded
# from xacro, so the ROS side never enters into it.
#
# See docs/SIM_ACCEPTANCE.md for where this fits in the bring-up order.

set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/isaac_env.sh"

lunabot_sim_find_isaac
lunabot_sim_export_pythonpath

exec "${ISAAC_SIM_PATH}/python.sh" -m lunabot_sim.probe "$@"

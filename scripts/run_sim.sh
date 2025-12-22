#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

GZ_BIN="/opt/ros/rolling/opt/gz_tools_vendor/bin/gz"
GZ_SYSTEM_PLUGIN_DIR="/opt/ros/rolling/opt/gz_sim_vendor/lib/gz-sim-9/plugins"
PY_LOADER="${GZ_SYSTEM_PLUGIN_DIR}/libgz-sim9-python-system-loader-system.so"

if [[ ! -f "${PY_LOADER}" ]]; then
  GZ_BIN="/usr/bin/gz"
  GZ_SYSTEM_PLUGIN_DIR="/usr/lib/x86_64-linux-gnu/gz-sim-9/plugins"
  export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

export GZ_SIM_RESOURCE_PATH="${PROJECT_ROOT}/sim/gazebo/models${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SYSTEM_PLUGIN_DIR}:${PROJECT_ROOT}/sim/gazebo/plugins${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"
export PYTHONPATH="${PROJECT_ROOT}/sim/gazebo/plugins${PYTHONPATH:+:${PYTHONPATH}}"
exec "${GZ_BIN}" sim -r "${PROJECT_ROOT}/sim/gazebo/worlds/car_world.sdf"

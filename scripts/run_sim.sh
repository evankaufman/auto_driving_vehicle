#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

export GZ_SIM_RESOURCE_PATH="${PROJECT_ROOT}/sim/gazebo/models${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
exec gz sim -r "${PROJECT_ROOT}/sim/gazebo/worlds/car_world.sdf"

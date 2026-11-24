#!/usr/bin/env bash
# Original instructor-supplied packages only; no local bridge or el_scenario.
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/misys/forza_ws/race_stack/install/setup.bash
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}" OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
cd "$root"
exec python3 scripts/official_eval.py "$@"

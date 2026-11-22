#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/misys/forza_ws/race_stack/install/setup.bash
export PYTHONPATH=/home/misys/forza_ws/race_stack/base_system/f110_simulator/f1tenth_gym/gym:${PYTHONPATH:-}
export OMP_NUM_THREADS=2
exec /build/el_eval_avoid "$@"

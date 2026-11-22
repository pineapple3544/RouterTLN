#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/misys/forza_ws/race_stack/install/setup.bash
export PYTHONPATH=/home/misys/forza_ws/race_stack/base_system/f110_simulator/f1tenth_gym/gym:${PYTHONPATH:-}
export OMP_NUM_THREADS=2
/build/el_collect_baseline_20261029 /workspace/config/teacher_holdout_baseline.json > /workspace/runs/teacher-improvement/holdout-baseline.log 2>&1
/build/el_collect /workspace/config/teacher_holdout_improved.json > /workspace/runs/teacher-improvement/holdout-improved.log 2>&1
/build/el_collect /workspace/config/teacher_cycling_improved.json > /workspace/runs/teacher-improvement/cycling-improved.log 2>&1

#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/misys/forza_ws/race_stack/install/setup.bash
cd "$(dirname "$0")"
export PYTHONPATH="/home/misys/forza_ws/race_stack/base_system/f110_simulator/f1tenth_gym/gym:$PWD:${PYTHONPATH:-}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
python3 build.py
for pair in 'failed 0' 'brake 3'; do
    read -r key phase <<< "$pair"
    python3 render.py "$key" "$phase" 181
    if [[ $key == failed ]]; then rows=(820 882 898 908); else rows=(514 561 606 625 637 666); fi
    ./replay "$key" "$phase" shadow "${rows[@]}"
    for row in "${rows[@]}"; do ./replay "$key" "$phase" "$row" tln-sync; done
done

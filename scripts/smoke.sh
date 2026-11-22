#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/misys/forza_ws/race_stack/install/setup.bash
duration=${1:-35}
obstacles=${2:-0}
mode=${3:-moe}
tag=${4:-smoke}
episode=${5:-}
seed=${6:-2026}
export EL_MAP_DIR=${EL_MAP_DIR:-/workspace/maps/berlin}
EL_MAP_DIR=${EL_MAP_DIR%/}
mkdir -p /workspace/runs
cleanup() {
  for task in "$controller" "$scenario" "$sim"; do
    if [ "$task" -gt 1 ]; then kill -INT -- "-$task" 2>/dev/null || true; fi
  done
  sleep 2
  for task in "$controller" "$scenario" "$sim"; do
    if [ "$task" -gt 1 ]; then kill -TERM -- "-$task" 2>/dev/null || true; fi
  done
  wait "$controller" "$scenario" "$sim" 2>/dev/null || true
}
sim=0;scenario=0;controller=0
trap cleanup EXIT
sim_affinity=()
control_affinity=()
if [[ -n ${EL_SIM_CPUSET:-} ]]; then sim_affinity=(taskset -c "$EL_SIM_CPUSET"); fi
if [[ -n ${EL_CONTROL_CPUSET:-} ]]; then control_affinity=(taskset -c "$EL_CONTROL_CPUSET"); fi
EL_LOCAL_STEADY_SIM=1 setsid "${sim_affinity[@]}" bash /workspace/scripts/run_sim.sh > "/workspace/runs/$tag-sim.log" 2>&1 & sim=$!
scenario_args=(-p use_sim_time:=true -p publish_steady_clock:=true -p "start_index:=${EL_START_INDEX:-0}")
if [ -n "${EL_OBSTACLE_OFFSETS:-}" ]; then scenario_args+=(-p "obstacle_offsets:=$EL_OBSTACLE_OFFSETS"); fi
setsid /build/el_scenario --ros-args "${scenario_args[@]}" -p "map_dir:=$EL_MAP_DIR" -p obstacle_count:="$obstacles" -p obstacle_stride:="${EL_OBSTACLE_STRIDE:-200}" -p obstacle_index:="${EL_OBSTACLE_INDEX:-100}" -p seed:="$seed" > "/workspace/runs/$tag-scenario.log" 2>&1 & scenario=$!
sleep 5
kill -0 "$sim" "$scenario"
extra=(-p use_sim_time:=true -p local_steady_sim_clock:=true)
if [ -n "$episode" ]; then extra+=(-p "episode:=$episode"); fi
extra+=(-p "map_dir:=$EL_MAP_DIR")
if [ -n "${EL_LAP_PLANNING:-}" ]; then extra+=(-p "lap_planning:=$EL_LAP_PLANNING"); fi
if [ -n "${EL_SPEED_CAP:-}" ]; then extra+=(-p "max_speed:=$EL_SPEED_CAP"); fi
if [ -n "${EL_STOP_AFTER_LAPS:-}" ]; then extra+=(-p "stop_after_laps:=$EL_STOP_AFTER_LAPS"); fi
if [[ -n ${EL_SIM_AVOID_MIN:-} ]]; then extra+=(-p "sim_avoid_min:=$EL_SIM_AVOID_MIN"); fi
if [[ -n ${EL_SIM_AVOID_MAX:-} ]]; then extra+=(-p "sim_avoid_max:=$EL_SIM_AVOID_MAX"); fi
if [[ -n ${EL_INFERENCE_DEVICE:-} ]]; then extra+=(-p "inference_device:=$EL_INFERENCE_DEVICE"); fi
if [[ -n ${EL_AVOID_CHECKPOINT:-} ]]; then extra+=(-p "avoid_checkpoint:=$EL_AVOID_CHECKPOINT"); fi
if [ -n "${EL_RACE_CHECKPOINT:-}" ]; then extra+=(-p "race_checkpoint:=$EL_RACE_CHECKPOINT"); fi
setsid "${control_affinity[@]}" /build/el_controller --ros-args -p mode:="$mode" -p metrics:="/workspace/runs/$tag.csv" -p seed:="$seed" "${extra[@]}" \
  -p scenario_id:="${EL_MAP_DIR##*/}-count${obstacles}-index${EL_OBSTACLE_INDEX:-100}-seed${seed}" \
  > "/workspace/runs/$tag-controller.log" 2>&1 & controller=$!
if [[ ${EL_STOP_AFTER_LAPS:-0} =~ ^[1-9][0-9]*$ ]]; then
  # A lap trial has a terminal state. Keep recording until the finish command
  # or until a latched emergency has actually brought the car to rest.
  end=$((SECONDS + duration))
  while (( SECONDS < end )); do
    kill -0 "$controller"
    if [[ -s /workspace/runs/$tag.csv ]] && tail -n 1 "/workspace/runs/$tag.csv" |
        awk -F, 'END {exit !(NF >= 15 && $1 ~ /^[0-9]+$/ && $8 == 0 && $11 == 0 && ($6 == 0 || ($6 == 3 && $5 < 0.05)))}'; then
      break
    fi
    sleep 1
  done
else
  sleep "$duration"
fi
kill -0 "$controller"
cleanup
trap - EXIT
/build/el_inspect --metrics "/workspace/runs/$tag.csv"
if [ -n "$episode" ]; then /build/el_inspect --dataset "$episode"; fi

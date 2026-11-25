#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
base=/home/misys/forza_ws/race_stack
if [[ -z ${EL_SIM_CONFIG_DIR:-} ]]; then source "$base/install/setup.bash"; fi
sim_config=${EL_SIM_CONFIG_DIR:-$base/stack_master/config/SIM}
map_dir=${EL_MAP_DIR:-/workspace/maps/berlin}
map_dir=${map_dir%/}
stem="$map_dir/${map_dir##*/}"
if [[ -f "${stem}_orig.yaml" ]]; then stem="${stem}_orig"; fi
pose=$(python3 -c 'import json, sys
with open(sys.argv[1]) as f:
    w = json.load(f)["global_traj_wpnts_iqp"]["wpnts"][0]
print(w["x_m"], w["y_m"], w["psi_rad"])' "$map_dir/global_waypoints.json")
read -r sx sy stheta <<< "$pose"
clock_args=()
if [[ ${EL_LOCAL_STEADY_SIM:-0} == 1 ]]; then clock_args=(-p use_sim_time:=true); fi
bridge=(ros2 run f1tenth_gym_ros gym_bridge)
if [[ ${EL_LOCAL_STEADY_SIM:-0} == 1 ]]; then bridge=(python3 /workspace/scripts/local_gym_bridge.py); fi
exec "${bridge[@]}" --ros-args \
  "${clock_args[@]}" -r __node:=bridge \
  --params-file "$sim_config/sim.yaml" \
  -p map_path:="$stem.yaml" \
  -p sim_params:="$sim_config/sim_params.yaml" \
  -p kb_teleop:=false -p sx:="$sx" -p sy:="$sy" -p stheta:="$stheta"

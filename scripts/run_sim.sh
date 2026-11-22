#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/misys/forza_ws/race_stack/install/setup.bash
base=/home/misys/forza_ws/race_stack
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
exec ros2 run f1tenth_gym_ros gym_bridge --ros-args \
  "${clock_args[@]}" -r __node:=bridge \
  --params-file "$base/stack_master/config/SIM/sim.yaml" \
  -p map_path:="$stem.yaml" \
  -p sim_params:="$base/stack_master/config/SIM/sim_params.yaml" \
  -p kb_teleop:=false -p sx:="$sx" -p sy:="$sy" -p stheta:="$stheta"

#!/usr/bin/env bash
# RouterTLN demo: Berlin simulator + RouterTLN controller + RViz + race_manager, one command.
#
#   ./run_demo.sh                 # 1 obstacle per lap, race_manager seed 2026, 10 laps
#   ./run_demo.sh 3               # 3 obstacles
#   ./run_demo.sh 5 3105 10       # 5 obstacles, seed 3105, stop after 10 laps
#
# Obstacles come from the shipped race_manager obstacle_spawner (re-drawn every lap); lap
# times and collisions from the shipped race_manager scorer. The run ends by itself after LAPS
# laps (or after a protective stop); Ctrl-C ends it early. Results: runs/demo-*/ (the scorer's
# race_result_*.json, controller CSV and logs). Set DEMO_HEADLESS=1 to run without windows.
#
# The controller only subscribes to /scan; the simulator's ground-truth pose feeds the scorer
# and RViz only. The driving configuration is config/final_demo.env (same for 1/3/5 obstacles).
set -eo pipefail

obstacles=${1:-1}
seed=${2:-2026}
laps=${3:-10}
for v in "$obstacles" "$seed" "$laps"; do
  [[ $v =~ ^[0-9]+$ ]] || { echo "usage: $0 [obstacles] [seed] [laps]  (non-negative integers)" >&2; exit 2; }
done
(( laps >= 1 )) || { echo "laps must be >= 1" >&2; exit 2; }

ROOT=/workspace
if [[ ! -f $ROOT/scripts/smoke.sh || ! -x /build/el_controller ]]; then
  echo "RouterTLN is not installed: run install_final_demo.sh first (it creates /workspace and /build)." >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source /home/misys/forza_ws/race_stack/install/setup.bash
set -u

# the one fixed driving configuration
while IFS='=' read -r key value; do
  [[ -z $key || $key == \#* ]] && continue
  export "$key=$value"
done < "$ROOT/config/final_demo.env"

tag=demo-$(date +%Y%m%d-%H%M%S)-obs$obstacles-seed$seed
out=$ROOT/runs/$tag
mkdir -p "$out"
rm_cfg=$(ros2 pkg prefix --share race_manager)/config/race_manager_params.yaml
waypoints=$ROOT/maps/berlin/global_waypoints.json
headless=${DEMO_HEADLESS:-0}
[[ -z ${DISPLAY:-} ]] && headless=1

pids=()
start() {  # start <log name> <command...> in its own process group
  local log=$1; shift
  setsid "$@" > "$out/$log.log" 2>&1 &
  pids+=($!)
}
finished=0
finish() {  # stop everything; SIGINT first so the scorer prints and saves its summary
  (( finished )) && return
  finished=1
  for p in "${pids[@]}"; do kill -INT -- "-$p" 2>/dev/null || true; done
  sleep 3
  for p in "${pids[@]}"; do kill -TERM -- "-$p" 2>/dev/null || true; done
  wait 2>/dev/null || true
  echo
  # The scorer's own summary can be lost when it raises on shutdown; its JSON is complete.
  result=$(ls -t "$out"/race_result/race_result_*.json 2>/dev/null | head -1 || true)
  if [[ -n $result ]]; then
    python3 - "$result" <<'PY' || true
import json, sys
r = json.load(open(sys.argv[1]))
c = r.get('collisions', {})
print(f"---- race_manager: seed {r.get('seed')}, {r.get('laps_completed')} laps ----")
print(f"  best lap    {r.get('best_lap')} s")
print(f"  collisions  {c.get('total', 0)}  (obstacle {c.get('obstacle', 0)}, wall {c.get('wall', 0)})")
for lap in r.get('laps', []):
    print(f"  lap {lap['lap']:>2}  {lap['time']:7.2f} s  hits {lap['total']}" + ('  partial' if lap.get('partial') else ''))
PY
    echo "race_manager result: $result"
  fi
  [[ -f $out/run-score.json ]] && python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); print("controller stop:", d.get("emergency") or d.get("controller_fault") or "none")' "$out/run-score.json" || true
  echo "logs: $out"
}
trap finish EXIT
trap 'echo; echo "stopping..."; exit 130' INT TERM

echo "RouterTLN demo: $obstacles obstacle(s) per lap, race_manager seed $seed, $laps laps"
echo "results: $out"

start spawner ros2 run race_manager obstacle_spawner --ros-args --params-file "$rm_cfg" \
  -p waypoints_file:="$waypoints" -p seed:="$seed" -p num_obstacles:="$obstacles"
start scorer ros2 run race_manager scorer --ros-args --params-file "$rm_cfg" \
  -p waypoints_file:="$waypoints" -p map_name:=berlin -p seed:="$seed" -p log_dir:="$out/race_result"

if [[ $headless != 1 ]]; then
  xacro_file=$(ros2 pkg prefix --share f1tenth_gym_ros)/config/ego_racecar.xacro
  start map_server ros2 run nav2_map_server map_server --ros-args \
    -p yaml_filename:="$ROOT/maps/berlin/berlin.yaml" -p topic:=map -p frame_id:=map -p use_sim_time:=true
  start lifecycle ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
    -r __node:=lifecycle_manager_demo -p use_sim_time:=true -p autostart:=true -p "node_names:=[map_server]"
  start robot_state ros2 run robot_state_publisher robot_state_publisher --ros-args \
    -r __node:=ego_robot_state_publisher -r /robot_description:=ego_robot_description \
    -p use_sim_time:=true -p robot_description:="$(xacro "$xacro_file")"
  start rviz rviz2 -d /home/misys/forza_ws/race_stack/forza_rviz.rviz --ros-args -p use_sim_time:=true
  # --watch follows the newest log under ./race_result, i.e. this run's scorer output
  start report bash -c "cd '$out' && exec ros2 run race_manager report --watch"
fi
sleep 3

# Simulator (shipped GymBridge via scripts/local_gym_bridge.py), el_scenario for the clock and the
# start pose only (its obstacle topic is moved aside: race_manager owns /sim/obstacles), controller.
budget=$(( laps * 18 + 30 ))
EL_SCENARIO_OBSTACLES=0 EL_START_INDEX=0 EL_OBSTACLE_OFFSETS= EL_STOP_AFTER_LAPS=$laps \
  bash "$ROOT/scripts/smoke.sh" "$budget" 0 moe "$tag/run" "" "$seed" > "$out/smoke.out" 2>&1 || true

#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/misys/forza_ws/race_stack/install/setup.bash
tag=${1:-watchdog}
prefix="/workspace/runs/$tag"
mkdir -p "$(dirname "$prefix")"
scenario=0;sim=0;controller=0
cleanup() {
  for task in "$controller" "$sim" "$scenario"; do
    if [ "$task" -gt 1 ]; then kill -INT -- "-$task" 2>/dev/null || true; fi
  done
  sleep 1
  for task in "$controller" "$sim" "$scenario"; do
    if [ "$task" -gt 1 ]; then kill -TERM -- "-$task" 2>/dev/null || true; fi
  done
  wait "$controller" "$sim" "$scenario" 2>/dev/null || true
}
trap cleanup EXIT
setsid /build/el_scenario --ros-args -p publish_steady_clock:=true -p use_sim_time:=true -p obstacle_count:=0 > "${prefix}-scenario.log" 2>&1 & scenario=$!
EL_LOCAL_STEADY_SIM=1 setsid bash /workspace/scripts/run_sim.sh > "${prefix}-sim.log" 2>&1 & sim=$!
sleep 4
kill -0 "$sim"
setsid /build/el_controller --ros-args -p use_sim_time:=true -p local_steady_sim_clock:=true -p metrics:="${prefix}.csv" > "${prefix}-controller.log" 2>&1 & controller=$!
sleep 4
kill -0 "$controller"
# Confirm a valid moving command before interrupting the real sensor publisher.
python3 - "$prefix.csv" <<'PY'
import csv
import sys
with open(sys.argv[1]) as stream:
    rows = list(csv.DictReader(stream))
assert rows and any(r["valid"] == "1" and float(r["speed"]) > .1 for r in rows), "never drove"
assert all(r["mode"] != "3" for r in rows), "already stopped before sensor dropout"
PY
kill -INT -- "-$sim"
sleep 2
timeout 8 ros2 topic echo /drive ackermann_msgs/msg/AckermannDriveStamped --once --field drive.speed > "${prefix}-command.txt"
grep -q "sensor_timeout: scan age" "${prefix}-controller.log"
grep -qx '0.0' "${prefix}-command.txt"
echo 'PASS: controller publishes zero target speed after simulator sensor stream stops'

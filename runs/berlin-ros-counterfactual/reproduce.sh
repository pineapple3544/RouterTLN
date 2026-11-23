#!/usr/bin/env bash
# Run inside el-native-dev. The currently configured /build must match the previous campaign.
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/misys/forza_ws/race_stack/install/setup.bash
cd "$(dirname "$0")"
export PYTHONPATH="/home/misys/forza_ws/race_stack/base_system/f110_simulator/f1tenth_gym/gym:$PWD:${PYTHONPATH:-}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
python3 prepare.py > alignment.log
g++ -O2 -std=c++17 -I../../include fit.cpp -o fit
for key in failed passed brake; do
    ./fit "$key.json" "$key-fit.json"
done
python3 - <<'PY'
from pathlib import Path
import json
d=json.loads(Path('brake.json').read_text())
d['physics']=d['physics'][:1691]
d['controls']=[c for c in d['controls'] if c['scan']<1691]
Path('brake-precontact.json').write_text(json.dumps(d))
PY
./fit brake-precontact.json brake-precontact-fit.json
python3 geometry.py > geometry.log
python3 brake_variants.py > brake-variants.log
python3 recorded_beams.py > recorded-beams.log
python3 build.py
for key in failed passed; do python3 render.py "$key" 0 181; done
./replay failed 0 shadow 898 908 915
./replay passed 0 shadow
for step in 898 908 915; do
    for arm in tln teacher one-sync one-raw; do
        ./replay failed 0 "$step" "$arm"
        STUDY_ALTERNATE_TAIL=1 ./replay failed 0 "$step" "$arm"
    done
done
python3 audit.py
python3 figure.py

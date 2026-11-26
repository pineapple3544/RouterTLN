#!/usr/bin/env bash
# RouterTLN installer for a clean misys:forza_full container.
#
#   sudo bash install_final_demo.sh          # (or run as root)
#   ~/run_demo.sh                            # then start the demo, see README [레이스 스택 사용법]
#
# What it does (safe to run again; finished steps are skipped):
#   1. apt packages: build tools, yaml-cpp, libpng, nlohmann-json, Eigen, scipy
#   2. LibTorch 2.9.1 CPU  -> /opt/libtorch-cpu
#   3. RouterTLN code, models and maps/berlin -> /home/misys/RouterTLN
#      (git clone of ROUTERTLN_REPO at ROUTERTLN_REF, or a copy of ROUTERTLN_SRC / the checkout
#      this script sits in)
#   4. /workspace -> /home/misys/RouterTLN and /build -> /home/misys/RouterTLN/build-cpu (symlinks;
#      the scripts and default parameters use these paths)
#   5. CPU-only build (-DEL_CONTROLLER_ONLY=ON: no CUDA needed) + its tests
#   6. one short headless drive, which compiles the simulator's numba kernels into their cache;
#      without it the first demo after install can stop on a sensor timeout while they compile
#   7. ~/run_demo.sh
# Nothing in F1TENTH Gym or race_manager is modified.
set -Eeuo pipefail

ROUTERTLN_REPO=${ROUTERTLN_REPO:-https://github.com/pineapple3544/RouterTLN.git}
ROUTERTLN_REF=${ROUTERTLN_REF:-main}          # pin to the submission commit/tag before handing in
ROUTERTLN_SRC=${ROUTERTLN_SRC:-}              # optional: copy from this local checkout instead
LIBTORCH_URL=${LIBTORCH_URL:-https://download.pytorch.org/libtorch/cpu/libtorch-shared-with-deps-2.9.1%2Bcpu.zip}
LIBTORCH_DIR=/opt/libtorch-cpu
USER_NAME=misys
USER_HOME=/home/$USER_NAME
DEST=$USER_HOME/RouterTLN
BUILD=$DEST/build-cpu
LOG=$USER_HOME/routertln-install.log
RACE_STACK=/home/misys/forza_ws/race_stack

step="starting"
fail() {
  local code=$?
  echo >&2
  echo "[install] FAILED during: $step (exit $code, line ${BASH_LINENO[0]})" >&2
  echo "[install] full log: $LOG" >&2
  exit "$code"
}
trap fail ERR
say() { step=$1; echo; echo "[install] == $1"; }
die() { echo "[install] ERROR: $*" >&2; exit 1; }

# ---- 0. preconditions ----------------------------------------------------------------------
if [[ $EUID -ne 0 ]]; then
  command -v sudo >/dev/null || die "run as root (sudo not available)"
  exec sudo -E bash "$0" "$@"
fi
id "$USER_NAME" >/dev/null 2>&1 || die "user $USER_NAME not found: is this misys:forza_full?"
[[ -f /opt/ros/humble/setup.bash ]] || die "ROS 2 Humble not found: is this misys:forza_full?"
[[ -f $RACE_STACK/install/setup.bash ]] || die "$RACE_STACK not built: is this misys:forza_full?"
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [[ -z $ROUTERTLN_SRC && -f $SCRIPT_DIR/CMakeLists.txt && -f $SCRIPT_DIR/src/node.cpp ]]; then
  ROUTERTLN_SRC=$SCRIPT_DIR
fi
touch "$LOG" && chown "$USER_NAME:$USER_NAME" "$LOG"
exec > >(tee -a "$LOG") 2>&1
echo "[install] $(date -Is) RouterTLN install, log $LOG"

as_user() { sudo -u "$USER_NAME" -H bash -c "$1"; }

# ---- 1. apt --------------------------------------------------------------------------------
say "apt packages"
pkgs=(git curl unzip ca-certificates cmake ninja-build g++-11 libyaml-cpp-dev libpng-dev
      nlohmann-json3-dev libeigen3-dev python3-scipy rsync)
missing=()
for p in "${pkgs[@]}"; do dpkg -s "$p" >/dev/null 2>&1 || missing+=("$p"); done
if ((${#missing[@]})); then
  echo "installing: ${missing[*]}"
  DEBIAN_FRONTEND=noninteractive apt-get update
  DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "${missing[@]}"
else
  echo "all present"
fi

# ---- 2. LibTorch CPU -----------------------------------------------------------------------
say "LibTorch 2.9.1 CPU"
if [[ -f $LIBTORCH_DIR/build-version ]] && grep -q '^2\.9\.1+cpu' "$LIBTORCH_DIR/build-version"; then
  echo "present: $(cat "$LIBTORCH_DIR/build-version")"
else
  tmp=$(mktemp -d)
  curl --fail --location --retry 3 --progress-bar "$LIBTORCH_URL" -o "$tmp/libtorch.zip"
  sha256sum "$tmp/libtorch.zip"
  unzip -q "$tmp/libtorch.zip" -d "$tmp"
  grep -q '^2\.9\.1+cpu' "$tmp/libtorch/build-version" || die "unexpected LibTorch build: $(cat "$tmp/libtorch/build-version")"
  rm -rf "$LIBTORCH_DIR"
  mv "$tmp/libtorch" "$LIBTORCH_DIR"
  rm -rf "$tmp"
  echo "installed: $(cat "$LIBTORCH_DIR/build-version")"
fi

# ---- 3. code, models, map ------------------------------------------------------------------
say "RouterTLN source"
if [[ -n $ROUTERTLN_SRC ]]; then
  [[ -f $ROUTERTLN_SRC/CMakeLists.txt ]] || die "ROUTERTLN_SRC=$ROUTERTLN_SRC is not a RouterTLN checkout"
  if [[ $(realpath "$ROUTERTLN_SRC") != $(realpath -m "$DEST") ]]; then
    echo "copying $ROUTERTLN_SRC -> $DEST"
    mkdir -p "$DEST"
    rsync -a --delete --exclude /runs/ --exclude /build/ --exclude /build-cpu/ --exclude /data/ \
      --exclude '.venv*' --exclude __pycache__ "$ROUTERTLN_SRC/" "$DEST/"
  fi
elif [[ -d $DEST/.git ]]; then
  echo "updating $DEST to $ROUTERTLN_REF"
  as_user "cd '$DEST' && git fetch --tags origin && git checkout --quiet '$ROUTERTLN_REF' && \
           { git symbolic-ref -q HEAD >/dev/null && git pull --ff-only --quiet || true; }"
else
  [[ -e $DEST ]] && die "$DEST exists but is not a git checkout; move it away or set ROUTERTLN_SRC"
  as_user "git clone --quiet '$ROUTERTLN_REPO' '$DEST' && cd '$DEST' && git checkout --quiet '$ROUTERTLN_REF'"
fi
chown -R "$USER_NAME:$USER_NAME" "$DEST"
mkdir -p "$DEST/runs" && chown "$USER_NAME:$USER_NAME" "$DEST/runs"
for f in maps/berlin/berlin.yaml maps/berlin/berlin.png maps/berlin/global_waypoints.json \
         maps/berlin/router.yaml config/final_demo.env scripts/smoke.sh run_demo.sh SUBMISSION_README.md; do
  [[ -f $DEST/$f ]] || die "missing $DEST/$f"
done
while IFS='=' read -r key value; do   # every checkpoint the demo config names must exist
  [[ $key == *CHECKPOINT ]] || continue
  [[ -f ${value/\/workspace/$DEST} ]] || die "missing model $value"
done < "$DEST/config/final_demo.env"
grep -q '^race_checkpoint:' "$DEST/maps/berlin/router.yaml"
[[ -f $(sed -n 's/^race_checkpoint: *//p' "$DEST/maps/berlin/router.yaml" | sed "s#^/workspace#$DEST#") ]] \
  || die "router.yaml race_checkpoint missing"
[[ -d $DEST/.git ]] && echo "commit: $(as_user "git -C '$DEST' rev-parse HEAD")"

# ---- 4. paths ------------------------------------------------------------------------------
say "/workspace and /build links"
link() {  # link <path> <target>: create or confirm a symlink, never replace a real directory
  if [[ -L $1 ]]; then
    [[ $(readlink "$1") == "$2" ]] || ln -sfn "$2" "$1"
  elif [[ -e $1 ]]; then
    die "$1 exists and is not a symlink; move it away (RouterTLN needs $1 -> $2)"
  else
    ln -s "$2" "$1"
  fi
  echo "$1 -> $(readlink "$1")"
}
mkdir -p "$BUILD" && chown "$USER_NAME:$USER_NAME" "$BUILD"
link /workspace "$DEST"
link /build "$BUILD"

# ---- 5. build ------------------------------------------------------------------------------
say "CPU-only build"
as_user "set -eo pipefail; source /opt/ros/humble/setup.bash; source '$RACE_STACK/install/setup.bash'
  cmake -S '$DEST' -B '$BUILD' -G Ninja -DCMAKE_BUILD_TYPE=Release -DEL_CONTROLLER_ONLY=ON \
    -DCMAKE_CXX_COMPILER=/usr/bin/g++-11 -DCMAKE_PREFIX_PATH='$LIBTORCH_DIR'
  cmake --build '$BUILD' -j\$(nproc)
  ctest --test-dir '$BUILD' --output-on-failure"
for b in el_controller el_scenario el_inspect; do [[ -x $BUILD/$b ]] || die "build did not produce $b"; done

# ---- 6. simulator warm-up ------------------------------------------------------------------
say "simulator warm-up (compiles the Gym numba cache, about 30 s)"
gym_cache=$RACE_STACK/base_system/f110_simulator/f1tenth_gym/gym/f110_gym/envs/__pycache__
if compgen -G "$gym_cache/*.nbi" >/dev/null; then
  echo "cache present ($(ls "$gym_cache"/*.nbi | wc -l) kernels)"
else
  # Headless, no obstacles, 25 s. The run itself may stop on a sensor timeout while the
  # kernels compile; only the cache matters here.
  as_user "cd '$DEST' && EL_INFERENCE_DEVICE=cpu EL_STOP_AFTER_LAPS=0 \
    bash scripts/smoke.sh 25 0 moe install-warmup '' 1 > runs/install-warmup.out 2>&1" || true
  compgen -G "$gym_cache/*.nbi" >/dev/null || die "simulator warm-up did not create $gym_cache/*.nbi (see $DEST/runs/install-warmup.out)"
  echo "cache created ($(ls "$gym_cache"/*.nbi | wc -l) kernels)"
fi

# ---- 7. demo launcher ----------------------------------------------------------------------
say "demo launcher"
install -m 0755 -o "$USER_NAME" -g "$USER_NAME" "$DEST/run_demo.sh" "$USER_HOME/run_demo.sh"
echo "$USER_HOME/run_demo.sh"
install -m 0644 -o "$USER_NAME" -g "$USER_NAME" "$DEST/SUBMISSION_README.md" "$USER_HOME/README.md"
echo "$USER_HOME/README.md"

step="done"
echo
echo "[install] OK. Start the demo as $USER_NAME:"
echo "    ~/run_demo.sh 1      # 1 obstacle per lap (also: 3, 5)"

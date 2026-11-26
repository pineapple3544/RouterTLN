#!/usr/bin/env bash
# Build the patched controller into a separate dir; /build (used by the matrix) stays untouched.
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/misys/forza_ws/race_stack/install/setup.bash
cmake -S /workspace -B /opt/build-locfix -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=/usr/bin/g++-11 \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda-12.8/bin/nvcc -DCMAKE_CUDA_HOST_COMPILER=/usr/bin/g++-11 \
  -DCMAKE_CUDA_ARCHITECTURES=75 -DCMAKE_PREFIX_PATH=/opt/libtorch > /opt/build-locfix.configure.log
cmake --build /opt/build-locfix --target el_controller el_scan_localizer_test -j4
/opt/build-locfix/el_scan_localizer_test /workspace/maps/berlin/berlin.yaml /workspace/maps/berlin/global_waypoints.json
ls -la /opt/build-locfix/el_controller

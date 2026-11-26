#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/misys/forza_ws/race_stack/install/setup.bash
set -u
cmake -S /workspace -B /build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=/usr/bin/g++-11 \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda-12.8/bin/nvcc \
  -DCMAKE_CUDA_HOST_COMPILER=/usr/bin/g++-11 \
  -DROUTERTLN_INFERENCE_ONLY=OFF -DCMAKE_CUDA_ARCHITECTURES="${EL_CUDA_ARCHITECTURES:-86;120}" -DCMAKE_PREFIX_PATH=/opt/libtorch
cmake --build /build -j2
ctest --test-dir /build --output-on-failure

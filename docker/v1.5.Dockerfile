FROM ros:humble-ros-base-jammy@sha256:a18afd4a42ad603579d03b9012385dfeaf2e848423f0a6c41699766e7849c86c
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    cmake ninja-build g++ curl unzip libpng-dev libyaml-cpp-dev nlohmann-json3-dev \
    python3-dev python3-pip python3-pil python3-yaml libgl1 libglu1-mesa libegl1 libegl-mesa0 \
    ros-humble-ackermann-msgs ros-humble-tf2-ros && rm -rf /var/lib/apt/lists/*
RUN curl -fLsS --retry 3 'https://download.pytorch.org/libtorch/cpu/libtorch-shared-with-deps-2.9.1%2Bcpu.zip' -o /tmp/libtorch.zip \
    && echo '9d0cf62db9b72de5b27071a945d6de4f340bffd996c3dd770eeb3f8237d05e75  /tmp/libtorch.zip' | sha256sum -c - \
    && unzip -q /tmp/libtorch.zip -d /opt && rm /tmp/libtorch.zip
RUN python3 -m pip install --no-cache-dir numpy==1.24.4 scipy==1.15.3 gymnasium==0.29.1 \
    numba==0.60.0 llvmlite==0.43.0 pyglet==1.5.20 transforms3d==0.4.2
ENV EL_SIM_WARMUP=1 EL_SIM_CONFIG_DIR=/opt/berlin-sim/config \
    PYTHONPATH=/opt/berlin-sim:/workspace/src \
    OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYGLET_HEADLESS=true
WORKDIR /workspace
COPY docker/simulator /opt/berlin-sim
COPY CMakeLists.txt ./
COPY include include
COPY src src
COPY tools/replay_corridor.cpp tools/
COPY tests/scan_localizer_test.cpp tests/
COPY tests/fixtures/corridor tests/fixtures/corridor
COPY maps/berlin maps/berlin
RUN source /opt/ros/humble/setup.bash && cmake -S . -B /build -G Ninja \
    -DROUTERTLN_INFERENCE_ONLY=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/libtorch \
    && cmake --build /build -j2 && ctest --test-dir /build --output-on-failure
COPY models/v1.5 models/v1.5
RUN cd models/v1.5 && sha256sum -c SHA256SUMS
COPY config config
COPY scripts scripts
COPY experiments/berlin-endurance/v1.5/cases.json experiments/berlin-endurance/v1.5/ros-cases.json experiments/berlin-endurance/v1.5/suites.json experiments/berlin-endurance/v1.5/replays.json /opt/berlin-cases/
COPY experiments/berlin-endurance/v1.5/reliability/paired300/manifest.json /opt/berlin-cases/reliability/paired300.json
COPY experiments/berlin-endurance/v1.5/reliability/fresh600/manifest.json /opt/berlin-cases/reliability/fresh600.json
ENTRYPOINT ["/ros_entrypoint.sh", "python3", "/workspace/scripts/berlin_v15.py"]
CMD ["--help"]

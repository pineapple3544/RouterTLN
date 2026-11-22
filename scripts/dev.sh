#!/usr/bin/env bash
# Run from a persistent WSL Ubuntu terminal. No training is started.
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
case "${1:-status}" in
  image)
    docker build -t el-native:cuda12.8 -f "$root/docker/Dockerfile" "$root/docker"
    ;;
  start)
    if docker container inspect el-native-dev >/dev/null 2>&1; then
      docker start el-native-dev
    else
      docker run -d --name el-native-dev --gpus all --shm-size=2g \
        --mount "type=bind,source=$root,target=/workspace" \
        --mount type=volume,source=el-native-build,target=/build el-native:cuda12.8
    fi
    ;;
  build) docker exec el-native-dev bash /workspace/scripts/build.sh ;;
  shell) docker exec -it el-native-dev bash ;;
  stop) docker stop el-native-dev ;;
  status) docker ps -a --filter name=el-native-dev; docker image inspect el-native:cuda12.8 --format '{{.Id}}' ;;
  *) echo 'Usage: dev.sh image|start|build|shell|stop|status'; exit 2 ;;
esac

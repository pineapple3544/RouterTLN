# Berlin 공통 속도 범위와 첫 랩 전체 경로 계획

성공한 첫 랩에서 LiDAR로 추정한 점유 격자와 자차 위치를 사용해, 트랙 각 지점에서 차체가 들어갈 수 있는 횡방향 후보를 저장한다. 첫 랩이 끝나면 한 바퀴의 후보들을 출발점으로 되돌아오는 경로로 연결한다. 통로 폭, 횡이동량, 첫 랩 궤적과의 편차를 비용에 반영한다. 좁은 코너는 첫 랩에서 실제로 지나간 위치 가까이에서 측정한 차체 방향으로 통과 가능 여부를 다시 확인한다. 경로를 빈 공간 안에서 평활화하고 곡률과 다음 감속 지점까지의 거리를 반영해 한 바퀴의 속도 상한을 만든다.

Avoid의 로컬 탐색기는 이 경로를 참고해 현재 스캔으로 조향을 검증한다. Race TLN은 기존 방식으로 조향하며 두 모드 모두 전체 경로의 속도 계획을 사용한다. 경로를 연결할 수 없으면 기존 로컬 유도로 주행한다. ROS 제어 CSV의 `whole_lap_path_ready`와 오프라인 결과의 `whole_lap_path_status`로 실제 생성 여부를 확인할 수 있다.

후속 랩의 정상 목표는 두 모드 모두 2.7~4.5m/s다. 첫 랩은 2.2m/s 상한을 적용한다. 센서 보호 150ms, 제동 보호, 정지 명령, Race·Avoid 체크포인트는 기존과 같다. Race 제동 대안은 정상 목표 하한 아래로도 감속할 수 있다.

## 180초 검증

| 오프라인 조건 | 기존 v1.5 | 출구 감속 완화만 | 공통 범위만¹ | 공통 범위와 전체 경로¹ |
|---|---:|---:|---:|---:|
| 기존 24배치 완주 | 24/24 | 24/24 | 23/24 | 24/24 |
| 24배치 평균 최고 랩 | 16.850초 | 16.852초 | 23개 공통 완주에서 16.526초 | 16.687초 |
| 사전 추첨 30배치 완주 | 28/30 | 28/30 | 미측정 | 29/30 |

¹ 출구 감속 완화도 켰다. 공통 범위만 사용했을 때 `holdout-5-01`이 실패했다. 30배치 중 양쪽 모두 완주한 28개 평균은 기존 16.950초, 전체 경로 후보 16.833초였다. `pace-3-081`은 새로 완주했고 `pace-3-023`은 양쪽 모두 실패했다. 30배치는 앞선 실험에서 이미 사용한 배치다.

기존 24배치와 사전 추첨 30배치에서 모두 전체 경로가 생성됐고 물리 충돌은 0건이었다. 24배치에서 최소 장애물 여유는 12.37cm에서 10.98cm로, Avoid TLN 직접 채택률은 88.14%에서 87.17%로 변했다. 24배치의 동일 위치 비교에서 장애물 통과 후 0.5~3m 구간은 평균 0.013초, 통과 후 0.5m부터 다음 장애물까지는 0.107초 줄었다. 다음 장애물이 최소 5.5m 떨어진 완전한 구간만 사용하고 각 배치 내 평균을 같은 비중으로 집계했다.

| ROS 장애물 | 노이즈 시드 | 기존 최고 랩 | 전체 경로 최고 랩 | 연속 랩 | 180초 무정지·무충돌 |
|---|---:|---:|---:|---:|---|
| 3 | 170826456 | 16.626초 | 16.708초 | 10 | 통과 |
| 5 | 402121338 | 19.071초 | 18.625초 | 9 | 통과 |
| 1 | 426691458 | 15.304초 | 15.404초 | 11 | 통과 |
| 3 | 170826557 | 16.544초 | 16.687초 | 10 | 통과 |

ROS 네 번에서 25ms 초과는 0회였다. 오프라인 24배치에서는 6회였다. 장애물 5개 ROS 주행은 0.446초 빨라졌고, 나머지 3회는 0.082~0.142초 느려졌다. 직접 채택률 목표 87.94%와 전 배치 무정지를 충족할 때까지 실험 설정으로 유지한다.

## 실행

기존 Berlin TLN 우선 프로필에서 `EL_SHARED_SPEED_BAND=true`, `EL_WHOLE_LAP_PATH=true`, `EL_AVOID_EXIT_PACE=true`를 함께 지정한다. `EL_SPEED_CAP`은 두 모드의 공통 상한, `EL_SIM_AVOID_MIN`은 공통 하한이다. 아래 예시는 개발 컨테이너의 `/workspace`에서 실행한다. 장애물 인덱스와 오프셋은 검증용 시나리오 노드에만 전달된다.

```bash
export EL_MAP_DIR=/workspace/maps/berlin EL_START_INDEX=37
export EL_RACE_DURATION_S=180 EL_STOP_AFTER_LAPS=0 EL_LAP_PLANNING=true
export EL_SHARED_SPEED_BAND=true EL_WHOLE_LAP_PATH=true EL_AVOID_EXIT_PACE=true
export EL_SIM_RACE_ONLY=true EL_SIM_SECTION_SPEED_PROFILE=true EL_SPEED_CAP=4.5
export EL_SIM_AVOID_MIN=2.7 EL_SIM_AVOID_MAX=4.5 EL_STRAIGHT_BOOST=true
export EL_AVOID_POLICY_FIRST=true EL_AVOID_CORRIDOR_STEER_SEED=true EL_RACE_VERIFIED_PACE=false
export EL_INFERENCE_DEVICE=cpu OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export EL_AVOID_CHECKPOINT=/workspace/runs/split-race-avoid-mixed-speed-v1/avoid.pt
export EL_RACE_CHECKPOINT=/workspace/runs/v2-highspeed-expert-wsp-20261124/race-bend-highspeed.pt
export EL_OBSTACLE_INDICES='[86,127,340,413,447]' EL_OBSTACLE_INDEX=86
export EL_OBSTACLE_OFFSETS='[-0.0866, -0.0246, -0.1591, -0.16, -0.056]'
export EL_SCAN_NOISE_SEED=402121338
bash scripts/smoke.sh 186 5 moe "shared-scout-$(date +%s)" '' 402121338
```

WSL 원기록은 `/home/nyko2/autodrive/el-project/native/runs/berlin-shared-scout-20261125`에 있다. `verified-result.json`, `holdout-result.json`, `ros-pairs.json`에 개별 결과와 비교값을 남겼다. `pace-3-023-route.json`에는 실패 배치의 전체 경로와 지점별 속도 계획이 있다.

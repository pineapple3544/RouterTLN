# Berlin 정방향 Race 속도 제어 실험

개발 목표는 장애물 1개 배치 `new-1-051`에서 첫 명령부터 180초 동안 정지·접촉·보호 거부 없이 주행하고, 같은 run에서 베스트랩 12.999초 이하를 기록하는 것이다. 제어 주기는 25ms이고 센서 타임아웃은 150ms다. 첫 랩은 2.2m/s로 정찰하고 성공한 뒤에만 새 속도 제어를 켠다. 원본 Avoid와 2.7–3.0m/s 계획 범위, Race WSP 모델과 조향은 기존 설정을 따른다. 지도와 센서 추정치로 제어하며 물리 시뮬레이터의 정답 위치는 독립 채점에만 쓴다.

## 구현

- `race_sight_speed`는 전방 ±0.12rad의 유효 LiDAR 거리에서 차체 앞 여유 0.35m를 뺀다. 0.20초의 지연과 6m/s²의 제동 가정으로 정지 가능한 속도를 계산한다. 전방 중앙이 가려졌거나 유효한 각도 범위가 부족하면 추가 가속을 허용하지 않는다.
- `race_exit_speed`는 전방 곡률과 좌우 차폭 여유, 실제·요청 조향 차이 및 차체 방향 오차로 속도를 제한한다. 재가속은 제어 주기당 4m/s²로 제한하고 기존 0.18초 제동 예측 후 높은 출구 속도에는 0.24초 예측을 추가한다. 이 상수들은 로컬 시뮬레이터에서 시험한 값이다. 실제 감속 보증값으로 사용하지 않는다.
- 두 설정의 기본값은 `false`다. 새 설정은 Race-only 시뮬레이터 프로파일에서만 활성화할 수 있다. 오프라인 평가기와 ROS는 같은 C++ 계산을 사용한다. `curve_cap`, `sight_cap`, `steering_cap`과 기존 속도·보호 로그로 속도를 제한한 단계를 추적할 수 있다.

## 같은 상태에서의 재생

기존 4.0·4.25·4.5m/s 주행에서 첫 명령 차이는 첫 랩을 마친 뒤 23.400초, 누적 48.845m에서 나왔다. 4.25m/s 설정에서는 둘째 랩의 굽은 구간에서 벽 접촉이, 4.5m/s에서는 넷째 랩의 같은 굽은 구간에서 제동 보호 거부가 발생했다. 직선 시야가 충분해도 커브 진입과 조향 반전 직후에 문제가 생겼다.

실패 전 누적 81m와 178m 부근에 각각 상태를 저장했다. 기존 설정을 복원하면 4.25에서 충돌, 4.5에서 정지가 재현됐다. 후보 상한 5.25에서 시야 제한만 켜면 두 상태 모두 정지했다. 출구 제어만 켜면 두 상태가 모두 130제어주기(3.25초)를 통과했다. 두 기능을 함께 켠 경우 각 상태에서 원래 진입, 방향 +0.02rad와 속도 +0.1m/s, 방향 −0.02rad와 속도 −0.1m/s를 적용한 6개 분기가 모두 통과했다. 같은 개발 배치를 처음부터 180초 주행하면 시야 단독은 1랩 뒤 정지했고 출구 단독은 8랩 뒤 벽과 충돌했다.

## 오프라인 180초

각 설정·배치마다 계획을 초기화했다. 셀에는 판정, 종료 전 완주 랩 수, 종료 전 최단 랩(초)을 표시한다. 정지한 run의 최단 랩은 목표 달성 기록에서 제외한다.

| 배치 | 기준 4.0 | 후보 5.25 | 5.25 최초 보호 사유 |
|---|---|---|---|
| new-1-051 | 통과 · 12랩 · 13.708 | 통과 · 12랩 · 13.387 | 없음 |
| new-3-028 | 통과 · 11랩 · 14.780 | 정지 · 1랩 · 25.959 | dynamic_braking_footprint |
| new-5-024 | 정지 · 1랩 · 28.033 | 정지 · 3랩 · 15.518 | braking_footprint |
| s43939214-n1 | 통과 · 11랩 · 15.125 | 통과 · 11랩 · 14.210 | 없음 |
| s49140402-n3 | 통과 · 11랩 · 15.061 | 정지 · 3랩 · 14.342 | braking_footprint |
| s66874438-n5 | 통과 · 9랩 · 18.288 | 통과 · 9랩 · 18.043 | 없음 |

개발 배치의 두 기능 조합은 13.387초였다. 별도로 조향 지연 중 차체가 지나갈 영역을 예측하는 5.65m/s 후보는 오프라인에서 12랩·13.093초까지 단축했지만 ROS에서는 1랩 뒤 `braking_footprint`로 정지했다. 이 후보는 실행 설정에서 제외했다. 5.25m/s 후보도 기존에 성공한 장애물 3개 배치 두 곳에서 새로 정지했다.

## ROS 180초

장애물 배치와 스캔 노이즈 시드를 고정하고 매번 첫 랩부터 새로 시작했다. 베스트랩과 2랩 이후 중앙값은 독립 물리 로그에서 결승선 통과 시각을 보간해 계산했다. 각 주행의 원시 기록과 점수 파일은 표의 태그로 찾을 수 있다. 계산 시간은 제어 로그의 벽시계 값이다.

| 설정 | 원시 태그 | 180초 판정 | 연속 랩 | 베스트 (초) | 2랩 이후 중앙값 (초) | 충돌 / 정지 샘플 | 보호 사유 | 최대 제어 (ms) |
|---|---|---|---:|---:|---:|---:|---|---:|
| 기준 4.0 | `new-1-051-4.0` | 통과 | 12 | 13.717 | 14.162 | 0 / 0 | 없음 | 11.72 |
| 초기 시야·출구 4.5 | `ros-v2` | 통과 | 12 | 13.749 | 13.831 | 0 / 0 | 없음 | 12.42 |
| 5.25 개발 빌드 | `ros-final` | 통과 | 12 | 13.547 | 13.608 | 0 / 0 | 없음 | 12.73 |
| 5.25 최종 빌드 1차 | `ros-production` | 실패 | 1 | 23.384 | — | 0 / 15536 | dynamic_braking_footprint | 10.80 |
| 5.25 최종 빌드 2차 | `ros-production-repeat` | 통과 | 12 | 13.706 | 13.765 | 0 / 0 | 없음 | 10.70 |
| 4.75 최종 빌드 | `ros-production-475` | 통과 | 12 | 13.714 | 13.803 | 0 / 0 | 없음 | 11.31 |
| 차체 예측 5.65 | `ros-g` | 실패 | 1 | 23.412 | — | 0 / 15554 | braking_footprint | 9.96 |

5.25m/s 개발 빌드의 13.547초 랩에서 Race는 11.117초, Avoid는 2.430초였다. 기준 13.717초 랩의 Race 11.317초, Avoid 2.400초와 비교하면 단축분은 Race에서 나왔다. 최종 소스의 같은 상한·같은 노이즈 시드 재시작에서는 둘째 랩 누적 51.58m에서 동적 제동 거부도 발생했다. 개발 성공 주행은 누적 51.2m에서 추정 3.27m/s, 명령 2.47m/s였고 실패 주행은 같은 거리에서 3.90m/s, 명령 3.77m/s였다. 같은 시드를 사용해도 ROS 제어 시점과 진입 상태가 달라질 수 있다.

12초대와 180초 안전을 함께 달성한 ROS 기록은 0건이다. 목표 달성 이후로 예정된 서로 다른 노이즈 시드 3회, 미사용 30개 및 기존 300개 평가는 진행하지 않았다. 기본 속도 설정은 기존 값을 사용한다.

## 재현

로컬 ROS Humble·F1TENTH Gym 컨테이너의 `/workspace`에서 실행한다. [평가기 입력](lap12-envelope-cases.json)에는 상한 5.25m/s의 여섯 배치가 들어 있다. 프로젝트의 일반 오프라인 평가와 같은 Gym Python 환경을 사용한다.

```bash
export PYTHONPATH="/home/misys/forza_ws/race_stack/base_system/f110_simulator/f1tenth_gym/gym:${PYTHONPATH:-}"
/build/el_eval_avoid models/avoid-corridor-v2.pt runs/lap12-replay.jsonl \
  experiments/berlin-endurance/lap12-envelope-cases.json models/race-berlin-wsp.pt
```

다음 명령은 개발 배치 한 곳의 ROS 실험이다. 장애물 인덱스와 오프셋은 시나리오 생성기에만 전달한다. 새 태그를 사용하면 이전 로그를 보존할 수 있다.

```bash
unset EL_UNIFIED_CHECKPOINT
export EL_MAP_DIR=/workspace/maps/berlin EL_START_INDEX=37 EL_OBSTACLE_INDEX=289
export EL_OBSTACLE_STRIDE=1 EL_OBSTACLE_OFFSETS='[0.1739]' EL_SCAN_NOISE_SEED=934488799
export EL_LAP_PLANNING=true EL_SIM_SECTION_SPEED_PROFILE=true EL_SIM_RACE_ONLY=true
export EL_SPEED_CAP=5.25 EL_RACE_SIGHT_SPEED=true EL_RACE_EXIT_SPEED=true EL_STRAIGHT_BOOST=false
export EL_SIM_AVOID_MIN=2.7 EL_SIM_AVOID_MAX=3.0 EL_RACE_BRAKE_PREVIEW_S=0.18
export EL_INFERENCE_DEVICE=cpu EL_RACE_DURATION_S=180 EL_STOP_AFTER_LAPS=0
export EL_AVOID_CHECKPOINT=/workspace/models/avoid-corridor-v2.pt
export EL_RACE_CHECKPOINT=/workspace/models/race-berlin-wsp.pt
bash scripts/smoke.sh 186 1 moe lap12-replay '' 934488799
```

원시 C++ 평가 결과·상태 스냅샷·ROS 로그는 로컬 `el-project/native/runs/berlin-12s-implementation/`에 있다. 기존 4.0m/s ROS 기록은 `el-project/native/runs/berlin-12s-feasibility/`에 있다. 기준 설정에서 수정 전후 오프라인 7,200주기의 명령, 차량 상태, 모드와 시각이 모두 일치했다. CTest 13개와 독립 채점기 테스트 6개가 통과했다.

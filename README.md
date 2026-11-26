# RouterTLN / v3 Scenario-Residual (experimental)

**매 랩 장애물이 재배치되는 자율주행 레이싱에서, 장애물을 발견하기 전에 대표적인 회피 경로를 준비하고 발견 이후 경량 제어기로 실행할 수 있는가?**

기존 v1.5의 RaceTLN을 유지하고 Avoid만 새로운 GPU 시나리오 경로 계획과 CPU 잔차 제어 구조로 대체한다. 최고 속도 초기값은 3.5 m/s이며, 목표는 불필요한 감속 및 온라인 경로 생성 비용 감소다. 이전의 고정 장애물 배치 기반 랩 계획은 새 모드에서 사용하지 않는다.

> **상태: 실험용 초기 구현.** CUDA/ROS 전체 빌드, 실제 race_manager 연속 주행 및 RTX 3080 실측은 아직 수행하지 않았다. RCN 학습 가중치는 제공하지 않는다. 신경망 없이 작동하는 기본 제어기부터 검증한다. 아래 수치들은 설계 초기값이며 시험 결과가 아니다.

- 기반: [v1.5 신뢰성 평가 및 기존 실행 방법](https://github.com/pineapple3544/RouterTLN/tree/v1.5)
- 기존 v1.5의 평가 수치는 고정 배치 결과이며 v3 또는 매 랩 재배치 시험의 결과로 간주하지 않는다.
- 기존 mode=moe 및 v1.5 CPU Docker Compose는 유지한다. 새 mode=scenario는 CUDA 개발 이미지가 필요하다.
- 수업 매뉴얼 ADP_Project_Manual.pdf 15–16쪽: race_manager는 랩마다 정적 장애물을 다시 배치한다. 실제 배치기 소스의 세부 규격/통과 보장은 아직 확인되지 않았다.

## 구조

    LiDAR + 정적 지도 + scan-only localization
                      |
      관측 Free / Occupied / Unknown + 트랙 폭
                      |
          GPU 작업 스레드 (비동기 / 약 10 Hz 요청)
          + 실제 관측 기반 최우선 경로
          + 미관측 구간의 가상 장애물 시나리오
          + 비용 기반 후보 선택 및 속도 계획
                      |
      최근 시나리오 캐시 / 현재 점유 지도 재검증
                 /              \
         Race (clear)          Avoid (obstacle)
         기존 RaceTLN           경로 추종 제어기
         CPU 40 Hz             CPU 40 Hz
                                + RCN 잔차 보정
                 \              /
          기존 독립 Safety Guard / /drive

RaceTLN은 자신이 학습한 기준 레이싱 라인에서만 사용한다. 현재 위치/방향이 기준선에 충분히 가깝고 전방 기준 경로의 차체 여유와 관측 거리 조건을 만족한 상태가 0.20초 유지되면 Race에 진입한다. 조건을 잃으면 Avoid로 전환한다. Race는 GPU 결과를 기다리지 않고, 기준선 검사 및 안전 속도 제한을 받는다. RaceTLN을 임의의 회피 경로 추종기로 사용하지 않는다.

Avoid는 곡률 feedforward + 횡방향/방향 오차 feedback을 사용한다. RCN은 기본 명령과 실제 LiDAR/경로/차량 상태의 관계를 학습한다. 예측 잔차를 적용한 명령은 다시 기존 제동 및 차체 검사에 통과해야 한다. 적합한 경로가 없으면 정지 명령을 내리고, 안전한 제동 명령도 찾을 수 없으면 기존 보호 정지를 유지한다.

## 시나리오 및 GPU 최적화

| 항목 | 초기 설정 |
|---|---|
| 목표 속도 상한 | 3.5 m/s |
| 공간 예측 범위 | 6 m / 49점 / 기준 0.125 m |
| 경로 후보 | 4개 횡방향 제어점 × 각 5단계 = 625개 조합 |
| 경로 구성 | cubic Hermite 횡방향 곡선 |
| 기본 장애물 | 한 변 0.15 m의 정사각형을 둘러싼 원 |
| 추가 크기 | 0.10 / 0.20 / 0.30 m |
| 초기 횡방향 샘플 간격 | 0.20 m |
| 추가 시나리오 | 제한된 두 장애물 조합 / 좁은 구간의 세분화 |
| GPU 요청 주기 / 추가 계산 예산 | 0.10 s / 55 ms (soft budget) |
| 캐시 | 최근 최대 48개 가상 후보 / 초기 유효 시간 0.55 s |
| CPU 제어 및 deadline | 25 ms / 40 Hz |

지도 폭은 기준 웨이포인트의 좌우 정적 벽까지 탐색해 자동 계산한다. 장애물 크기를 고려해 물리적으로 맵 내부에 놓을 수 있는 위치만 후보로 생성한다. 가상 장애물을 실제 점유 지도에 삽입하지 않는다. 실제 관측과 가까운 가상 배치만 실행 후보로 사용하며 현재 차량 상태와 최신 점유 지도를 다시 검사한다. 아직 관측하지 못한 공간은 자유 공간으로 확정하지 않는다. 관측 상태는 LiDAR 광선과 최근 관측 정보로 추정한다.

배치기가 반드시 통과 가능성을 보장한다는 가정은 기본적으로 비활성화한다. 원본 규격이 확인된 경우에만 scenario_assume_passable:=true로 단순 통과 폭 필터를 실험할 수 있다. 이 옵션도 동역학적 통과 가능성을 보장하지는 않는다.

CUDA는 각 가상 배치의 경로 후보를 병렬 평가한다. 충돌, 최대 곡률 및 조향/가감속 제한을 확인하고 예상 통과 시간과 안전 여유 및 조향 변화 비용을 사용한다. 인코스 선호는 별도 고정 좌표가 아니라 거리/속도/곡률을 통해 간접 반영한다. 비용 함수 및 탐색 방법은 아직 튜닝하지 않은 초기 유한 후보 탐색이며 전역 최적해를 보장하지 않는다.

실제 관측 경로 계산은 항상 먼저 처리한다. 추가 시나리오는 작업 시간이 남았을 때 소규모 배치로 처리하고 신규 요청이 들어오면 다음 배치부터 양보한다. CUDA 동기화는 GPU 작업 스레드에서만 수행한다. CPU는 점유 스냅샷 복사, 후보 검증, 제어 명령 생성, 최종 안전 검사를 수행한다. GPU 배치 자체는 강제 선점하지 않으므로 soft budget을 넘을 수 있다.

랩 경계는 센서 기반 위치 추정과 기준 경로 진행도만으로 검출하고 동적 점유/경로/시나리오 캐시를 무효화한다. 시뮬레이터 장애물 토픽, GT odom, 숨겨진 seed는 제어기에 전달하지 않는다. 실제 배치 시점과 위치 추정 랩 경계의 일치 여부는 별도 검증이 필요하다.

## 빌드 / 실행

기존 강사 제공 misys:forza_full 이미지가 있는 PC에서 CUDA 개발 이미지를 사용한다. CUDA 12.8 / LibTorch cu128이며 RTX 3080 (sm_86), RTX 5070 Ti (sm_120)를 빌드 대상으로 한다. 호스트 NVIDIA 드라이버와 Docker GPU 접근은 별도로 준비한다.

~~~bash
git clone --branch v3-scenario-residual --single-branch https://github.com/pineapple3544/RouterTLN.git
cd RouterTLN
bash scripts/dev.sh image
docker run -d --name routertln-v3-dev --gpus all --shm-size=2g \
  --mount "type=bind,source=$PWD,target=/workspace" \
  --mount type=volume,source=routertln-v3-build,target=/build \
  el-native:cuda12.8
docker exec routertln-v3-dev bash /workspace/scripts/build.sh
docker exec -it routertln-v3-dev bash
~~~

기존 v1.5의 CPU 전용 이미지에서는 CUDA scenario 모드를 실행할 수 없다. 빌드 스크립트는 기본적으로 두 아키텍처를 모두 포함한다. 필요하면 EL_CUDA_ARCHITECTURES=86 또는 120으로 빌드 대상을 선택할 수 있다.

아래 모든 터미널에서 동일한 컨테이너/ROS domain을 사용하고 다음 환경을 소싱한다.

~~~bash
source /opt/ros/humble/setup.bash
source /home/misys/forza_ws/race_stack/install/setup.bash
cd /workspace
~~~

### 원본 시뮬레이터 / race_manager

터미널 1 (로컬 수정 브리지 미사용):

~~~bash
EL_LOCAL_STEADY_SIM=0 EL_MAP_DIR=/workspace/maps/berlin bash scripts/run_sim.sh
~~~

터미널 2 (강사 제공 장애물 배치기):

~~~bash
cfg="$(ros2 pkg prefix --share race_manager)/config/race_manager_params.yaml"
ros2 run race_manager obstacle_spawner --ros-args --params-file "$cfg" \
  -p waypoints_file:=/workspace/maps/berlin/global_waypoints.json \
  -p seed:=2026 -p num_obstacles:=3
~~~

터미널 3 (강사 제공 점수 기록기):

~~~bash
cfg="$(ros2 pkg prefix --share race_manager)/config/race_manager_params.yaml"
ros2 run race_manager scorer --ros-args --params-file "$cfg" \
  -p waypoints_file:=/workspace/maps/berlin/global_waypoints.json \
  -p map_name:=berlin -p seed:=2026 -p log_dir:=/workspace/runs/v3-official
~~~

터미널 4 (제어기):

~~~bash
/build/el_controller --ros-args \
  -p mode:=scenario -p inference_device:=cpu \
  -p map_dir:=/workspace/maps/berlin \
  -p race_checkpoint:=/workspace/models/v1.5/race.pt \
  -p max_speed:=3.5 -p deadline_ms:=25.0 \
  -p metrics:=/workspace/runs/v3-control.csv
~~~

RaceTLN은 기본 활성화한다. 순수 로보틱스 비교군은 -p scenario_race_tln:=false를 추가한다. RCN은 학습된 파일을 명시하지 않는 한 비활성화되며, 보정량은 0이다. 공식 평가 시에는 원본 채점기의 180초 / 최대 10랩 조건에 맞춰 종료하고 채점기 로그로 판정한다. 위 명령의 seed는 개발 예시이며 시험 seed와 무관하다. 원본 race_manager의 설치 상태와 파라미터가 실제 이미지에서 일치하는지 확인해야 한다. 기존 고정 배치 smoke.sh와 혼용하지 않는다.

| 옵션 | 용도 |
|---|---|
| scenario_period_s | 경로 요청 간격 |
| scenario_optional_budget_ms | 추가 가상 시나리오 작업 예산 (0이면 실제 관측 경로만) |
| scenario_max_age_s | 후보 유효 시간 |
| scenario_nominal_size_m | 기본 가상 장애물 크기 |
| scenario_position_step_m | 초기 위치 샘플 간격 |
| scenario_race_tln | 기존 RaceTLN 활성화 |
| residual_checkpoint | 학습 완료된 RCN1 가중치 경로 |
| scenario_record | 별도 학습 기록 JSONL 경로 |
| scenario_run_id | 학습용 실행 식별자 |

## 잔차 신경망 / 학습

새 모델 RCN은 CPU에서 64→48→32→2 MLP로 추론한다. 입력은 LiDAR 구간별 거리/유효율, 차량 상태, 기본 조향/속도, 참조 경로 추종 오차/곡률, 가상 장애물과 관측된 장애물의 위치/가시 폭·길이 차이 및 좌우 여유다. 관측된 폭/길이는 LiDAR에 보이는 부분의 추정치이며 실제 전체 크기를 보장하지 않는다.

조향 보정 범위 ±0.08 rad / 속도 보정 범위 ±0.5 m/s. 초기 구현에서는 최종 속도를 기본 안전 상한보다 높이지 못하게 제한한다. 별도의 안정성 인증은 없다.

1. 장애물 재배치 환경에서 신경망 없이 여러 run을 수집한다. 각 run의 식별자를 바꾼다.

~~~bash
# 위 제어기 실행 인자에 추가:
-p scenario_record:=/workspace/runs/train-001.jsonl \
-p scenario_run_id:=train-001
~~~

2. 기존 CUDA MPPI로 별도 학습 라벨을 생성한다.

~~~bash
/build/el_scenario_teacher /workspace/runs/train-001.jsonl /workspace/runs/labels-001.jsonl
~~~

3. 서로 다른 3개 이상의 run에서 라벨을 확보한 뒤 PyTorch 환경에서 훈련한다. run 단위로 학습/검증을 분리하며 보정량을 학습한다.

~~~bash
python3 tools/train_scenario_residual.py /workspace/runs/labels-*.jsonl \
  --out /workspace/models/rcn-v3.txt
~~~

4. 기존 실행 인자에 -p residual_checkpoint:=/workspace/models/rcn-v3.txt를 추가한다.

학습은 오프라인에서만 수행한다. 교사의 로컬 격자/동역학 근사와 실제 Safety Guard는 다르므로, 가중치 검증 손실만으로 주행 적합성을 판정하지 않는다. 수집 모드의 추가 격자 계산/파일 기록은 주행 지연을 유발할 수 있어 공식 성능 측정에서는 끈다.

## 검증 계획 및 지표

| 비교군 | 설정 |
|---|---|
| 기준 | v1.5, 원본 재배치 환경에서 재측정 |
| A | scenario, RaceTLN off / RCN off |
| B | scenario, RaceTLN on / RCN off |
| C | B + RCN |
| D | B/C에서 scenario_optional_budget_ms=0 |

원본 race_manager 동일 seed 순서 / 장애물 1·3·5개 / 180초 또는 최대 10랩으로 비교한다. 성공 랩 수, best lap, 충돌과 보호 정지, CPU 제어 지연 P99/P99.9/max, 25 ms 초과 횟수, GPU 경로 계산 지연, 캐시 선택 비율 및 Race/Avoid 전환을 기록한다. 실제 시험 PC에서 동일한 안전 기준과 알고리즘을 사용한다.

기존 metrics CSV 및 새 metrics.scenario.csv에 epoch, generation, 후보 수, 경로 나이/계산 시간, 가시 거리, 기본/보정/최종 제어 명령, 정지 원인을 기록한다.

~~~bash
ctest --test-dir /build --output-on-failure -R 'scenario_geometry|scenario_cuda_parity'
~~~

새 회귀 테스트는 합성 지도에서 경로 생성/충돌 검사/속도 계획과 CPU/GPU 평가 일치도를 검사한다. 아직 이 브랜치의 CUDA/ROS 테스트 실행 결과가 없으며, 첫 주행 전 위 테스트와 전체 빌드를 수행해야 한다. 성능 목표는 개발 PC 최대 15 ms, 시험 PC 25 ms 미만이며 아직 실측으로 확인되지 않았다.

## 파일

| 파일 | 담당 |
|---|---|
| include/el/scenario_plan.hpp / src/scenario_plan.cu | CUDA 후보 및 비용 평가 |
| include/el/scenario_driver.hpp / src/scenario_driver.cpp | 지도 분석 / 캐시 / 비동기 작업 / 기본 추종기 / RCN |
| include/el/core.hpp / src/core.cpp | 관측 상태 및 점유 스냅샷 |
| include/el/section_router.hpp | 기존 안전 검사 재사용 |
| src/node.cpp | RaceTLN/Avoid 전환 / ROS 실행 및 로깅 |
| src/scenario_teacher.cpp / tools/train_scenario_residual.py | 학습 라벨 및 RCN 학습 |
| tests/scenario_geometry.cpp / tests/scenario_cuda.cpp | 회귀 테스트 |

이전 모델/실험/평가 설명은 [v1.5 README](https://github.com/pineapple3544/RouterTLN/blob/v1.5/README.md)에 보존되어 있다. v3에서는 원본 시뮬레이터 및 채점기를 수정하지 않는다.

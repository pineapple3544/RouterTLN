# RouterTLN — 최종 제출

Race TLN과 Avoid TLN 두 신경망, 규칙 기반 C++ 구간 라우터, 첫 랩에 학습하는 랩 계획, 물리 기반 안전 검사로 이루어진 Berlin 주행 스택이다. 주행 중에는 `/scan`만 사용한다. 저장소: <https://github.com/pineapple3544/RouterTLN>

## [레이스 스택 사용법]

아래 명령을 그대로 복사해 실행한다. ROS 환경 설정(`source`)은 스크립트가 직접 처리한다.

**1. 설치** (깨끗한 `misys:forza_full` 컨테이너 안에서, 한 번만. `misys:final_demo`에는 이미 설치되어 있으므로 건너뛴다.)

```bash
curl -fsSL https://raw.githubusercontent.com/pineapple3544/RouterTLN/main/install_final_demo.sh -o install_final_demo.sh
sudo bash install_final_demo.sh
```

설치가 끝나면 `[install] OK`가 출력된다. 실패하면 실패한 단계와 로그 경로(`~/routertln-install.log`)를 출력하고 멈춘다. 다시 실행하면 끝난 단계는 건너뛴다.

**2. 데모 실행** (시뮬레이터 + RouterTLN 제어기 + RViz + race_manager를 명령 하나로 실행)

```bash
~/run_demo.sh 1      # 랩마다 장애물 1개
~/run_demo.sh 3      # 랩마다 장애물 3개
~/run_demo.sh 5      # 랩마다 장애물 5개
```

- 두 번째 인자는 race_manager 시드, 세 번째 인자는 랩 수다. 기본값은 `2026`, `10`이다. 예: `~/run_demo.sh 5 3105 10`
- 장애물 배치와 랩타임·충돌 기록은 기본 이미지의 `race_manager`(`obstacle_spawner`, `scorer`)를 수정 없이 사용한다. 장애물은 매 랩 다시 뽑힌다.
- 지정한 랩 수를 마치거나 보호 정지하면 스스로 끝난다. 일찍 끝내려면 **Ctrl-C**를 누른다. 끝나면 scorer 요약(최고 랩, 충돌 수)이 터미널에 출력된다.
- 결과: `/workspace/runs/demo-<시각>-obs<N>-seed<S>/race_result/race_result_*.json` (race_manager), `run.csv` (제어기 기록), `*.log`
- 화면이 없는 환경에서는 `DEMO_HEADLESS=1 ~/run_demo.sh 1`

장애물 수와 관계없이 설정은 하나(`config/final_demo.env`)다. 세 평가 주행 사이에 매개변수를 바꾸지 않는다.

## [변경 사항들]

`misys:forza_full` 대비 설치 스크립트가 추가하는 것의 전부다. **F1TENTH Gym, `race_manager`, `forza_ws`의 소스와 설정은 수정하지 않는다.**

| 구분 | 경로 / 패키지 | 내용 |
|---|---|---|
| apt 패키지 | `unzip`, `ninja-build`, `nlohmann-json3-dev`, `rsync` | 빌드 도구. 나머지 의존성(`cmake`, `g++-11`, `libyaml-cpp-dev`, `libpng-dev`, `libeigen3-dev`, `python3-scipy`, `git`, `curl`)은 기본 이미지에 이미 있다 |
| 라이브러리 | `/opt/libtorch-cpu` | LibTorch 2.9.1 CPU (`libtorch-shared-with-deps-2.9.1+cpu`) |
| 코드·모델·지도 | `/home/misys/RouterTLN` | 이 저장소 전체. 제어기 소스(`src/`, `include/el/`), 모델(`models/*.pt`), Berlin 지도(`maps/berlin/`), 실행 스크립트(`scripts/`), 설정(`config/`) |
| 빌드 결과 | `/home/misys/RouterTLN/build-cpu` | `el_controller`, `el_scenario`, `el_inspect`와 CPU 테스트. `-DEL_CONTROLLER_ONLY=ON`이라 CUDA가 필요 없다 |
| 경로 링크 | `/workspace → /home/misys/RouterTLN`, `/build → /home/misys/RouterTLN/build-cpu` | 스크립트와 기본 매개변수가 쓰는 경로 |
| 실행 스크립트 | `/home/misys/run_demo.sh` | 데모 실행기 (저장소의 `run_demo.sh` 사본) |
| 문서 | `/home/misys/README.md` | 이 문서 (저장소의 `SUBMISSION_README.md` 사본) |
| 캐시 | `…/f1tenth_gym/gym/f110_gym/envs/__pycache__/*.nbi` | Gym이 처음 실행될 때 스스로 만드는 numba 컴파일 캐시. 소스 변경은 아니다. 설치 중 짧은 무장애물 주행으로 미리 만든다. 캐시가 없으면 첫 주행에서 컴파일 지연으로 센서 시간 초과 정지가 날 수 있다 |
| 로그 | `/home/misys/routertln-install.log`, `/workspace/runs/install-warmup*` | 설치 기록 |

이 저장소에서 기본 이미지의 노드를 감싸는 파일:

- `scripts/local_gym_bridge.py`: 기본 `f1tenth_gym_ros.gym_bridge.GymBridge`를 **상속만** 하는 래퍼다. 물리 한 단계와 센서 발행을 한 주기로 묶고, 제어기와 같은 단조 시계를 쓴다. 채점용 물리 기록(`EL_JUDGE_FILE`)을 남긴다. 강건성 시험에서만 쓰는 선택 옵션 `EL_SCAN_NOISE_SCALE`(기본 1, 꺼짐)이 있다.
- `scripts/run_sim.sh`, `scripts/smoke.sh`: 시뮬레이터, `el_scenario`(시계와 출발 위치), 제어기를 실행한다. `run_demo.sh`에서는 `el_scenario`의 장애물 발행을 꺼서(`EL_SCENARIO_OBSTACLES=0`) 장애물 발행기가 race_manager 하나뿐이다.

## [내부 조사가 요구되는 파일들]

> **작성 중:** 담당자 열은 팀이 확인한 담당 구분으로 채운다. 아래 "커밋 작성자"는 `git log --stat --author=<이름>`으로 만든 기록이며 실제 담당과 다를 수 있다. 공동 작업 파일에는 함수마다 `// <함수> 함수는 <이름> 담당` 주석을 단다.

| 파일 | 역할 | 커밋 작성자 (git) | 담당자 |
|---|---|---|---|
| `src/node.cpp` | ROS 제어기 노드: `/scan` 수신, 시간 검사, 위치 추정, 라우터 호출, `/drive` 발행 | pineapple4566 | _(확인 필요)_ |
| `include/el/section_router.hpp` | Race/Avoid 구간 라우터, 속도 상한, 명령 안전 검사 | pineapple4566 | _(확인 필요)_ |
| `include/el/avoid_corridor.hpp` | 관측한 점유에서 회피 통로 계획, 조향 미리보기 | pineapple4566 | _(확인 필요)_ |
| `include/el/lap_plan.hpp` | 첫 랩 정찰 기록과 이후 랩 속도 계획 | pineapple4566 | _(확인 필요)_ |
| `include/el/scan_localizer.hpp` | 스캔–정적 지도 정합 위치 추정 (정답 위치 미사용) | pineapple4566 | _(확인 필요)_ |
| `src/core.cpp`, `include/el/core.hpp` | 점유 기억(Memory), 관측 생성, 경로 | pineapple4566 | _(확인 필요)_ |
| `include/el/dynamics.hpp`, `include/el/race_speed.hpp` | 차량 동역학 예측, 시야 기반 속도 상한 | pineapple4566 | _(확인 필요)_ |
| `include/el/network.hpp`, `src/policy.cpp` | TLN 신경망 구조와 추론 | pineapple4566 | _(확인 필요)_ |
| `src/train.cpp`, `src/collect.cpp`, `src/teacher.cu`, `src/mppi.cu` | 교사 데이터 수집과 학습 | pineapple4566 | _(확인 필요)_ |
| `src/scenario.cpp`, `scripts/*` | 시험 시나리오와 실행·채점 스크립트 | pineapple4566 | _(확인 필요)_ |
| `README.md`, `docs/berlin-policy-comparison.csv` | 문서, 모델 비교 | KN_Yeon, pineapple4566 | _(확인 필요)_ |
| `install_final_demo.sh`, `run_demo.sh`, `config/final_demo.env`, `CMakeLists.txt`(`EL_CONTROLLER_ONLY`) | 설치·데모 | _(커밋 전)_ | _(확인 필요)_ |

전체 커밋 이력: [`git_commit_history.txt`](git_commit_history.txt)

## [자체 평가 결과]

> **작성 중:** `final_demo.mp4`에 사용한 주행의 race_manager 결과로 채운다. 최고 랩은 가장 긴 무충돌 연속 랩 안에서 고른다.

| 항목 | 값 |
|---|---|
| obs1_best_lap_time | _(측정 후 기입)_ |
| obs1_highest_lap_count | _(측정 후 기입)_ |
| obs3_best_lap_time | _(측정 후 기입)_ |
| obs3_highest_lap_count | _(측정 후 기입)_ |
| obs5_best_lap_time | _(측정 후 기입)_ |
| obs5_highest_lap_count | _(측정 후 기입)_ |

## 신경망을 두 개 쓰는 이유

라우터가 주기마다 Race TLN(일반 주행)과 Avoid TLN(장애물 사이 통로를 추가 입력으로 받는 회피 주행) 중 하나를 고른다. 단일 통합망(schema 5, `unified-race-avoid-v2.pt`)과 같은 라우터·안전 검사로 두 번 비교했다.

- **오프라인, 18개 배치** (장애물 1/3/5개 × 6, 배치당 최대 3랩, [`docs/berlin-policy-comparison.csv`](docs/berlin-policy-comparison.csv)): 분리형은 **18/18 배치에서 3랩**(54랩)을 완주했다. 통합형은 누적 **8랩**을 완주했고 2랩에 도달한 배치는 없었다. 최고 랩타임은 분리형 16.35–20.69초, 통합형 23.62–30.24초였다.
- **ROS 폐루프, 5개 배치** ([`runs/berlin-sensor-only/policy-comparison.csv`](runs/berlin-sensor-only/policy-comparison.csv)): 분리형은 4개 배치를 완주했다(1개는 `no_corridor`로 2랩 후 보호 정지). 통합형은 5개 모두 첫 랩 전에 `braking_footprint`로 보호 정지했다.

**한계:** 두 구성은 학습 순서와 WSP 적용 범위도 달랐다. 따라서 이 결과는 "분리 자체의 효과"를 따로 측정한 것이 아니다. 같은 데이터·에폭·WSP 처리로 통합망을 다시 학습해 비교하는 실험은 아직 하지 않았다. 분리형을 택한 근거는 위의 연속 완주 수와 랩타임 차이, 그리고 기준 경로 추종과 회피 조향을 따로 조정할 수 있다는 구조적 이점이다.

## 정답 정보 감사 (Ground truth audit)

**주행 경로는 정답 정보를 쓰지 않는다.** 제어기 노드(`el_native_controller`)의 실제 구독 목록 ([`runs/berlin-sensor-only/ros-node-info.txt`](runs/berlin-sensor-only/ros-node-info.txt)):

| 구독 | 용도 |
|---|---|
| `/scan` (`sensor_msgs/LaserScan`) | 유일한 센서 입력 |
| `/clock` | 시뮬레이션 시계 |
| `/parameter_events` | ROS 기본 |

발행은 `/drive`, `/el/status`뿐이다. `/car_state/odom_GT`, `/car_state/pose_GT`, `/sim/obstacles`, `/initialpose`는 구독하지 않는다.

- 위치는 `ScanLocalizer`가 스캔을 정적 지도에 정합해 추정한다. 초기 위치도 지도 전체에서 찾으며 출발 위치나 `/initialpose`를 받지 않는다 (`include/el/scan_localizer.hpp`).
- 장애물은 스캔으로 관측한 점유 기억(`Memory`)으로만 판단한다.
- 정답 정보(장애물 위치·크기, 차량 위치)는 **학습**(교사 데이터 수집, `src/collect.cpp`, `src/teacher.cu`)과 **채점**(race_manager `scorer`의 `/car_state/odom_GT`, 물리 기록 `EL_JUDGE_FILE`, `scripts/score_drive.py`)에만 쓴다.
- `el_scenario`는 시뮬레이터에 출발 위치(`/initialpose`)를 보내고 시계를 발행한다. 제어기는 이 노드의 어떤 토픽도 구독하지 않는다.
- 구간별 상수(웨이포인트 번호, 좌표, 위치별 표)가 없다는 점의 근거: [`docs/hardcoding-audit.md`](docs/hardcoding-audit.md)

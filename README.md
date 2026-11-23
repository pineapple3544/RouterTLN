# RouterTLN

## 문제 정의

**경량 TLN이 첫 랩에서 관측한 장애물과 성공한 회피 경로를 다음 랩의 감속·모델 전환·복귀에 활용할 수 있을까? Berlin에서 충돌과 정지 없이 연속 주행하면서 랩타임을 줄이는 것이 목표다.**

원형 TinyLidarNet은 2D 라이다 스캔으로 조향과 스로틀을 직접 예측한다. 코너 직후에 장애물이 있다면 첫 랩에는 시야에 들어온 뒤 대응할 수 있다. 두 번째 랩에는 코너에 들어가기 전부터 감속할 수 있지만, 그러려면 장애물을 만난 위치와 통과 경로를 기억하고 현재 위치와 연결해야 한다. RouterTLN에서는 매 제어 주기에 Race와 Avoid 중 하나가 명령을 만들고, C++ 라우터가 랩을 넘나드는 기억과 전환 계획을 맡는다. [TinyLidarNet 원 논문](https://arxiv.org/abs/2410.07447) · [랩 계획 구현](include/el/lap_plan.hpp)

## Berlin 정방향에서 확인한 완주율과 랩타임

**첫 랩에서 만든 Race 속도 계획은 같은 4.25m/s 상한에서 오프라인 180초 완주를 1/6에서 6/6으로 늘렸다.** 장애물 1개인 개발 배치 `new-1-051`에서는 센서 노이즈 시드를 바꾼 ROS 두 주행이 각각 12랩을 정지·충돌 없이 마쳤다. 베스트랩은 **14.086초와 13.837초**였고, 두 주행 모두 첫 랩을 제외한 중앙값이 약 14.16초였다. 이 설정은 선택형 실험 옵션이다. [구현·배치별 결과](experiments/berlin-endurance/scout-speed-plan.md) · [분기 재생과 평가 명령](runs/berlin-scout-speed-plan/README.md)

| 시험과 설정 | 시험 범위 | 무정지·무충돌 완주 | 기록된 랩타임 |
|---|---|---:|---|
| 센서 기반 오프라인 3랩 | Berlin 장애물 배치 300개 | 289/300 (96.3%) | 통과 주행의 1·2·3랩 중앙값 28.17 / 18.20 / 18.18초 |
| 센서 기반 ROS 3랩 | 사전 추첨 30개 배치 × 2회 | 58/60 (96.7%) | 통과 주행의 1·2·3랩 중앙값 28.70 / 18.51 / 18.45초 |
| 기존 Race 속도 제어, 오프라인 180초 | 장애물 1·3·5개 각 100배치, 상한 3.5 / 3.65 / 3.75m/s | 292 / 292 / 290회 (각 300회) | 세 설정이 모두 통과한 289개에서 배치별 베스트랩 평균 16.560 / 16.471 / 16.414초 |
| 기존 Race 속도 제어, 미사용 배치 180초 | 30개 배치, 상한 3.5 / 3.75m/s | 각 30/30 | 배치별 베스트랩 평균 16.700 / 16.558초 |
| 첫 랩 기반 Race 속도 계획, ROS 180초 | 장애물 1개 동일 배치, 상한 4.25m/s, 노이즈 시드 2개 | 2/2, 각각 12랩 | 각 주행의 베스트랩 14.086 / 13.837초 |

속도 개선 전 Race 상한 3.5m/s의 오프라인 180초 완주율은 **292/300(97.3%)**였다. 장애물 1·3·5개 조건에서 각각 99/100, 99/100, 94/100개가 통과했고, 전체 물리 접촉은 0건이었다. 실패 8건은 제동 궤적 보호 정지 6건과 통로 탐색 실패 2건이었다. 통과한 292개 배치의 평균 베스트랩은 **16.569초**였다. [기준 성적과 실패 배치](experiments/berlin-endurance/README.md#baseline-300)

3랩 ROS 시험에서는 29개 배치가 두 번 모두 통과했다. 남은 한 배치에서는 첫 랩의 비슷한 위치에서 두 번 모두 통로를 찾지 못해 보호 정지했다. 오프라인 180초 시험에서는 속도를 높여 평균 랩타임이 줄어도 기존 성공 배치에서 새 실패가 나왔다. 첫 랩 기반 4.25m/s 계획의 검증 범위는 오프라인 6개 배치와 ROS 1개 배치다. 이 계획의 미사용 300개 배치 성공률과 다른 지도에서의 완주율은 아직 측정하지 않았다. [3랩 시드별 판정](runs/berlin-sensor-only/README.md) · [180초 짝비교 결과](experiments/berlin-endurance/README.md)

개발 배치에서 기록한 **180초 안전 완주 중 ROS 최단은 13.547초**였다. 시야·출구 제어 5.25m/s 개발 빌드의 기록이며, 같은 상한의 최종 빌드는 한 번은 13.706초로 완주하고 한 번은 둘째 랩에서 보호 정지했다. 새 첫 랩 계획도 5.25m/s에서 6랩 뒤 제동 보호가 작동했다. **180초 안전 완주와 12.999초 이하를 함께 달성한 기록은 없다.** [설정·빌드별 ROS 랩타임](experiments/berlin-endurance/lap12-implementation.md) · [독립 채점 기록](experiments/berlin-endurance/ros-development-scores.json)

같은 배치에서 180초를 통과한 7회 주행의 정찰 이후 77랩을 8등분했다. 각 구간의 최단 시간만 합치면 13.077초이고, 현재 4.25m/s 계획의 22랩만 사용하면 13.675초다. 각 구간은 서로 다른 진입 속도에서 시작할 수 있어 이 합계는 실제 랩 기록에서 제외한다. [랩별 구간 시간](experiments/berlin-endurance/flying-lap-sectors.csv)

## 대회 주행 입력과 평가 기준

180초 내구 시험은 첫 제어부터 랩 수 제한 없이 달린다. 실제 대회는 3분 또는 최대 10랩이며, 장애물 1·3·5개 조건에서 연속 완주 수와 베스트랩을 각각 평가한다.

- **자차 자세**는 매 LiDAR 스캔을 Berlin 정적 점유 지도에 정합해 추정한다. 출발 시 지도 전체에서 위치와 방향 후보를 비교하고, 주행 중에는 직전 구동 명령으로 예측한 자세를 스캔으로 보정한다. 장애물 때문에 벽과 맞지 않는 반사점의 영향을 제한한다. 센서 입력은 `/scan` 하나다. 실제 ROS 구독 목록에는 시계와 파라미터 이벤트도 표시된다. IMU·오도메트리의 미공개 노이즈 크기는 추정기의 설정값으로 사용하지 않는다. [위치 추정기](include/el/scan_localizer.hpp) · [실제 구독 목록](runs/berlin-sensor-only/ros-node-info.txt)
- **장애물 인지와 랩 계획**에는 추정 자세, LiDAR 반사점, 정적 지도를 사용한다. 첫 랩에서 관측한 장애물과 성공한 회피 구간을 기억하고 이후 랩의 감속과 전환에 쓴다. 지도 경로점은 연속적인 경로 진행과 기하 계산에 사용한다. 시험 스크립트의 출발 인덱스, 장애물 위치·크기와 시드는 시뮬레이터 배치를 정의한다. 제어기에는 배치별 분기나 위치별 lookahead 표가 없다. [점유 지도](src/core.cpp) · [랩 계획](include/el/lap_plan.hpp)
- **구동 명령**은 C++ 라우터 한 개가 발행한다. 검증한 구성은 Race와 Avoid 두 신경망 중 하나를 주기마다 선택한다. Race는 일반 주행을 학습했고 Avoid는 장애물 사이에서 차체가 통과할 경로를 추가 입력으로 학습했다. 관측한 장애물, 경로의 통과 가능성, 복귀 여유가 전환 조건이다. 단일망 후보 `unified-race-avoid-v2.pt`와 동일한 다섯 배치를 비교한 결과, 두 모델은 4회 완주했고 단일망은 5회 모두 `braking_footprint`로 보호 정지했다. 이 비교는 두 모델을 사용한 기술적 근거이며 MUX 배점은 심사자가 판단한다. [동일 조건 결과](runs/berlin-sensor-only/policy-comparison.csv) · [라우터](include/el/section_router.hpp)
- **학습과 채점**에서 사용하는 정답은 경로가 다르다. 교사 데이터 생성과 연구용 인계 재생에는 시뮬레이터 정답을 쓸 수 있다. 정식 3랩 시험에서 물리 시뮬레이터는 충돌과 궤적을 별도 파일에 쓴다. 주행을 끝낸 뒤 평가기가 실제 전진 3랩, 결승선 전 정지와 충돌을 판정한다. 완료 명령을 보냈더라도 실제 3랩을 채우지 못하면 실패다. [ROS 제어기](src/node.cpp) · [독립 채점기](scripts/score_drive.py) · [평가 결과와 재현법](runs/berlin-sensor-only/README.md)

### 자차 정답 자세를 사용했던 이전 실험

Berlin 정방향 동기식 시험에서 같은 두 모델, 같은 8개 시드, 한 변 30 cm인 장애물 3개를 사용했다. 첫 랩 속도 상한을 2.2 m/s, 이후 상한을 3.0 m/s로 설정했을 때 **랩 계획을 끄면 0/8회, 켜면 8/8회** 연속 3랩을 완주했다. [비교 조건과 시드별 결과](runs/berlin-lap-plan/RESULTS.md)

**기존 ROS 검증 결과** · 첫 랩 이후 Avoid 목표 속도 하한을 2.7 m/s로 둔 CPU 추론 시험에서는 **18회 중 15회가 무정지·무충돌로 3랩을 완주**했다. 여섯 배치 모두 완주 기록이 있지만 보호 정지 3회가 남아 있다. 전체 충돌은 0회였다. [배치별 판정과 랩타임](runs/berlin-control-timing/results.json)

## Berlin 분리형 채택 근거

동일한 규칙 기반 라우터와 안전 검사를 사용해 WSP 통합 TLN과 WSP Race + 원본 Avoid를 비교했다. Berlin 오프라인 시뮬레이션에서 장애물 수별 6개 배치를 사용했고, 배치당 최대 3랩을 주행했다.

| 장애물 수 | 통합형 누적 완주 랩 | 분리형 누적 완주 랩 | 통합형 최고 랩타임 | 분리형 최고 랩타임 |
|---|---:|---:|---:|---:|
| 1 | 4/18 | 18/18 | 23.62초 | 16.35초 |
| 3 | 2/18 | 18/18 | 27.43초 | 16.89초 |
| 5 | 2/18 | 18/18 | 29.47초 | 18.07초 |

분리형은 18개 배치 모두에서 3랩을 완주했다. 통합형은 8개 배치에서 1랩을 완주했고 2랩에 도달한 배치는 없었다. 양쪽 모두 첫 랩을 마친 8개 배치의 첫 랩 평균은 통합형 26.31초, 분리형 28.13초였다. 기준 경로 추종과 회피 조향을 독립적으로 조정할 수 있고 이번 시험에서 연속 완주 수와 최고 랩타임이 모두 우세한 분리형을 채택했다. 학습 순서와 WSP 적용 범위도 달랐으므로 분리 자체의 효과를 따로 측정한 결과는 아니다. [배치별 결과](docs/berlin-policy-comparison.csv)

## 주행 구성

| 구성 | Berlin에서 하는 일 |
|---|---|
| Race TLN | 일반 구간에서 조향과 속도를 출력한다. Berlin 설정은 WSP로 편집한 `models/race-berlin-wsp.pt`를 선택한다. SMPPI를 교사로 학습했다. |
| Avoid TLN | 라이다·점유 지도·회피 경로를 입력받아 조향과 속도를 출력한다. 주행 시험은 원본 `models/avoid-corridor-v2.pt`를 사용했다. SVG-MPPI를 교사로 학습했다. |
| C++ 라우터 | 장애물 관측, 차체가 지나갈 공간, 복귀 조건에 따라 Race·Avoid·보호 정지를 선택한다. 라우터 자체는 규칙 기반이다. |
| 랩 계획 | 첫 랩 상한은 2.2 m/s다. 회피 진입·복귀 위치와 횡방향 경로를 기록하고, 이후 랩에서 장애물이 다시 관측되면 계획을 적용한다. 시험마다 계획을 초기화한다. |

[라우터](include/el/section_router.hpp) · [랩 계획](include/el/lap_plan.hpp) · [Berlin 설정](maps/berlin/router.yaml) · [계획을 켰을 때와 껐을 때의 비교](runs/berlin-lap-plan/RESULTS.md)

## Weight-Space Programming 실험

MIT CSAIL의 *Sparks of Weight-Space Programming*은 신경망 가중치를 읽고 직접 편집해 동작을 바꾸는 실험을 다룬다. 이 프로젝트에서는 Python/PyTorch로 기존 TorchScript 체크포인트의 가중치를 편집해 개별 정책의 조향·속도 출력을 개선할 수 있는지 시험했다. 편집 효과는 원본과 대조군을 같은 폐루프 조건에서 주행시켜 평가했다. [연구진의 원문](https://rikiyat.github.io/blog/weight-space-programming/) · [가중치 조회 도구](tools/read_weights.py)

- **Race에는 적용했다.** 출력에 미치는 가중치가 작은 은닉 유닛을 재사용해 전방 경로점의 횡방향 특징(`context[6]`)이 조향 헤드에 직접 반영되도록 편집했다. 재배선만 한 대조군의 이득은 0, 적용 후보의 이득은 12다. Berlin의 네 가지 추가 시작 조건에서 원본은 4회 모두 1랩 전에 충돌했고, 편집 모델은 4회 모두 충돌 없이 1랩을 완주했다. 이 시험은 Race 단독 일반화를 본 것으로, 장애물이 있는 3랩 라우터 시험은 아래에 따로 정리했다. [편집 코드](runs/race-wsp-berlin/make_edit.py) · [원본 결과](runs/race-wsp-berlin/original-test/summary.json) · [편집 결과](runs/race-wsp-berlin/guided-test/summary.json)
- **Avoid는 후보를 시험하고 원본을 유지했다.** 속도 출력을 일괄 또는 곡률에 따라 올리는 편집과, 회전 속도·전방 경로를 조향에 반영하는 편집을 비교했다. 당시 2.7 m/s 동기식 시험 12건에서 원본은 5건, 요레이트 보정은 6건, 경로 보정은 8건 완주했다. 배치와 시드에 걸쳐 정지·충돌 0이라는 기준에 도달하지 못했다. 이후의 제동·복귀 수정과 아래의 최신 계측 결과는 원본 Avoid 기준이다. [속도 편집 코드](runs/avoid-wsp-speed/make_edits.py) · [후보별 평가](runs/avoid-flying-band/RESULTS.md)

**고속 Avoid 조향 WSP 후보.** 원본 `avoid-corridor-v2.pt`의 두 은닉층에 유닛을 4개씩 추가했다. 추정 속도 2.7 m/s까지 추가 조향은 0이고, 2.7~3.7 m/s에서는 보정 허용량을 점진적으로 늘린다. 회피 경로의 기준 조향과 현재 추정 조향의 차이를 줄이는 방향으로 작용하며, 원본 대비 모델 조향 명령 차이는 최대 0.020 rad다. 추가 유닛에서 속도 출력으로 가는 가중치는 모두 0이다. 저장한 체크포인트를 다시 읽어 기존 가중치와 보정식을 확인했다. ROS·오프라인 주행, 추론 지연과 완주율은 미측정이다. 기본 Avoid는 원본이고 후보 선택에는 `EL_AVOID_CHECKPOINT=/workspace/models/avoid-corridor-v2-wsp-highspeed-lead.pt`를 사용한다. [후보 체크포인트](models/avoid-corridor-v2-wsp-highspeed-lead.pt) · [편집 및 검증 코드](tools/make_avoid_highspeed_wsp.py)

## 해결한 문제

1. **가속을 과소평가하던 제동 예측.** F1TENTH의 가속 응답과 조향 지연을 반영해 정지까지의 차체 궤적을 검사한다. 3.073 m/s에서 4.9 m/s를 지시한 재현 구간의 50 ms 후 속도는 기존 예측 3.273 m/s, 수정 예측과 시뮬레이터 기록은 3.468 m/s였다. [차량 동역학](include/el/dynamics.hpp)
2. **회피 후 복귀 조향 지연.** 차가 주행선 옆에 있을 때 현재 속도에 맞는 전방 목표점을 보간하도록 변경했다. 원본 Avoid·하한 2.7 m/s로 같은 24개 배치·시드 조합을 비교한 동기식 시험에서 3랩 완주는 13건에서 24건으로 늘었다. 전후 모두 완주한 13건의 2·3랩 평균은 16.92초에서 16.83초로 줄었다. [동일 조건 비교](runs/berlin-brake-merge/offline-summary.json)
3. **간헐적인 25 ms 제어 기한 초과.** 스캔 처리, 점유 지도 갱신, 라우팅, 관측 생성, 회피 경로, 텐서 준비·배치, 순전파, 결과 읽기, 안전 검사를 계측했다. 재현한 33.14 ms 주기에서 입력 준비·GPU 전송은 17.75 ms, GPU 완료를 기다려 결과를 읽는 단계는 13.51 ms였다. 당시 스캔 나이는 45.83 ms였다. 로컬 ROS 시뮬레이션의 기본 추론을 CPU 단일 스레드로 바꾼 뒤 42,423개 제어 주기에서 25 ms 초과가 0회였다. 기록된 관측 766건을 재생했을 때 CUDA와의 최대 속도 명령 차이는 0.0000012 m/s였다. [초과 주기](runs/berlin-control-timing/baseline-extra.json) · [재생 비교](runs/berlin-control-timing/replay.log)
4. **센서 지연의 정지 사유 오표기.** 연산 기한 초과와 센서 데이터 만료를 구별해 기록한다. 150 ms 보호 기준을 유지했다. 전용 시험에서 브리지를 약 241 ms 멈췄을 때 수신 나이 153 ms에서 보호 정지가 발동했고 최종 속도는 0이었다. [제어기](src/node.cpp) · [보호 동작 기록](runs/berlin-control-timing/watchdog-verified/check-controller.log)

## 검증 결과와 남은 문제

통로 진입을 바꾼 후보는 기존 실패 7건 중 6건을 복구했다. 새 300개에서는 3랩 완주가 293회에서 290회로 줄어 기존 버전을 기본으로 유지했다. [배치별 판정과 랩타임 개선 계획](experiments/berlin-passage/README.md)

[3.5 m/s 위치 추정 복구 검증](experiments/berlin-localizer-recovery/README.md)에서 기존 300개, 신규 60개, 대상 배치 ROS 재생 결과를 확인할 수 있다.

[Race 상한과 선제 제동 실험](experiments/berlin-race-speed/README.md)에 동일 300개 배치의 완주율과 장애물 1·3·5개 ROS 랩타임을 정리했다.

조건은 Berlin 정방향, 시작 웨이포인트 37, 30 cm 정사각형 장애물 3개, 랩 계획 켜짐, 전역 상한 3.0 m/s다. 기본·거울 대칭·밀집·같은 쪽·중간 굽이·분산 배치를 사용했다. 결승선의 정상 종료 명령은 보호 정지 집계에서 제외한다. [배치 정의와 실행 코드](runs/berlin-brake-merge/ros.py)

| ROS 시험 | 무정지·무충돌 3랩 | 충돌 | 연산 시간 99백분위 | 최대 | 25 ms 초과 |
|---|---:|---:|---:|---:|---:|
| CUDA 원인 재현, 3개 배치 | 8/9 | 0 | 5.27 ms | 33.14 ms | 1 |
| CPU, 6개 배치 각 2회 | 10/12 | 0 | 2.81 ms | 4.73 ms | 0 |
| CPU, 시뮬레이터·제어기 물리 코어 분리, 6개 배치 각 1회 | 5/6 | 0 | 2.80 ms | 4.55 ms | 0 |

- **스캔 공백 1회.** 기본 배치 3랩 101.97 m에서 새 스캔 간격이 160 ms까지 벌어졌다. 해당 정지 행의 연산은 0.68 ms였다. 간헐적인 센서 공백의 발생 원인은 추가 확인이 필요하다. [원본 CSV](runs/berlin-control-timing/cpu/original-1.csv)
- **회피 경로 탐색 실패 2회.** 기본 배치 첫 랩 4.80 m와 분산 배치 첫 랩 20.09 m에서 라우터가 연결 가능한 경로를 찾지 못했다. 두 건 모두 속도 약 1.24 m/s, 계획 활성화 전, 해당 주기의 신경망 순전파 0 ms였다. 기본 배치에서 실패 주행은 회피 방향이 바뀌었고 같은 위치의 완주 주행과 횡위치가 약 36 cm 달랐다. 분산 배치에서는 방향이 같았고 차이는 약 11 cm였다. 정적 지도와 공표된 장애물 위치로 재구성하면 두 위치 모두 전방 후보가 남는다. 당시 실시간 점유 격자와 처음 끊긴 탐색 단계는 저장되지 않아 정확한 차단 셀은 미확정이다. [기본 배치 실패](runs/berlin-control-timing/cpu-separated/original-1.csv) · [분산 배치 실패](runs/berlin-control-timing/cpu/distributed-2.csv) · [탐색 구현](include/el/avoid_corridor.hpp)
- **학습 판단.** 다음 진단에서는 실패 직전 스캔·점유 격자·단계별 도달 가능 후보를 재생해 지도 오검출과 진입 조향 문제를 구분해야 한다. 통로가 보이는데도 앞선 Avoid 명령이 입구를 놓치는 패턴이 확인되면 해당 진입 상태를 교사 데이터에 추가할 근거가 생긴다. 4.9 m/s 하한의 별도 동기식 시험에서는 2건 중 1건이 충돌했다. [고속 시험 원본](runs/berlin-brake-merge/v1-wall.jsonl)

## 확장 계획

다음 실험에서는 빠른 주행과 안정적인 주행이 갈라진 상태에서 제동 시작과 재가속 시점을 비교한다. 교사 조향은 저장 상태의 비교 재생에만 사용했다. 이번 속도 계획 변경에서 Race와 Avoid 가중치는 유지했다. [교사 조향 비교와 학습 후보](experiments/berlin-endurance/scout-speed-plan.md)

## 검증 환경

| 항목 | 사용한 환경 |
|---|---|
| 호스트 | Windows 11 Pro 빌드 26200, WSL 2.7.8.0, Ubuntu 24.04.3 LTS, 커널 6.18.33.1-microsoft-standard-WSL2 |
| 하드웨어 | AMD Ryzen 7 7800X3D, NVIDIA GeForce RTX 5070 Ti, Windows 드라이버 596.02 |
| 런타임 | Docker 29.6.0, 컨테이너 Ubuntu 22.04.4 LTS, ROS 2 Humble, F1TENTH Gym ROS 브리지와 Forza race_stack |
| 빌드·수치 계산 | C++17, GCC 11.4.0, CMake 3.22.1, CUDA 12.8.93, LibTorch 2.9.1+cu128, Python 3.10.12, Gym 0.19.0, NumPy 1.24.4 |
| 실행 경로 | 컨테이너에서 저장소 루트는 `/workspace`, 빌드 경로는 `/build`. 로컬 단조 시계 ROS 시뮬레이션은 CPU 단일 스레드 추론, 그 밖의 제어기 기본값은 CUDA다. |

베이스 이미지 `misys:forza_full`에 설치된 `/home/misys/forza_ws/race_stack`과 ROS 패키지가 필요하다. 베이스 이미지는 별도 확보해야 한다. 검증 시 로컬 베이스 이미지 ID는 `sha256:55cf4c0cda11d7050827e0cde6e1f25423e67d9b6ff1ec82d63c3d9e8b090bb1`, 개발 이미지 ID는 `sha256:b61c9333e8a2d2bb584402c35c4918a83173358e6f91c8fd32768f66428f8ec5`였다. Docker의 NVIDIA GPU 접근이 필요하다. [Dockerfile](docker/Dockerfile) · [빌드 스크립트](scripts/build.sh)

## 실행과 문제 재현

### 1. 빌드하고 기존 실패 기록 확인

WSL에서 다음 명령으로 별도 컨테이너를 연다.

```bash
git clone https://github.com/pineapple3544/RouterTLN.git
cd RouterTLN
docker image inspect misys:forza_full --format '{{.Id}}'
docker build -t routertln:local -f docker/Dockerfile docker
docker run --rm -it --gpus all --shm-size=2g -v "$PWD:/workspace" -w /workspace --entrypoint /bin/bash routertln:local -i
```

이후 명령은 모두 컨테이너의 `/workspace`에서 실행한다. 첫 명령은 컴파일과 CTest 4개를 실행한다. 다음 두 명령은 저장된 센서·제어 기록에서 같은 판정을 재계산한다.

```bash
bash scripts/build.sh
python3 runs/berlin-control-timing/summarize.py runs/berlin-control-timing/baseline-extra
python3 runs/berlin-control-timing/summarize.py runs/berlin-control-timing/cpu
```

GPU 기록의 예상값은 초과 1건, 기본 배치 2랩, 총 33.14 ms다. CPU 기록에서는 유효한 판단 주기와 첫 거부를 포함한 28,980주기, 기한 초과 0건, 3랩 통과 10/12가 나온다.

### 2. 원본 Avoid로 ROS 폐루프 반복

아래 각 명령은 새 출력 디렉터리를 사용한다. 동일 배치와 시드에서도 ROS 발행 간격과 차량의 진입 위치에 따라 결과가 달라질 수 있다. 간헐적인 실패를 검증할 때는 모든 시도를 분모에 포함한다.

```bash
run=runs/berlin-control-timing/repro-$(date +%Y%m%d-%H%M%S)
EL_INFERENCE_DEVICE=cpu python3 runs/berlin-brake-merge/ros.py "$run"
python3 runs/berlin-control-timing/summarize.py "$run"

EL_INFERENCE_DEVICE=cpu python3 runs/berlin-brake-merge/ros.py "${run}-cores" --spread-cores --repeats 1
EL_INFERENCE_DEVICE=cuda python3 runs/berlin-brake-merge/ros.py "${run}-gpu" --layouts original mirror same-side --repeats 1
python3 runs/berlin-control-timing/summarize.py "${run}-gpu"
```

CUDA 시험에서는 CSV의 `compute_ms > 25`와 `reason=deadline`을 확인한다. `forward_ms`는 호스트의 순전파 호출 시간이고 `readback_ms`에는 GPU 완료 대기가 포함된다. 최신 코드의 입력 준비·전송 합계는 `packing_ms + inputs_ms`이며 초기 CUDA 기록은 두 값을 `inputs_ms` 하나로 기록했다.

### 3. 센서 보호 시험 및 경로 탐색 실패 확인

센서 시험은 전용 ROS 도메인에서 브리지를 약 240 ms 일시 중단한 뒤 재개한다. 센서 만료 사유, 유지된 정지 명령, 최종 속도 0을 검사한다. 경로 탐색 시험은 실패 이력이 있는 기본·분산 배치만 선택한다.

```bash
run=runs/berlin-control-timing/check-$(date +%Y%m%d-%H%M%S)
python3 runs/berlin-control-timing/sensor_check.py "${run}-sensor"
EL_INFERENCE_DEVICE=cpu python3 runs/berlin-brake-merge/ros.py "${run}-path" --layouts original distributed
python3 runs/berlin-control-timing/summarize.py "${run}-path"
```

저장된 실패의 첫 행은 아래 명령으로 바로 확인할 수 있다. `mode=3`, `plan_ready=0`, `forward_ms=0`을 함께 보면 첫 랩의 탐색 실패와 추론 지연을 구별할 수 있다.

```bash
python3 - <<'PY'
import csv
from pathlib import Path
root = Path("runs/berlin-control-timing")
for path in [root/"cpu-separated/original-1.csv", root/"cpu/distributed-2.csv"]:
    with path.open() as stream:
        row = next(r for r in csv.DictReader(stream) if r["reason"] == "no_corridor")
    print(path, {k: row[k] for k in ("step", "progress_m", "speed", "mode", "plan_ready", "forward_ms")})
PY
```

### 4. 가중치 확인 및 WSP 후보 다시 만들기

Python용 PyTorch가 설치된 환경에서 저장소 루트를 기준으로 실행한다. C++ 주행에 쓰는 LibTorch와 Python용 PyTorch는 별도 패키지다. 편집 스크립트는 원본을 읽어 실험 디렉터리에 새 후보를 저장한다. 이미 후보 파일이 있으면 새 작업 사본을 사용한다.

```bash
python3 tools/read_weights.py models/avoid-corridor-v2.pt --tensor head.4.bias
python3 runs/race-wsp-berlin/make_edit.py
python3 runs/avoid-wsp-speed/make_edits.py
```

## 랜덤 장애물 배치기와 평가기 사용법

### 매 랩 새 배치로 평가하기 · Forza `race_manager`

`obstacle_spawner`는 경로와 트랙 폭을 이용해 후보를 만들고, 랩이 바뀔 때 장애물을 다시 뽑아 `/sim/obstacles`로 발행한다. `scorer`는 랩타임과 벽·장애물 충돌을 기록한다. 시뮬레이터와 주행 제어기를 먼저 실행한 뒤, 같은 ROS 도메인의 별도 터미널에서 두 노드를 함께 시작한다. 이때 장애물 발행기는 하나만 실행한다. 아래는 베이스 이미지에 설치된 `hyhall12` 맵 예시다.

```bash
source /opt/ros/humble/setup.bash
source /home/misys/forza_ws/race_stack/install/setup.bash
cd /workspace
ros2 launch race_manager race_manager_launch.xml \
  map_name:=hyhall12 seed:=2026 num_obstacles:=4 \
  log_dir:=/workspace/runs/race-manager plot:=false
```

같은 웨이포인트·배치 설정·시드를 사용하면 같은 순서로 배치를 추첨한다. `seed:=-1`은 새 실행마다 다른 시드를 뽑는다. `num_obstacles`는 랩당 개수다. 원하는 랩 수를 주행한 뒤 launch 터미널에서 **Ctrl-C**로 종료한다. `race_result_*.json`에는 랩별 시간과 충돌 수가 남고, `best lap`에는 출발선부터 결승선까지 충돌 없이 달린 랩만 들어간다. 아래 명령으로 최근 결과의 PNG를 저장할 수 있다. [race_manager 사용 설명서](https://github.com/jimin1203/misys-forza-2026/blob/bfba899165e4de371ac4014cad52add06f27559d/utilities/nodes/race_manager/README.md)

```bash
log=$(ls -t /workspace/runs/race-manager/race_result_*.json | head -1)
ros2 run race_manager report "$log" --no-show
```

**Berlin에서 실행할 때** launch 파일은 `stack_master/maps/<map_name>/global_waypoints.json`을 읽고, 이 저장소의 Berlin 맵은 `/workspace/maps/berlin`에 있다. Berlin 시뮬레이터와 제어기를 실행한 상태에서 아래 두 명령을 각각 별도 터미널에서 실행하면 저장소의 웨이포인트를 직접 지정할 수 있다. 각 터미널에서 위의 두 `source` 명령을 먼저 실행한다.

```bash
# 터미널 1 · 장애물 배치기
cfg="$(ros2 pkg prefix --share race_manager)/config/race_manager_params.yaml"
ros2 run race_manager obstacle_spawner --ros-args --params-file "$cfg" \
  -p waypoints_file:=/workspace/maps/berlin/global_waypoints.json -p seed:=2026 -p num_obstacles:=3
```

```bash
# 터미널 2 · 랩과 충돌 평가기
cfg="$(ros2 pkg prefix --share race_manager)/config/race_manager_params.yaml"
ros2 run race_manager scorer --ros-args --params-file "$cfg" \
  -p waypoints_file:=/workspace/maps/berlin/global_waypoints.json -p map_name:=berlin \
  -p seed:=2026 -p log_dir:=/workspace/runs/race-manager-berlin
```

### 한 배치를 유지하며 랩 계획 평가하기 · RouterTLN

Berlin 3랩 시험에서는 첫 랩의 기록을 다음 랩에 쓸 수 있도록 한 주행 동안 장애물 위치를 유지한다. `scripts/smoke.sh`가 시뮬레이터, `el_scenario`, 제어기를 함께 실행한다. 아래 예시는 웨이포인트 94부터 30개 인덱스 간격으로 장애물 3개를 놓고, 각 장애물의 횡방향 위치를 시드 2026으로 −0.12~+0.12 m 범위에서 뽑는다. 같은 시드로 배치를 재현하거나 `EL_OBSTACLE_OFFSETS='[0.083,-0.083,0.083]'`으로 세 위치를 직접 지정할 수 있다. [배치기 코드](src/scenario.cpp)

```bash
cd /workspace
tag=berlin-seed2026-$(date +%Y%m%d-%H%M%S)
EL_OBSTACLE_OFFSETS= EL_START_INDEX=37 EL_OBSTACLE_INDEX=94 EL_OBSTACLE_STRIDE=30 \
EL_MAP_DIR=/workspace/maps/berlin EL_LAP_PLANNING=true EL_STOP_AFTER_LAPS=3 \
EL_SPEED_CAP=3.0 EL_SIM_AVOID_MIN=2.7 EL_SIM_AVOID_MAX=3.0 EL_INFERENCE_DEVICE=cpu \
bash scripts/smoke.sh 130 3 moe "$tag" "" 2026
/build/el_inspect --metrics "/workspace/runs/$tag.csv"
```

`130`은 주행 제한 시간(초), `3`은 장애물 수, 마지막 인자는 시드다. CSV에서 `reason`, `progress_m`, `collision`으로 정지 지점과 충돌을 확인한다. **여섯 고정 배치의 무정지·무충돌 3랩 판정**은 위의 ‘원본 Avoid로 ROS 폐루프 반복’ 명령을 사용한다. `ros.py`가 배치·회차별 좌표와 CSV를 저장하고, `summarize.py`의 `trials[].accepted`가 각 주행의 통과 여부를 보여 준다.

### 같은 조건에서 모델 두 개를 비교할 때 · 동기식 Gym 평가기

`evaluate_avoid.sh`에 Avoid 체크포인트, 새 결과 JSONL 경로, 시험 조건 JSON, Race 체크포인트를 순서대로 전달한다. 아래는 Berlin 정방향 4개 배치와 3개 시드에서 모델 조합을 3랩씩 평가한다. 모델 간 비교에서는 같은 조건 파일을 사용하고 출력 파일명을 바꾼다. [시험 조건 12개](runs/berlin-brake-merge/paired.json) · [평가기](src/eval_avoid.cpp)

```bash
out="/tmp/berlin-paired-$(date +%Y%m%d-%H%M%S).jsonl"
bash scripts/evaluate_avoid.sh models/avoid-corridor-v2.pt "$out" \
  runs/berlin-brake-merge/paired.json models/race-berlin-wsp.pt
python3 - "$out" <<'PY'
import json, sys
with open(sys.argv[1]) as stream:
    for line in stream:
        r = json.loads(line)
        print(r["case"], r["reason"], len(r["laps"]), r["first_rejection"], r["contact"])
PY
```

한 줄이 한 케이스다. `reason=completed`, 완료 랩 3개, `first_rejection`과 `contact`가 모두 `null`인지 확인한다. 이 시험의 랩타임은 시뮬레이션 시간으로 계산한다. ROS 스캔 지연과 실제 제어 연산 기한은 ROS 폐루프 시험에서 측정한다.


## 기여 기록

저장소 이력에서 `KN_Yeon`은 초기 커밋을 작성했고, `pineapple4566`은 Berlin 제어기·WSP 실험·ROS 재현 문서를 통합한 뒤 문제 정의와 배치·평가 문서를 갱신했다. 팀 리더와 커밋에 드러나지 않는 공동 작업의 역할은 팀 확인이 필요하다. 이력 파일에는 생성 시점까지의 작성자, 날짜, 변경 파일을 기록했다. [커밋 이력](git_commit_history.txt)

제출 직전에는 저장소 루트에서 다음 명령으로 이력 파일을 갱신한다.

```bash
git log --reverse --date=iso-strict --format='%H%n%ad | %an <%ae>%n%s%n' --stat > git_commit_history.txt
```

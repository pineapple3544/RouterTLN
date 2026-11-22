# 연속 장애물 통과 실패 수정

## 원인 재현

기존 Race/Avoid TLN과 구간 라우터에서, HYHALL12 기준 배치 0은 오른쪽 우회를 선택했지만 제동 검사가 실패한 시점에도 차량 중심이 기준선 왼쪽 0.064 m에 있었다. 기준 배치 1은 왼쪽을 선택했는데 정지 직전 차량은 기준선 오른쪽 0.136 m에 있었다. 센서와 기억으로 판단한 통로가 TLN의 조향에 반영되지 않았다. 기존 체크포인트에 국소 경유점만 입력한 대조 시험도 36회 중 2회 성공했다.

## 구현 및 학습

- 라우터가 기준 경로 앞쪽의 횡위치 후보를 이어 통로를 만든다. 차체 점유, 경로 기울기의 변화, 벽·장애물 여유, 이전 통로와의 차이를 평가한다. 가까운 장애물과 복귀 통로는 매 주기 다시 확인한다. 구간 후단을 벗어나 복귀 공간이 열리면 기준선을 다시 추종하고 기존 자세 조건을 충족한 뒤 Race로 전환한다.
- Avoid TLN의 schema 4 입력에 통로 경유점과 기준 조향을 넣는다. 출력 조향은 기준 조향과 네트워크가 학습한 보정량의 합이며, 보정 범위는 ±0.08 rad이다. 속도는 TLN이 출력하고, 전방 곡률과 감속 여유로 상한을 적용한다. 마지막 한 주기 및 제동 궤적 검사를 통과하지 못하면 STOP을 유지한다. 온라인 MPPI는 호출하지 않는다.
- 기존 Avoid 체크포인트를 초기값으로 사용했다. SVG-MPPI 교사 시연의 명령을 같은 배치에서 재생하고, 기록 시각과 속도를 원본과 대조한 뒤 통로 입력을 생성했다. 학습은 46개 주행의 6,158프레임, 검증은 별도 12개 주행의 1,638프레임이다. 16 epoch 중 검증 손실 최저인 16 epoch(0.012081)를 저장하고 다시 로드해 손실을 확인했다. Race는 기존 체크포인트를 사용한다.

## 폐루프 시험

두 지도 HYHALL12·TERAS, 장애물 3개, 속도 상한 3 m/s, LiDAR 18 m, 제어 주기 50 ms. 성공은 세 장애물을 충돌·정지 없이 지난 뒤 Race로 복귀하고 마지막 장애물보다 2 m 이상 전진한 경우다. 수정 전후 배치별 시드, 방향, 장애물 좌표가 일치하는지 확인했다. 새 24개 배치는 수정 전 고정했으며 학습과 모델 선택에 사용하지 않았다.

| 배치 | 이전 성공 | 수정 후 성공 | 수정 후 정지 | 충돌 | 장애물 사이 조기 복귀 |
|---|---:|---:|---:|---:|---:|
| 기존 36개 | 2 | 30 | 6 | 0 | 0 |
| 새 24개 | 1 | 18 | 6 | 0 | 0 |

기존에 성공한 3개 배치는 수정 후에도 모두 성공했다. 전체 60회 중 12회는 정지했다. 7회는 연결 통로 후보 소진, 5회는 조향 명령 적용 후 제동 시의 점유 충돌 예측이었다. 통로 후보 소진은 사용한 격자에서 경로를 찾지 못했다는 의미다. 별도의 폭 3.4 m 차단 장애물에서는 통로 소진으로 정지했고 장애물을 통과하지 않았다. 이 결과는 구간 통과 시험으로, 10 m/s와 완주 랩타임은 측정하지 않았다.

[비교 집계](runs/corridor-control/comparison.json) · [기존 배치 전환 기록](runs/corridor-control/final-reference.jsonl) · [새 배치 전환 기록](runs/corridor-control/final-holdout.jsonl) · [전면 차단 시험](runs/corridor-control/final-blocked.jsonl) · [학습 기록](models/avoid-corridor-v2.pt.training.json)

## ROS 브리지 확인

HYHALL12에 waypoint 56부터 20개 간격으로 장애물 3개를 배치했다. 같은 배치로 다섯 번 실행해 세 번은 Race 복귀, 두 번은 STOP을 관측했다. 다섯 실행 모두 충돌 프레임과 MPPI 제어 주기는 0이었다. 첫 정지는 당시 로그에 최초 사유가 남지 않아 원인을 특정하지 못했다. 이후 최초 정지 사유를 유지하도록 계측을 바꿨고, 한 번은 스캔 또는 오도메트리의 수신 제한 시간(150 ms) 초과가 기록됐다. ROS의 수신 시간 제한과 제동 검사는 유지했다.

| 실행 로그 | Avoid 시작 거리(m) | Race 복귀 거리(m) | 최초 정지 사유 |
|---|---:|---:|---|
| [section-router-guided-ros](runs/section-router-guided-ros.csv) | 2.19 | - | stopped |
| [section-router-guided-repeat](runs/section-router-guided-repeat.csv) | 2.19 | 11.37 | - |
| [corridor-repeat-a](runs/corridor-repeat-a.csv) | 2.19 | 11.27 | - |
| [corridor-repeat-b](runs/corridor-repeat-b.csv) | 2.19 | - | sensor_timeout |
| [corridor-repeat-c](runs/corridor-repeat-c.csv) | 2.19 | 11.37 | - |

## 재실행

WSL의 개발 컨테이너 안에서 실행한다. 평가기는 기존 출력 파일을 보호하므로 새 JSONL 경로를 지정한다.

```bash
cd /workspace
bash scripts/evaluate_avoid.sh models/avoid-corridor-v2.pt /tmp/reference-repeat.jsonl runs/corridor-control/reference.json models/race-teachers-expanded-v5.pt
bash scripts/evaluate_avoid.sh models/avoid-corridor-v2.pt /tmp/holdout-repeat.jsonl runs/corridor-control/holdout.json models/race-teachers-expanded-v5.pt
ROS_DOMAIN_ID=82 EL_OBSTACLE_INDEX=56 EL_OBSTACLE_STRIDE=20 bash scripts/smoke.sh 18 3 moe corridor-smoke-repeat
```

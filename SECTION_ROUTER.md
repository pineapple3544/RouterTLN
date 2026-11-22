# 구간 라우터 구현과 연속 장애물 주행 검증

현재 기본 Avoid 체크포인트는 `avoid-corridor-v2.pt`이다. 수정 근거, 배치별 전후 비교와 ROS 재실행 결과는 [연속 장애물 통과 실패 수정](SECTION_ROUTER_REPAIR.md)에 기록했다.

2026-11-22. WSL Ubuntu의 el-native-dev에서 기존 F110 Gym을 사용했다. 기존 Race 체크포인트 `models/race-teachers-expanded-v5.pt`와 Avoid 체크포인트 `models/avoid-chain-v2.pt`를 로드했다.

## 주행 제어

`include/el/section_router.hpp`의 SectionRouter를 ROS 노드와 동기식 평가기가 공유한다. 매 스캔에서 고정 트랙, 벽과 구별한 LiDAR 반환점, 누적 점유를 조회한다. 라우터는 IQP 기준 경로의 차체 영역이 막힌 위치를 진행 순서로 검사한다. 랩 경계에서 진행 거리를 이어 붙인다.

- 시작 거리는 현재 속도 v(m/s)에 대해 `2 + 0.45*v + v*v/10` m로 설정했다. 이 거리 안에서 기준 경로가 막히면 Avoid를 선택한다.
- 연속해서 막힌 지점 사이의 여유가 시작 거리와 복귀 여유(0.7 m + 현재 횡오차 절댓값)의 합 이하면 같은 구간에 넣는다. 구간 끝을 기억해 중간에 스캔이 끊겨도 Avoid를 유지한다.
- 가까운 장애물에서 좌우 후보 통로의 차체 점유와 곡률을 확인한다. 양쪽이 열려 있으면 가능한 횡위치 샘플이 더 많은 방향을 기록하고, 기존 방향이 유효하면 유지한다. schema 3 체크포인트의 방향 입력은 학습 때처럼 0이다. 라우터가 기록한 방향을 TLN이 따르도록 강제하지 않는다.
- 구간 끝에서 0.35 m 이상 진행하고 1.5 m 복귀 통로가 비어 있어야 한다. 횡오차 0.30 m 미만, 방향 오차 0.32 rad 미만, 전방 위험 해소까지 0.20초 연속 충족하면 Race로 돌아간다.
- 선택한 TLN이 조향과 목표 속도를 낸다. 한 주기의 명령 및 뒤이은 제동 궤적을 지도에서 검사한다. 검사 실패 또는 양쪽 우회 후보 소진 시 속도 0을 유지한다. ROS 주행 노드는 MPPI 라이브러리를 링크하지 않는다. SMPPI와 SVG-MPPI는 별도 오프라인 교사 수집기에서 실행한다.

## 결과

HYHALL12와 TERAS에서 배치당 고정 장애물 3개, 속도 상한 3 m/s, LiDAR 범위 18 m, 제어 주기 50 ms, 물리 주기 10 ms로 평가했다. 실제 장애물 좌표는 배치 및 독립적인 차체 겹침·통과 판정에 사용했다. 장애물 중심을 0.6 m 지난 시점을 통과로 셌다. 정지 명령은 실제 속도 0.05 m/s 미만까지 진행했다. 통과 성공은 세 장애물 통과 후 Race 복귀 및 마지막 장애물에서 2 m 이상 주행이다.

| 배치 | 수 | 첫 장애물 전 Avoid 진입 | 세 장애물 통과 및 Race 복귀 | 정지 | 충돌 |
|---|---:|---:|---:|---:|---:|
| 기존 기준 배치 | 8 | 8 | 0 | 8 | 0 |
| 기존 테스트 배치 | 16 | 15 | 1 | 15 | 0 |
| 이번에 추가한 배치 | 12 | 12 | 1 | 11 | 0 |

- 36회에서 마지막 장애물 통과 전 Race 복귀는 0회였다. 첫 두 장애물이 동시에 라이다에 잡힌 프레임은 34개 배치에서 관측됐다.
- 실제 통과 개수는 0개 24회, 1개 7회, 2개 3회, 3개 2회였다. 정지 사유는 후보 명령의 예상 제동 중 점유 영역과 겹침 21회, 라우터가 허용한 좌우 후보에서 연결 통로를 찾지 못함 13회였다. 후자는 해당 위치에서 모든 물리적 회피 경로가 막혔다는 판정으로 해석할 수 없다.
- 새 TERAS 배치 fresh-07에서는 출발 후 1.84 m에서 Avoid에 진입했다. 5.59, 8.09, 9.78 m의 장애물을 지난 뒤 11.08 m에서 Race로 돌아갔다. 기존 HYHALL12 배치 test-012에서는 2.45 m에서 진입해 14.50 m에서 복귀했다.
- 기존 24개 배치는 초기 실행 후 통로 검사 형상을 보정하고 다시 평가했다. 추가 12개 배치는 수정 후 배치와 시드를 고정해 별도로 실행했다. 이번 시험은 단일 구간 평가로, 랩 간 기억 효과와 10 m/s 성능은 측정하지 않았다.
- 별도의 ROS 브리지 시험은 HYHALL12에 20 waypoint 간격으로 세 장애물을 배치했다. 2.29 m에서 Avoid에 진입한 뒤 4.29 m에서 통로 후보 소진으로 STOP이 발행됐다. 기록 618주기에서 MPPI 0회, 충돌 0프레임이었다.

## 재실행

WSL에서 개발 컨테이너에 들어가 `/workspace`에서 실행한다. 출력 JSONL은 새 경로를 지정한다.

```bash
bash scripts/evaluate_avoid.sh models/avoid-chain-v2.pt /tmp/router-reference.jsonl runs/avoid-chain-v2/test.json models/race-teachers-expanded-v5.pt
bash scripts/evaluate_avoid.sh models/avoid-chain-v2.pt /tmp/router-fresh.jsonl runs/section-router-fresh-cases.json models/race-teachers-expanded-v5.pt
ROS_DOMAIN_ID=82 EL_OBSTACLE_INDEX=56 EL_OBSTACLE_STRIDE=20 bash scripts/smoke.sh 18 3 moe router-repeat
```

[기존 24개 배치 결과](runs/section-router-final.jsonl) · [추가 12개 배치 결과](runs/section-router-fresh.jsonl) · [추가 배치의 좌우 위치와 시드](runs/section-router-fresh-cases.json) · [ROS 명령 기록](runs/section-router-ros-chain.csv)

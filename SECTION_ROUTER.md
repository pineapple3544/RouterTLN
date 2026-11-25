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

## v1.5 실험 모드

v1.5는 첫 랩부터 Avoid 조향 후보의 시작값으로 통로 추종각을 사용한다. 이 각도는 현재 속도와 복귀 여부를 반영해 선택한 통로 전방 지점으로부터 계산하며, 기존 Avoid 입력의 `context[4]`에 들어 있다. Avoid TLN은 계속 추론하고 속도를 제안한다. 두 모드는 같은 Race 정책, 랩 계획, 후보 탐색 및 제동 검사를 사용한다.

| 실행 경로 | v1.5 설정 |
|---|---|
| ROS 파라미터 | `-p lap_planning:=true -p avoid_corridor_steer_seed:=true` |
| Berlin 주행 스크립트 | 기존 실행에 `EL_LAP_PLANNING=true EL_AVOID_CORRIDOR_STEER_SEED=true` 추가 |
| 오프라인 평가기 | 각 case에 `"full_lap": true`, `"lap_planning": true`, `"avoid_corridor_steer_seed": true` 지정 |

v1.5에서 Avoid TLN 조향을 먼저 사용하려면 위 설정에 ROS 파라미터 `-p avoid_policy_first:=true`, 스크립트 환경 변수 `EL_AVOID_POLICY_FIRST=true` 또는 오프라인 case의 `"avoid_policy_first": true`를 추가한다. 기본값은 꺼짐이다.

TLN 원래 조향이 명령·제동 검사와 지연 동역학 예측을 통과하면 각도를 그대로 실행한다. 예측 중 통로 중심과의 거리, 확장 차체와 전면 여유, 정지 가능성을 함께 확인한다. 여유가 부족하면 기존 통로 추종각에서 후보 탐색을 시작한다. 첫 랩의 조향 유지 보정과 최종 센서·제어 마감 보호도 적용된다. 오프라인 결과의 `verified_steps`는 검증한 TLN 조향을 실제 선택한 횟수이며 `output_match_steps`는 다른 경로로 같은 각도가 나온 경우도 포함한다. `control_wall.over_25ms`는 오프라인에서 측정한 초과 횟수다. ROS에서는 `compute_ms`와 최종 전송 상태를 확인한다.

첫 랩 계획이 준비된 TLN 우선 모드는 장애물을 지난 복귀 구간에서 60 ms 앞의 자세로 통로 추종각을 계산한다. 장애물이 사라지고 실제 자세가 복귀 범위에 50 ms 이상 머문 경우, 직전 명령으로 200 ms를 더 예측해 차체 여유와 횡·방향 오차가 계속 허용 범위인지 확인한다. 예측 끝의 횡오차가 시작보다 커지면 조기 복귀를 보류한다. 조기 복귀가 보류된 경우 기존 200 ms 실측 조건을 적용한다. 직선에서는 조향, 곡률, 랩 계획과 LiDAR 제동 거리로 허용되는 중간 속도를 사용한다. Avoid 원시 조향이 처음 150 ms 검증을 통과한 후 기본 추종 거리에서 탈락하면, 속도에 맞춘 추종 거리로 남은 궤적을 포함해 전체를 재검증한다. 경로 오차·차체 여유·제동 기준은 두 검증에서 동일하다.

평가 case의 `profile_control_stages`는 최악 주기와 25 ms 초과 주기의 단계별 경과 시간·스레드 CPU 시간을 기록한다. `profile_step`으로 지정한 제어 주기는 `watched_control`에 따로 보존한다. 단계 측정과 함께 `profile_scheduler`를 켜면 Linux 스레드 실행 대기, 문맥 전환, 주요 페이지 폴트도 기록한다.


기본값은 v1이다. 같은 통로 조건부 Avoid 체크포인트와 같은 배치를 사용해 옵션 하나만 바꾸면 된다. ROS의 `trace_corridor` 또는 평가기의 `checkpoint.ticks` 기록에서 `policy_steer`(TLN)와 `preview_seed_steer`(탐색 시작값)를 라디안 단위로 확인할 수 있다.

Berlin 정방향 기존 6배치, 3.5m/s, 배치별 180초 오프라인 비교에서 v1.5는 첫 랩 6/6, 180초 5/6, 물리 접촉 0건을 기록했다. 3장애물 배치 `new-3-028`은 8랩 후 `no_corridor`로 보호 정지했다.

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

[공통 속도 범위와 첫 랩 전체 경로 실험](experiments/berlin-endurance/shared-scout-route.md)에 후속 랩 설정, 비교 결과, ROS 실행 명령을 정리했다.

[연속 장애물 진입과 단독 출구 비교](experiments/berlin-endurance/avoid-guide-comparison.md)에 네 조합의 180초 주행 결과와 재현 설정을 정리했다.

## 재실행

WSL에서 개발 컨테이너에 들어가 `/workspace`에서 실행한다. 출력 JSONL은 새 경로를 지정한다.

```bash
bash scripts/evaluate_avoid.sh models/avoid-chain-v2.pt /tmp/router-reference.jsonl runs/avoid-chain-v2/test.json models/race-teachers-expanded-v5.pt
bash scripts/evaluate_avoid.sh models/avoid-chain-v2.pt /tmp/router-fresh.jsonl runs/section-router-fresh-cases.json models/race-teachers-expanded-v5.pt
ROS_DOMAIN_ID=82 EL_OBSTACLE_INDEX=56 EL_OBSTACLE_STRIDE=20 bash scripts/smoke.sh 18 3 moe router-repeat
```

[기존 24개 배치 결과](runs/section-router-final.jsonl) · [추가 12개 배치 결과](runs/section-router-fresh.jsonl) · [추가 배치의 좌우 위치와 시드](runs/section-router-fresh-cases.json) · [ROS 명령 기록](runs/section-router-ros-chain.csv)

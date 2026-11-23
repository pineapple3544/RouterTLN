# Berlin 정방향 회피 경로 실패 재현

기본(original)과 분산(distributed) 배치에서 첫 랩부터 새 계획을 만든다. 차는 인덱스 37에서 출발한다. Race는 Berlin WSP 체크포인트, Avoid는 원본 `avoid-corridor-v2.pt`를 사용한다. 계획이 활성화된 랩의 회피 목표 범위는 2.7–3.0 m/s다.

## 점유 격자와 후보 추적

[snapshots](snapshots/)에는 배치별로 경로가 마지막으로 이어졌던 프레임(previous), 처음 끊긴 프레임(failure), 동일 진행 거리의 완주 기록(complete)이 있다. 각 map.yaml과 map.png는 정적 벽과 당시 센서·메모리의 점유 셀을 합친 격자다. frame.json에는 실측 차량 자세, 동적 점유 셀, 각 탐색 단계에서 비어 있는 후보(free)와 이전 단계에서 도달할 수 있는 후보(reachable)가 들어 있다. 비트 0부터 24까지가 왼쪽/오른쪽 부호에 대응하는 횡방향 오프셋 −1.2 m부터 +1.2 m다. first_empty가 1이면 첫 전방 단계에서 후보가 모두 끊겼고, −1이면 끝까지 이어졌다.

[동일 위치의 실패·완주 격자 비교](entry-comparison.png)와 [차량 자세와 격자를 교차 재생한 결과](diagnosis.json)를 함께 볼 수 있다. 분산 배치는 자세를 바꾸는 것만으로 결과가 바뀌었다. 기본 배치는 실패 자세와 실패 시점의 장애물 경계 셀이 함께 있어야 끊겼다. 두 실패 모두 차량을 즉시 트랙 접선 방향으로 놓고 차체를 검사할 때 발생했다. 수정한 탐색기는 실측 차체 방향에서 출발해 트랙 방향으로 연결하고, 첫 구간의 1/4, 1/2, 3/4 위치에서도 차체를 검사한다.

## 재현 명령

ROS Humble 개발 컨테이너의 /workspace에서 빌드한 뒤 다음과 같이 실행한다. ROS 배치별 장애물 좌표와 초기 계획 상태는 평가기가 대조한다. 주행 결과 디렉터리는 매번 새 이름을 사용한다.

```sh
ctest --test-dir /build --output-on-failure
/build/el_replay_corridor tests/fixtures/corridor/original/map.yaml maps/berlin/global_waypoints.json tests/fixtures/corridor/original/frame.json
/build/el_replay_corridor tests/fixtures/corridor/distributed/map.yaml maps/berlin/global_waypoints.json tests/fixtures/corridor/distributed/frame.json
EL_CORRIDOR_TRACE=1 python3 runs/berlin-brake-merge/ros.py runs/berlin-corridor-recheck-trace --layouts original distributed
python3 runs/berlin-brake-merge/ros.py runs/berlin-corridor-recheck --layouts original distributed --spread-cores
python3 runs/berlin-brake-merge/analyze.py runs/berlin-corridor-recheck
```

## 스캔 지연

before와 after는 수신기와 시뮬레이터 양쪽을 계측했다. verify는 계측을 끄고 시뮬레이터와 제어기에 서로 다른 물리 코어를 배정했다. [results.json](results.json)은 각 주행의 랩타임, 정지·충돌, 목표·실제 회피 속도 범위와 센서 시각 통계를 담고 있다. 센서 간격 통계는 제어기가 주행을 기록한 구간에 한정한다. 기존 브리지의 동시 기록으로 손상된 발행기 로그 3줄은 물리 콜백 중첩 집계에서 제외했다. 수신기 간격은 별도 로그에서 계산했다.

기존 로컬 ROS 브리지에서 두 타이머가 물리 스텝 갱신과 스캔 발행을 독립적으로 처리했다. 수정한 브리지는 10 ms 타이머 한 번에 물리를 갱신하고 그 상태를 발행하며, 센서 시각은 갱신을 시작한 시점이다. [의도적 지연 시험](watchdog/check-controller.log)에서는 발행기를 약 241 ms 멈췄을 때 센서 나이 152.4 ms에서 보호 정지가 발생했다. 이전에 관측된 160 ms 지연 자체는 이번 기준 주행에서 재현되지 않았다.

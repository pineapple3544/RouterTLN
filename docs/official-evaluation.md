# 원본 GymBridge / race manager 평가 환경

이 경로는 강사 제공 패키지를 그대로 사용하는 별도 평가 환경이다. `scripts/smoke.sh`, `local_gym_bridge.py`, `el_scenario`, 자체 물리 채점기 및 오프라인 Gym API는 개발용으로만 유지하며 이 경로에서 실행하지 않는다. 기존 패키지의 소스, 타이머, 센서 발행, 노이즈, 충돌 판정 또는 race manager의 배치/채점 구현을 수정하지 않는다.

**현재 상태:** 실행기와 제어 설정을 구현했다. 실제 `misys:forza_full` 환경의 ROS 폐루프 및 race manager 출력은 아직 확인하지 않았다. 설치된 패키지 인터페이스는 기존 저장소에 문서화된 명령을 따르며, 실패하면 해당 로그를 확인한다. 강사의 원본 이미지와 패키지 버전을 유지해야 한다.

## 1. 기존 개발 컨테이너에서 빌드

```bash
cd /workspace
git switch v2
bash scripts/build.sh
```

공식 제어 옵션 `official_simulator=true`는 v1의 시뮬레이터용 속도 계획과 v2를 원본 ROS 시각에서 실행할 수 있도록 한다. `local_steady_sim_clock=false`와 함께 사용한다. 기존 25 ms 기한 및 150 ms 센서 보호는 유지한다. 공식 브리지의 스캔 타임스탬프와 ROS 시계가 다르면 정지한다. 기본은 시스템 시각(`use_sim_time=false`)이다. 강사 환경이 /clock 기반이면 실제 토픽을 확인한 뒤, **시험 시작 전** 하나의 프로파일에서 use_sim_time을 true로 지정한다. 모든 run에서 동일하게 사용한다.

## 2. 실행 전 검사

```bash
bash scripts/official_eval.sh check --profile config/official/v1.yaml
bash scripts/official_eval.sh check --profile config/official/v2.yaml
```

설치된 원본 실행 파일 이름, 설정 파일 경로, 지도, 모델 및 제어 바이너리를 검사하고 해시를 출력한다. 해시는 시험 간 동일성 확인용이지 원본 이미지와의 동일성 자체를 보증하지 않는다. 강사 제공 Berlin 정적 지도와 저장소 지도 파일의 동일성을 확인한다.

| 프로파일 | 모델 | 제어 | 최초 시험 상한 |
|---|---|---|---|
| v1.yaml | 기존 Race WSP / 기존 Avoid | 기존 endurance 방식 | 3.5m/s |
| v2.yaml | 동일 모델 | v2 / 조건부 조향 탐색 | 3.5m/s |

두 설정은 독립적인 개발 비교군이다. 공식 시험에는 검증한 프로파일 하나만 선택한다.

## 3. 자체 평가 (원본 시뮬레이터 및 원본 race manager)

먼저 짧게 연결과 초기화 여부를 확인한다. 별도 ROS 도메인에서 실행한다.

```bash
bash scripts/official_eval.sh run --profile config/official/v1.yaml \
  --count 1 --seed 2026 --duration 30 --domain 74 \
  --output runs/official-v1-smoke
```

통과하면 동일한 모델과 파라미터로 장애물 1/3/5개를 차례로 평가한다.

```bash
bash scripts/official_eval.sh suite --profile config/official/v1.yaml \
  --seed 2026 --domain 74 --output runs/official-v1-2026

bash scripts/official_eval.sh suite --profile config/official/v2.yaml \
  --seed 2026 --domain 74 --output runs/official-v2-2026
```

시드는 자체 시험용이다. 공식 평가는 강사 측의 미공개 seed와 원본 배치기를 사용한다. 실행기는 아래 순서로 각 run을 새 프로세스에서 시작한다.

1. `ros2 run f1tenth_gym_ros gym_bridge` (원본)
2. 원본 `race_manager scorer` 및 `obstacle_spawner`
3. 신선한 /scan과 장애물 메시지의 발행 여부 확인
4. `el_controller` (고정 프로파일)
5. 원본 scorer 결과 저장 후 종료

실행기는 GT 메시지의 장애물 좌표를 읽거나 제어기에 전달하지 않는다. 준비 여부 확인용 별도 노드는 /sim/obstacles 메시지 수신 여부만 확인한다. 제어기의 구독은 /scan과 ROS 기본 관리 토픽 그대로다. 기존 ROS 도메인에 /drive, /scan 또는 /sim/obstacles 발행자가 있으면 실행을 거절한다.

### 180초 또는 10랩 종료

원본 race manager의 정확한 자동 종료 옵션과 JSON 형식은 현재 환경에서 검증할 수 없으므로 임의의 파라미터나 채점 로직을 추가하지 않았다. 실행기는 최초 양의 /drive 명령부터 최대 180초의 보조 벽시계를 사용한다. 원본 scorer가 자체 종료하면 즉시 종료하며, 운영자는 별도 터미널에서 공식 scorer 또는 RViz를 보고 **공식 10랩 도달 또는 종료 충돌 시 Enter**로 종료한다.

```bash
tail -f /workspace/runs/official-v2-2026/obs1/scorer.log
```

Enter를 사용하지 않은 시간 제한 시험은 최대 10랩의 공식 평가를 정확히 재현했다는 의미가 아니다. `trial.json`의 `official_lap_cap_verified`는 검증 전까지 false이며, 실제 점수는 원본 scorer 결과를 기준으로 확인한다. 기존 자체 180초 채점기를 공식 점수로 대체하지 않는다. 원본 race manager가 매 랩 장애물을 재배치하는 설정이면 그 동작을 그대로 둔다. 실제 시험 배치 유지 여부는 강사에게 확인한다.

## 4. 강사 환경에 제어기만 연결

강사 또는 조교가 이미 원본 GymBridge와 race manager를 실행한 상태에서만 사용한다. 기존 /drive 발행자는 먼저 종료해야 한다. 동일 ROS_DOMAIN_ID와 clock 설정을 사용한다.

```bash
bash scripts/official_eval.sh controller --profile config/official/v2.yaml \
  --output runs/official-controller-obs1
```

이 동작은 원본 브리지/배치기/채점기를 새로 실행하지 않는다. 장애물 수 또는 시드를 제어기에 전달하지 않는다. 각 run 전에 제어기를 새로 시작해 기억을 초기화한다. 공식 3조건에서 동일한 프로파일, 모델, 바이너리 및 지도 설정을 사용한다.

## 5. 결과 및 문제 확인

`manifest.json`은 파일 해시 및 설치 패키지 경로, `controller.yaml`은 고정된 설정, `obsN/commands.json`은 실행 명령, `obsN/{bridge,scorer,spawner,controller}.log`와 `obsN/control.csv`는 진단 자료, `obsN/race_manager/`는 원본 scorer의 출력, `trial.json`과 `suite.json`은 실행 상태다. 설정 또는 입력 파일 해시가 바뀌면 다음 run을 시작하지 않는다. 실패 시 자동 재시도하지 않는다.

공식 환경의 첫 시험에서는 스캔 시각, /clock 사용 여부, 제어 기한, 초기 위치 추정, 실제 장애물 초기 발행, 원본 scorer 출력 및 10랩 종료 방식부터 확인한다. 이후 동일 고정 프로파일로 노이즈 강건성과 미사용 배치를 비교한다. 기존 실험의 수치와 새 환경의 수치를 혼용하지 않는다.

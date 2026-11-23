from pathlib import Path
import json,csv,statistics
D=Path(__file__).resolve().parent
s=json.loads((D/'summary.json').read_text());tr=json.loads((D/'ros-results.json').read_text())
with (D/'offline.csv').open() as f:off={r['layout']:r for r in csv.DictReader(f)}
layout_order=json.loads((D/'preregister.json').read_text())['ros_selected']
rows=[]
for seed in layout_order:
    pair=sorted((t for t in tr if t['tag'][:-2]==seed),key=lambda t:t['tag'])
    row={'seed':seed,'offline_ok':off[seed]['accepted']}
    for i,t in enumerate(pair,1):
        row[f'ros_{i}_pass']=t['accepted'];row[f'ros_{i}_reason']=(t['failure'] or ['completed'])[-1]
        for lap,sec in enumerate(t['lap_times'],1):row[f'ros_{i}_lap{lap}_s']=sec
    rows.append(row)
fields=['seed','offline_ok']+[f'ros_{i}_{field}' for i in (1,2) for field in ('pass','reason','lap1_s','lap2_s','lap3_s')]
with (D/'ros-layouts.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
raw={r['case']:r for p in (D/'offline').glob('results-*.jsonl') for l in p.open() if (r:=json.loads(l))}
failrows=[]
for f in s['offline_failures']:
    r=raw[f['layout']];k=r['first_rejection'] or {}
    failrows.append(f"| {f['layout'][7:]} | {len(r['laps'])+1} | {k.get('progress_m',r['progress_m']):.2f} | {f['reason']} |")
text=f"""# Berlin 정방향 미사용 장애물 배치 독립 시험

기준 빌드와 두 모델을 고정했다. 기존과 같은 생성기로 미사용 배치 300개를 생성하고, 결과를 보기 전에 별도 난수 추출로 ROS 배치 30개를 확정했다. 시작점 37, 장애물 3개(0.30 m 정사각형), 장애물 간격 18~140 웨이포인트, 기준선 횡오프셋 ±0.12 m를 사용했다. 원본 Avoid 목표 속도는 계획 주행에서 2.7~3.0 m/s, 제어 기한은 25 ms, 센서 보호는 150 ms였다. 오프라인은 각 배치의 첫 시도, ROS는 배치별 독립 프로세스의 두 시도를 모두 집계했다. 첫 랩 이후 계획 활성화와 2·3랩의 감속을 확인했다.

| 평가 | 통과 | 충돌 | 실패율 95% 단측 상한 |
|---|---:|---:|---:|
| 독립 오프라인 배치 300개 | {s['offline_clean']}/300 | {s['offline_collisions']} | {100*s['offline_upper95']:.2f}% |
| 사전 추첨 ROS 30개 배치에서 두 번 모두 통과 | {s['ros_both']}/30 | {s['ros_collisions']} | {100*s['ros_layout_pair_upper95']:.2f}% |

ROS 개별 주행 통과는 {s['ros_clean']}/60회, 주행 중 물리적 정지 {s['ros_stops']}회, 센서 타임아웃 {s['ros_timeouts']}회였다. 최대 제어 시간은 {s['max_compute_ms']:.2f} ms, 최대 스캔 나이는 {s['max_scan_age_ms']:.2f} ms였다. 반복 주행 두 번은 같은 장애물 배치를 공유하므로 두 번 모두 통과한 배치를 통계 단위로 집계했다. 상한은 독립 배치를 가정한 정확 이항 신뢰한계다.

## 오프라인 실패

| 시드 | 최초 거부 랩 | 누적 거리(m) | 사유 |
|---|---:|---:|---|
"""+'\n'.join(failrows)+"""

시드당 3랩은 하나의 시험이다. 사전 추첨된 ROS 배치와 오프라인 실패 배치의 중복은 0개였다. 관측한 성공률은 이 Berlin 정방향 생성 조건에 해당한다. 오프라인 실패율 1% 미만의 목표는 이번 시험에서 입증하지 못했다.

[추첨 시드·ROS 명단·빌드 해시](preregister.json) · [300개 배치별 랩타임](offline.csv) · [30개 배치별 두 ROS 결과](ros-layouts.csv) · [검증 요약](summary.json)
"""
(D/'README.md').write_text(text)

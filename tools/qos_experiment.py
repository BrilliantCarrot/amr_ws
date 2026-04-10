#!/usr/bin/env python3
"""
qos_experiment.py — W11 Step2 QoS 비교 실험 자동화 스크립트

[역할]
  3가지 케이스를 순서대로 실행하고 결과를 CSV + 터미널 표로 출력.

  Case 1: drop_rate=0.0, RELIABLE    (베이스라인 — 이상 없음)
  Case 2: drop_rate=0.3, RELIABLE    (30% 드롭 + 재전송)
  Case 3: drop_rate=0.3, BEST_EFFORT (30% 드롭 + 재전송 포기)

[사전 조건]
  - bringup.launch.py 실행 중
  - map_ekf_node, mpc_node, localization_monitor_node 실행 중
  - mock_link_node는 이 스크립트가 직접 제어함 (ros2 param set)

[실행 방법]
  cd ~/amr_ws
  python3 tools/qos_experiment.py

[출력]
  tools/qos_results.csv      : 케이스별 측정값
  터미널: 비교표 출력

[동작 원리]
  ① mock_link_node의 drop_rate 파라미터를 런타임에 변경
     ros2 param set /mock_link_node drop_rate 0.3
  ② qos_experiment_node를 RELIABLE/BEST_EFFORT로 각각 실행
  ③ /qos_experiment/stats 토픽을 N초간 구독해서 평균값 수집
  ④ 케이스 완료 후 다음 케이스로 전환

[주의]
  mock_link_node가 실행 중이어야 drop_rate 파라미터 변경이 가능.
  bringup.launch.py를 mock_link:=true로 실행해야 함:
    ros2 launch scenarios bringup.launch.py mock_link:=true
"""

import subprocess
import time
import csv
import os
import sys
import threading
import re
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ============================================================
# 실험 설정
# ============================================================
MEASURE_DURATION_SEC = 30    # 케이스별 측정 시간 [s]
WARMUP_SEC           = 5     # 측정 전 워밍업 시간 [s] (안정화 대기)
OUTPUT_CSV           = os.path.join(
    os.path.dirname(__file__), 'qos_results.csv')

# 실험 케이스 정의
CASES = [
    {
        'name':        'Case1_Baseline',
        'drop_rate':   0.0,
        'reliability': 'reliable',
        'description': '베이스라인 (드롭 없음, RELIABLE)',
    },
    {
        'name':        'Case2_Drop30_Reliable',
        'drop_rate':   0.3,
        'reliability': 'reliable',
        'description': '30% 드롭 + RELIABLE (재전송 있음)',
    },
    {
        'name':        'Case3_Drop30_BestEffort',
        'drop_rate':   0.3,
        'reliability': 'best_effort',
        'description': '30% 드롭 + BEST_EFFORT (재전송 없음)',
    },
]

# 측정 토픽 3개
TOPICS = ['/cmd_vel', '/map_ekf/odom', '/localization/status']


# ============================================================
# StatsCollector — /qos_experiment/stats 구독 및 파싱
# ============================================================
class StatsCollector(Node):
    """
    /qos_experiment/stats 토픽을 구독해서 측정값을 수집함.
    qos_experiment_node가 1Hz로 발행하는 String을 파싱.

    발행 형식 예시:
      [QosExp] reliability=reliable |
      /cmd_vel hz=49.20 loss=1.60% jitter=0.30ms rx=1476 |
      /map_ekf/odom hz=49.10 loss=1.80% jitter=0.31ms rx=1473 |
      /localization/status hz=9.90 loss=1.00% jitter=0.50ms rx=297 |
    """

    def __init__(self):
        super().__init__('stats_collector')

        # 측정값 누적 버퍼
        # key: 토픽 이름, value: {hz, loss, jitter} 리스트
        self.samples: dict = {t: {'hz': [], 'loss': [], 'jitter': []} for t in TOPICS}
        self.lock = threading.Lock()
        self.collecting = False

        self.sub = self.create_subscription(
            String,
            '/qos_experiment/stats',
            self._callback,
            10
        )

    def _callback(self, msg: String):
        if not self.collecting:
            return

        raw = msg.data

        # 각 토픽 파싱
        # 형식: /cmd_vel hz=49.20 loss=1.60% jitter=0.30ms rx=1476
        for topic in TOPICS:
            # 토픽 이름 뒤의 필드를 정규식으로 추출
            pattern = (
                re.escape(topic) +
                r'\s+hz=([\d.]+)\s+loss=([\d.]+)%\s+jitter=([\d.]+)ms'
            )
            m = re.search(pattern, raw)
            if m:
                with self.lock:
                    self.samples[topic]['hz'].append(float(m.group(1)))
                    self.samples[topic]['loss'].append(float(m.group(2)))
                    self.samples[topic]['jitter'].append(float(m.group(3)))

    def start_collecting(self):
        with self.lock:
            for t in TOPICS:
                self.samples[t] = {'hz': [], 'loss': [], 'jitter': []}
            self.collecting = True

    def stop_collecting(self):
        with self.lock:
            self.collecting = False

    def get_averages(self) -> dict:
        """
        수집된 샘플의 평균값을 반환.
        샘플이 없으면 None 반환 (측정 실패).
        """
        result = {}
        with self.lock:
            for topic in TOPICS:
                s = self.samples[topic]
                if not s['hz']:
                    result[topic] = None
                    continue
                result[topic] = {
                    'hz':     sum(s['hz'])     / len(s['hz']),
                    'loss':   sum(s['loss'])   / len(s['loss']),
                    'jitter': sum(s['jitter']) / len(s['jitter']),
                    'n':      len(s['hz']),
                }
        return result


# ============================================================
# 헬퍼 함수들
# ============================================================

def set_mock_link_drop_rate(drop_rate: float):
    """
    mock_link_node의 drop_rate 파라미터를 런타임에 변경.
    ros2 param set 명령어를 subprocess로 실행.
    """
    cmd = [
        'ros2', 'param', 'set',
        '/mock_link_node', 'drop_rate', str(drop_rate)
    ]
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
    if result.returncode != 0:
        print(f"  [경고] drop_rate 설정 실패: {result.stderr.strip()}")
        print(f"  mock_link_node가 실행 중인지 확인하세요.")
        return False
    print(f"  [OK] mock_link_node drop_rate = {drop_rate}")
    return True


def launch_qos_experiment_node(reliability: str) -> subprocess.Popen:
    """
    qos_experiment_node를 지정한 reliability로 실행.
    기존 프로세스가 있으면 종료 후 새로 실행.
    """
    cmd = [
        'ros2', 'run', 'safety', 'qos_experiment_node',
        '--ros-args',
        '-p', f'reliability:={reliability}',
    ]
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    return proc


def kill_qos_experiment_node(proc: subprocess.Popen):
    """qos_experiment_node 프로세스 종료"""
    if proc and proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()


def print_results_table(all_results: list):
    """
    수집된 결과를 터미널 비교표로 출력.

    [출력 형식]
    ┌─────────────────────────────────────────────────────────────────┐
    │ W11 Step2 QoS 실험 결과                                         │
    ├──────────────────────┬──────────┬──────────┬──────────┬─────────┤
    │ 케이스               │ 토픽     │ Hz       │ Loss(%)  │ Jitter  │
    ...
    """
    print()
    print('=' * 75)
    print('  W11 Step2 QoS 실험 결과')
    print('=' * 75)
    print(f"  {'케이스':<28} {'토픽':<22} {'Hz':>6} {'Loss%':>7} {'Jitter(ms)':>11}")
    print('-' * 75)

    for case_result in all_results:
        case_name = case_result['case']['name']
        desc      = case_result['case']['description']
        avgs      = case_result['averages']

        print(f"  [{case_name}]  {desc}")
        for topic in TOPICS:
            short = topic.split('/')[-1][:20]
            data  = avgs.get(topic)
            if data is None:
                print(f"  {'':28} {short:<22} {'N/A':>6} {'N/A':>7} {'N/A':>11}")
            else:
                print(
                    f"  {'':28} {short:<22} "
                    f"{data['hz']:>6.1f} "
                    f"{data['loss']:>6.1f}% "
                    f"{data['jitter']:>10.2f}ms"
                )
        print()

    print('=' * 75)
    print()


def save_csv(all_results: list):
    """결과를 CSV 파일로 저장"""
    fieldnames = [
        'timestamp', 'case_name', 'description',
        'drop_rate', 'reliability', 'topic',
        'avg_hz', 'avg_loss_pct', 'avg_jitter_ms', 'n_samples'
    ]

    now_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

    with open(OUTPUT_CSV, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for case_result in all_results:
            case = case_result['case']
            avgs = case_result['averages']

            for topic in TOPICS:
                data = avgs.get(topic)
                row = {
                    'timestamp':      now_str,
                    'case_name':      case['name'],
                    'description':    case['description'],
                    'drop_rate':      case['drop_rate'],
                    'reliability':    case['reliability'],
                    'topic':          topic,
                    'avg_hz':         round(data['hz'],     2) if data else '',
                    'avg_loss_pct':   round(data['loss'],   2) if data else '',
                    'avg_jitter_ms':  round(data['jitter'], 3) if data else '',
                    'n_samples':      data['n']                if data else 0,
                }
                writer.writerow(row)

    print(f"  CSV 저장 완료: {OUTPUT_CSV}")


# ============================================================
# 메인 실험 루프
# ============================================================
def main():
    print()
    print('=' * 60)
    print('  W11 Step2 — QoS 비교 실험 시작')
    print('=' * 60)
    print(f'  케이스 수      : {len(CASES)}')
    print(f'  케이스별 시간  : 워밍업 {WARMUP_SEC}s + 측정 {MEASURE_DURATION_SEC}s')
    print(f'  총 예상 시간   : ~{len(CASES) * (WARMUP_SEC + MEASURE_DURATION_SEC)}s')
    print(f'  결과 저장 경로 : {OUTPUT_CSV}')
    print()
    print('  [사전 조건 확인]')
    print('  - bringup.launch.py mock_link:=true로 실행 중이어야 함')
    print('  - map_ekf_node, mpc_node 실행 중이어야 함')
    print()

    # rclpy 초기화
    rclpy.init()
    collector = StatsCollector()

    # rclpy spin을 별도 스레드에서 실행
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(collector,), daemon=True)
    spin_thread.start()

    all_results = []
    qos_proc    = None

    try:
        for i, case in enumerate(CASES):
            print(f'[{i+1}/{len(CASES)}] {case["name"]} — {case["description"]}')

            # ① mock_link_node drop_rate 설정
            print(f'  drop_rate = {case["drop_rate"]} 설정 중...')
            if not set_mock_link_drop_rate(case['drop_rate']):
                print('  [경고] drop_rate 설정 실패 — 이 케이스를 건너뜁니다.')
                continue

            # ② 기존 qos_experiment_node 종료 후 새로 실행
            kill_qos_experiment_node(qos_proc)
            time.sleep(1.0)  # 노드 종료 대기
            print(f'  qos_experiment_node reliability={case["reliability"]} 실행 중...')
            qos_proc = launch_qos_experiment_node(case['reliability'])
            time.sleep(2.0)  # 노드 초기화 대기

            # ③ 워밍업 (안정화 대기)
            print(f'  워밍업 {WARMUP_SEC}s...')
            time.sleep(WARMUP_SEC)

            # ④ 측정 시작
            print(f'  측정 시작 ({MEASURE_DURATION_SEC}s)...')
            collector.start_collecting()
            time.sleep(MEASURE_DURATION_SEC)
            collector.stop_collecting()

            # ⑤ 결과 수집
            avgs = collector.get_averages()
            all_results.append({'case': case, 'averages': avgs})

            # 케이스별 간략 결과 출력
            print(f'  [결과]')
            for topic in TOPICS:
                data = avgs.get(topic)
                if data:
                    print(f'    {topic:<25} '
                          f'Hz={data["hz"]:.1f}  '
                          f'Loss={data["loss"]:.1f}%  '
                          f'Jitter={data["jitter"]:.2f}ms  '
                          f'(N={data["n"]})')
                else:
                    print(f'    {topic:<25} 데이터 없음 (노드 연결 확인 필요)')
            print()

    except KeyboardInterrupt:
        print('\n  [중단] 사용자가 실험을 중단했습니다.')

    finally:
        # 정리
        kill_qos_experiment_node(qos_proc)
        # drop_rate 원복
        set_mock_link_drop_rate(0.0)
        rclpy.shutdown()

    # ⑥ 최종 결과 출력 및 저장
    if all_results:
        print_results_table(all_results)
        save_csv(all_results)

        # 핵심 비교: Case2 vs Case3 (30% drop 환경에서 QoS 차이)
        if len(all_results) >= 3:
            print('  [핵심 비교: 30% drop 환경에서 RELIABLE vs BEST_EFFORT]')
            print()
            c2 = all_results[1]['averages']
            c3 = all_results[2]['averages']
            for topic in TOPICS:
                d2 = c2.get(topic)
                d3 = c3.get(topic)
                if d2 and d3:
                    hz_diff     = d3['hz']     - d2['hz']
                    jitter_diff = d3['jitter'] - d2['jitter']
                    print(f'  {topic}')
                    print(f'    Hz:     RELIABLE={d2["hz"]:.1f}  '
                          f'BEST_EFFORT={d3["hz"]:.1f}  '
                          f'차이={hz_diff:+.1f}')
                    print(f'    Jitter: RELIABLE={d2["jitter"]:.2f}ms  '
                          f'BEST_EFFORT={d3["jitter"]:.2f}ms  '
                          f'차이={jitter_diff:+.2f}ms')
                    print()

    print('  실험 완료.')


if __name__ == '__main__':
    main()

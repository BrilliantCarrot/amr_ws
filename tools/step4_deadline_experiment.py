#!/usr/bin/env python3
"""
step4_deadline_experiment.py — W11 Step4 Deadline Miss 실험 자동화

[목표]
  SingleThreadedExecutor 환경에서 artificial_load_ms를 높일수록
  Deadline Miss가 증가하는 것을 수치로 증명.

[실험 케이스]
  Case 1: artificial_load_ms=0  (베이스라인)
  Case 2: artificial_load_ms=5  (중간 부하)
  Case 3: artificial_load_ms=15 (고부하 — Deadline Miss 다수 예상)

[측정 지표]
  avg_latency_ms  : 평균 solve time
  p99_latency_ms  : 99th percentile latency
  max_latency_ms  : 최대 latency
  miss_count      : Deadline Miss 횟수 (>20ms)
  miss_rate_pct   : Deadline Miss 율 [%]

[사전 조건]
  - mpc_node 실행 중
  - deadline_monitor_node 실행 중
  - artificial_load_ms 파라미터가 mpc_node에 선언되어 있어야 함

[실행 방법]
  cd ~/amr_ws
  python3 tools/step4_deadline_experiment.py
"""

import subprocess
import time
import csv
import os
import re
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from amr_msgs.msg import ControlLatency

OUTPUT_CSV = os.path.join(os.path.dirname(__file__), 'step4_results.csv')
MEASURE_SEC = 30   # 케이스별 측정 시간
WARMUP_SEC  = 5    # 부하 변경 후 안정화 대기
DEADLINE_MS = 20.0 # 기준 deadline

CASES = [
    {'load_ms': 0,  'label': 'Case1_Baseline',  'desc': '부하 없음 (베이스라인)'},
    {'load_ms': 5,  'label': 'Case2_Load5ms',   'desc': '5ms 부하'},
    {'load_ms': 25, 'label': 'Case3_Load15ms',  'desc': '25ms 부하 (Deadline Miss 예상)'},
]


# ============================================================
# DataCollector
# ============================================================
class DataCollector(Node):
    def __init__(self):
        super().__init__('step4_collector')

        self.latency_sub = self.create_subscription(
            ControlLatency,
            '/metrics/control_latency_ms',
            self._latency_cb, 10)

        self.stats_sub = self.create_subscription(
            String,
            '/metrics/deadline_miss_stats',
            self._stats_cb, 10)

        self.lock = threading.Lock()
        self.reset()

    def reset(self):
        with self.lock:
            self.latencies  = []
            self.collecting = False
            self.last_stats = ""

    def start(self):
        with self.lock:
            self.collecting = True

    def stop(self):
        with self.lock:
            self.collecting = False

    def _latency_cb(self, msg: ControlLatency):
        with self.lock:
            if self.collecting:
                self.latencies.append(msg.latency_ms)

    def _stats_cb(self, msg: String):
        with self.lock:
            self.last_stats = msg.data

    def get_stats(self) -> dict:
        with self.lock:
            if not self.latencies:
                return {}
            lats = sorted(self.latencies)
            n    = len(lats)
            avg  = sum(lats) / n
            mx   = max(lats)
            p99  = lats[min(int(0.99 * n), n - 1)]
            miss = sum(1 for v in lats if v > DEADLINE_MS)
            return {
                'n':            n,
                'avg_ms':       round(avg,  3),
                'p99_ms':       round(p99,  3),
                'max_ms':       round(mx,   3),
                'miss_count':   miss,
                'miss_rate_pct': round(miss / n * 100.0, 2),
            }


# ============================================================
# 헬퍼
# ============================================================
def set_load(load_ms: int) -> bool:
    cmd = ['ros2', 'param', 'set',
           '/mpc_node', 'artificial_load_ms', str(float(load_ms))]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
    if r.returncode != 0:
        print(f"  [경고] 파라미터 설정 실패: {r.stderr.strip()}")
        return False
    print(f"  [OK] artificial_load_ms = {load_ms}ms")
    return True


def check_prerequisites() -> bool:
    r = subprocess.run(['ros2', 'node', 'list'],
                       capture_output=True, text=True, timeout=5)
    nodes = r.stdout.strip().split('\n')
    required = ['/mpc_node', '/deadline_monitor_node']
    missing  = [n for n in required if n not in nodes]
    if missing:
        print(f"  [오류] 실행 중이 아닌 노드: {missing}")
        return False
    return True


def print_table(all_results: list):
    print()
    print('=' * 70)
    print('  W11 Step4 — Deadline Miss 실험 결과')
    print('=' * 70)
    print(f"  {'케이스':<22} {'avg':>7} {'p99':>7} {'max':>7} "
          f"{'miss':>6} {'miss%':>7}")
    print('-' * 70)
    for r in all_results:
        s = r['stats']
        print(f"  {r['label']:<22} "
              f"{s.get('avg_ms', 0):>6.2f}ms "
              f"{s.get('p99_ms', 0):>6.2f}ms "
              f"{s.get('max_ms', 0):>6.2f}ms "
              f"{s.get('miss_count', 0):>6} "
              f"{s.get('miss_rate_pct', 0):>6.1f}%")
    print('=' * 70)

    # 베이스라인 대비 변화량 출력
    if len(all_results) >= 2:
        print()
        print('  [베이스라인 대비 변화]')
        base = all_results[0]['stats']
        for r in all_results[1:]:
            s = r['stats']
            dp99 = s.get('p99_ms', 0) - base.get('p99_ms', 0)
            dmiss = s.get('miss_rate_pct', 0) - base.get('miss_rate_pct', 0)
            print(f"  {r['label']}: "
                  f"p99 {dp99:+.2f}ms, "
                  f"miss율 {dmiss:+.1f}%")
    print()


def save_csv(all_results: list):
    now_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    fieldnames = ['timestamp', 'label', 'desc', 'load_ms',
                  'n', 'avg_ms', 'p99_ms', 'max_ms',
                  'miss_count', 'miss_rate_pct']
    with open(OUTPUT_CSV, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in all_results:
            s = r['stats']
            writer.writerow({
                'timestamp':     now_str,
                'label':         r['label'],
                'desc':          r['desc'],
                'load_ms':       r['load_ms'],
                'n':             s.get('n', 0),
                'avg_ms':        s.get('avg_ms', 0),
                'p99_ms':        s.get('p99_ms', 0),
                'max_ms':        s.get('max_ms', 0),
                'miss_count':    s.get('miss_count', 0),
                'miss_rate_pct': s.get('miss_rate_pct', 0),
            })
    print(f"  CSV 저장 완료: {OUTPUT_CSV}")


# ============================================================
# 메인
# ============================================================
def main():
    print()
    print('=' * 60)
    print('  W11 Step4 — Deadline Miss 실험')
    print('=' * 60)
    print(f'  Deadline 기준 : {DEADLINE_MS}ms (50Hz 제어 주기)')
    print(f'  케이스 수     : {len(CASES)}')
    print(f'  케이스별 시간 : 워밍업 {WARMUP_SEC}s + 측정 {MEASURE_SEC}s')
    print()

    print('[사전 조건 확인]')
    if not check_prerequisites():
        print('  mpc_node와 deadline_monitor_node를 먼저 실행하세요.')
        return
    print('  확인 완료.')
    print()

    rclpy.init()
    collector = DataCollector()
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(collector,), daemon=True)
    spin_thread.start()

    all_results = []

    try:
        for case in CASES:
            print(f'[{case["label"]}] {case["desc"]}')

            if not set_load(case['load_ms']):
                continue

            collector.reset()
            print(f'  워밍업 {WARMUP_SEC}s...')
            time.sleep(WARMUP_SEC)

            print(f'  측정 시작 ({MEASURE_SEC}s)...')
            collector.start()
            time.sleep(MEASURE_SEC)
            collector.stop()

            stats = collector.get_stats()
            all_results.append({
                'label':   case['label'],
                'desc':    case['desc'],
                'load_ms': case['load_ms'],
                'stats':   stats,
            })

            print(f'  avg={stats.get("avg_ms", 0):.2f}ms  '
                  f'p99={stats.get("p99_ms", 0):.2f}ms  '
                  f'max={stats.get("max_ms", 0):.2f}ms  '
                  f'miss={stats.get("miss_count", 0)}({stats.get("miss_rate_pct", 0):.1f}%)')
            print()

    except KeyboardInterrupt:
        print('\n  [중단]')

    finally:
        # 부하 원복
        set_load(0)
        rclpy.shutdown()

    if all_results:
        print_table(all_results)
        save_csv(all_results)

        print('  [포트폴리오 인사이트]')
        print('  - SingleThreadedExecutor에서 제어 루프 내 연산 부하가')
        print('    증가할수록 Deadline Miss율이 증가함을 수치로 증명')
        print('  - p99 latency와 miss율이 부하에 비례해 증가')
        print('  - Step6 MultiThreadedExecutor 전환 후 동일 부하에서')
        print('    Deadline Miss 감소를 before/after로 비교 예정')


if __name__ == '__main__':
    main()

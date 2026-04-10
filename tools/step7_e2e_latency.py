#!/usr/bin/env python3
"""
W11 Step7: End-to-End Latency 측정 스크립트
============================================================
측정 파이프라인:
  /map_ekf/odom 수신 → mpc_node controlCallback → /cmd_vel 발행

측정 항목:
  - MPC solve time (평균, 최대, p99)
  - e2e latency    (평균, 최대, p99)

KPI 기준:
  - e2e latency p99 < 50ms

사용법:
  python3 ~/amr_ws/tools/step7_e2e_latency.py
  python3 ~/amr_ws/tools/step7_e2e_latency.py --duration 120
============================================================
"""

import rclpy
from rclpy.node import Node
from amr_msgs.msg import ControlLatency
import numpy as np
import argparse
import sys
import time


class E2ELatencyCollector(Node):
    def __init__(self, duration: float):
        super().__init__("step7_e2e_latency")

        self.duration = duration
        self.start_time = time.time()

        # 수집 데이터
        self.solve_times = []   # MPC solve time [ms]
        self.e2e_times   = []   # end-to-end latency [ms]

        self.sub = self.create_subscription(
            ControlLatency,
            "/metrics/control_latency_ms",
            self.callback,
            10
        )

        self.get_logger().info(
            f"[Step7] e2e latency 수집 시작. 측정 시간: {duration}초"
        )
        self.get_logger().info(
            "[Step7] /metrics/control_latency_ms 구독 중..."
        )

    def callback(self, msg: ControlLatency):
        elapsed = time.time() - self.start_time

        # 측정 시간 초과 시 종료
        if elapsed > self.duration:
            self.print_results()
            rclpy.shutdown()
            return

        # 유효값만 수집 (e2e가 비정상적으로 크거나 음수인 경우 제외)
        if 0.0 < msg.e2e_latency_ms < 500.0:
            self.e2e_times.append(msg.e2e_latency_ms)
        if 0.0 < msg.latency_ms < 500.0:
            self.solve_times.append(msg.latency_ms)

        # 10초마다 중간 결과 출력
        n = len(self.e2e_times)
        if n > 0 and n % 500 == 0:
            remaining = self.duration - elapsed
            self.get_logger().info(
                f"[Step7] 수집 중... {n}샘플 | "
                f"e2e avg={np.mean(self.e2e_times):.2f}ms | "
                f"남은 시간: {remaining:.0f}초"
            )

    def print_results(self):
        print("\n" + "=" * 60)
        print("W11 Step7: End-to-End Latency 측정 결과")
        print("=" * 60)

        if not self.e2e_times:
            print("[ERROR] 수집된 데이터 없음. 시스템이 실행 중인지 확인하세요.")
            return

        solve = np.array(self.solve_times)
        e2e   = np.array(self.e2e_times)

        print(f"\n총 샘플 수: {len(e2e)}개 ({self.duration}초 수집)")
        print(f"샘플링 주파수: {len(e2e)/self.duration:.1f} Hz\n")

        print("─" * 60)
        print(f"{'항목':<30} {'MPC Solve [ms]':>15} {'e2e [ms]':>15}")
        print("─" * 60)
        print(f"{'평균 (Mean)':<30} {np.mean(solve):>15.2f} {np.mean(e2e):>15.2f}")
        print(f"{'중앙값 (Median)':<30} {np.median(solve):>15.2f} {np.median(e2e):>15.2f}")
        print(f"{'최솟값 (Min)':<30} {np.min(solve):>15.2f} {np.min(e2e):>15.2f}")
        print(f"{'최댓값 (Max)':<30} {np.max(solve):>15.2f} {np.max(e2e):>15.2f}")
        print(f"{'90th Percentile':<30} {np.percentile(solve, 90):>15.2f} {np.percentile(e2e, 90):>15.2f}")
        print(f"{'95th Percentile':<30} {np.percentile(solve, 95):>15.2f} {np.percentile(e2e, 95):>15.2f}")
        print(f"{'99th Percentile (p99)':<30} {np.percentile(solve, 99):>15.2f} {np.percentile(e2e, 99):>15.2f}")
        print(f"{'표준편차 (Std)':<30} {np.std(solve):>15.2f} {np.std(e2e):>15.2f}")
        print("─" * 60)

        # KPI 판정
        p99_e2e = np.percentile(e2e, 99)
        kpi_pass = p99_e2e < 50.0

        print(f"\n[KPI] e2e latency p99 < 50ms: ", end="")
        if kpi_pass:
            print(f"✅ PASS ({p99_e2e:.2f}ms)")
        else:
            print(f"❌ FAIL ({p99_e2e:.2f}ms)")

        # e2e > 50ms 발생 비율
        over_50 = np.sum(e2e > 50.0)
        print(f"[INFO] e2e > 50ms 발생: {over_50}회 / {len(e2e)}회 "
              f"({over_50/len(e2e)*100:.2f}%)")

        # e2e latency 분해 분석
        print(f"\n[분석] e2e latency 분해:")
        print(f"  MPC solve time avg  : {np.mean(solve):.2f}ms")
        print(f"  오dom→control 대기  : {np.mean(e2e) - np.mean(solve):.2f}ms")
        print(f"  총 e2e avg          : {np.mean(e2e):.2f}ms")

        print("\n" + "=" * 60)

        # 결과 파일 저장
        import os
        output_path = os.path.expanduser(
            "~/amr_ws/tools/step7_e2e_result.txt"
        )
        with open(output_path, "w") as f:
            f.write("W11 Step7: End-to-End Latency 측정 결과\n")
            f.write("=" * 60 + "\n")
            f.write(f"총 샘플 수: {len(e2e)}개\n")
            f.write(f"측정 시간: {self.duration}초\n\n")
            f.write(f"MPC solve time:\n")
            f.write(f"  avg={np.mean(solve):.2f}ms  p99={np.percentile(solve,99):.2f}ms  max={np.max(solve):.2f}ms\n\n")
            f.write(f"e2e latency:\n")
            f.write(f"  avg={np.mean(e2e):.2f}ms  p99={np.percentile(e2e,99):.2f}ms  max={np.max(e2e):.2f}ms\n\n")
            f.write(f"KPI (p99 < 50ms): {'PASS' if kpi_pass else 'FAIL'} ({p99_e2e:.2f}ms)\n")

        print(f"[저장] 결과 저장 완료: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="W11 Step7 e2e latency 측정")
    parser.add_argument(
        "--duration", type=float, default=60.0,
        help="측정 시간 [초] (기본값: 60)"
    )
    args = parser.parse_args()

    rclpy.init()
    node = E2ELatencyCollector(duration=args.duration)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.print_results()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
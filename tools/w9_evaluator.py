"""
W9 동적 장애물 회피 검증 — 실시간 데이터 로거 (w9_evaluator.py)
================================================================
목적:
    W9 동적 장애물 회피 시나리오 실행 중 MPC 제어 출력과
    장애물 이격 거리를 실시간으로 구독하여 CSV로 기록한다.
    기록된 CSV는 충돌 여부 판정 및 KPI 사후 분석에 사용된다.

구독 토픽:
    /cmd_vel                      : MPC가 발행하는 선속도(v), 각속도(w)
    /metrics/min_obstacle_distance: obstacle_tracker_node가 발행하는
                                    로봇-장애물 최소 이격 거리 (m)

출력:
    ~/amr_ws/w9_result.csv
        컬럼: Time(s), v(m/s), w(rad/s), min_clearance(m)
        주기: 0.1s (10Hz)

KPI 판정 기준 (W9):
    - min_clearance > 0.0m 유지 → 충돌 없음
    - v = 0.200 m/s 일관 유지 → 속도 안정성
    - MPC solve time 평균 < 20ms (별도 토픽으로 확인)

사용법:
    # 시뮬레이션 실행 중 별도 터미널에서 실행
    ros2 run control_mpc w9_evaluator

    # 종료 (Ctrl+C) 시 CSV 자동 저장
    # 이후 분석:
    python3 tools/rmse_calc.py --input ~/amr_ws/w9_result.csv
"""

import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from amr_msgs.msg import MinObstacleDistance

class W9Evaluator(Node):
    def __init__(self):
        super().__init__('w9_evaluator')
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_subscription(MinObstacleDistance, '/metrics/min_obstacle_distance', self.dist_cb, 10)
        
        self.v = 0.0
        self.w = 0.0
        self.min_dist = 9.99
        
        # CSV 파일 경로 설정
        csv_path = os.path.expanduser('~/amr_ws/w9_result.csv')
        os.makedirs(os.path.dirname(csv_path), exist_ok=True)
        self.csv_file = open(csv_path, 'w')
        self.csv_file.write("Time(s), v(m/s), w(rad/s), min_clearance(m)\n")
        self.get_logger().info(f"CSV 저장 경로: {csv_path}")

        self.timer = self.create_timer(0.1, self.log_cb)
        self.start_time = self.get_clock().now()
        self.get_logger().info("데이터 로깅을 시작합니다. (Ctrl+C로 종료)")

    def cmd_cb(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def dist_cb(self, msg):
        self.min_dist = msg.min_distance_m

    def log_cb(self):
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        line = f"{t:.1f}, {self.v:.3f}, {self.w:.3f}, {self.min_dist:.3f}"
        print(line)
        self.csv_file.write(line + '\n')
        self.csv_file.flush()  # Ctrl+C 시 손실 방지

def main(args=None):
    rclpy.init(args=args)
    node = W9Evaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.csv_file.close()  # 여기서 파일 닫기
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
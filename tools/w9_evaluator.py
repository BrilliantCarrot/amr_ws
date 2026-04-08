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
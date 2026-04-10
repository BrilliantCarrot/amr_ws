std_msgs/Header header
float64 latency_ms        # 제어 루프 1회 실행 시간 (MPC solve time)
float64 avg_latency_ms    # 이동 평균
float64 max_latency_ms    # 최대값
float64 e2e_latency_ms    # end-to-end: /map_ekf/odom stamp → cmd_vel 발행
float64 avg_e2e_ms        # e2e 이동 평균
float64 p99_e2e_ms        # e2e 99th percentile
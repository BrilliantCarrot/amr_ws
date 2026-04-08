std_msgs/Header header
float64[] x        # centroid x [m] (map frame)
float64[] y        # centroid y [m]
float64[] vx       # 추정 속도 x [m/s] (첫 감지 시 0)
float64[] vy       # 추정 속도 y [m/s]
float64[] radius   # 추정 반경 [m]
int32 count        # 장애물 수
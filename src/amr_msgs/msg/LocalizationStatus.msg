std_msgs/Header header
float64 rmse_x          # map frame x RMSE [m]
float64 rmse_y          # map frame y RMSE [m]
float64 rmse_yaw        # yaw RMSE [rad]
float64 rmse_total      # 2D 위치 RMSE [m]
bool is_localized       # NORMAL 또는 DEGRADED이면 true, bool is_localized 상태가 LOST가 아니면 true. MPC나 Fail-safe 같은 상위 모듈이 "지금 localization 믿어도 돼?"를 빠르게 판단할 때 쓰는 단순 플래그
uint8 status            # 0=NORMAL, 1=DEGRADED, 2=LOST. 0 = NORMAL : 정상, 1 = DEGRADED : TF 점프 감지, 품질 저하, 2 = LOST : TF 타임아웃, localization 완전 소실
float64 tf_jump_m       # 직전 스텝 대비 map→odom TF 점프 크기 [m]. 직전 모니터링 스텝(100ms 전) 대비 map 프레임에서 로봇 위치가 얼마나 갑자기 바뀌었는지. 0.3m 초과 시 DEGRADED 판정. slam_toolbox가 스캔 매칭에 실패하거나 루프 클로저가 갑자기 보정할 때 튀는 값
float64 tf_age_sec      # map→odom TF 마지막 갱신 후 경과 시간 [s]
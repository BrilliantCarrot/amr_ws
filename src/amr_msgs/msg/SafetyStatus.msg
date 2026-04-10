std_msgs/Header header
uint8 state             # 현재 상태: 0=NORMAL, 1=DEGRADED, 2=SAFE_STOP, 3=MANUAL_OVERRIDE
string state_name       # 상태 이름 문자열 (로그/디버깅용)
string last_trigger     # 마지막 전환을 일으킨 트리거 이름
float64 time_in_state   # 현재 상태 진입 후 경과 시간 [s]
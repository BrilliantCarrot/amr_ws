# WatchdogEvent.msg — watchdog_node가 감지한 이상 조건 이벤트
#
# [이벤트 종류 상수]
#   LOC_JUMP_CRITICAL : localization 위치 점프 > 0.5m (심각)
#   EKF_DIVERGED      : EKF innovation norm 폭증 (발산 신호)
#   CMD_TIMEOUT       : /cmd_vel 수신 없음 > timeout
uint8 LOC_JUMP_CRITICAL = 1
uint8 EKF_DIVERGED      = 2
uint8 CMD_TIMEOUT       = 3

std_msgs/Header header
uint8   event_type   # 이벤트 종류 (상수 참조)
string  event_name   # 이벤트 이름 문자열 (로그용)
float64 value        # 트리거된 실측값 (점프 크기, norm, 경과 시간 등)
float64 threshold    # 해당 이벤트의 임계값
string  detail       # 추가 설명

# MoveIt2 Config Placeholder

W6에서 실제 로봇팔 USD/URDF를 확정한 뒤 MoveIt Setup Assistant 결과를 이디렉터리에 둔다.

현재 W6 최종 구조는 MoveIt2 `FollowJointTrajectory` action을
`/franka/joint_command`로 변환하는 bridge를 사용한다.

MoveIt2 설치 후에는 다음 항목을 이 디렉터리에 추가한다.

- `config/*.srdf`
- `config/kinematics.yaml`
- `config/joint_limits.yaml`
- `config/controllers.yaml`
- `launch/demo.launch.py` 또는 프로젝트용 `moveit.launch.py`

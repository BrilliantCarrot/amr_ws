import os
import subprocess
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_scenarios = get_package_share_directory('scenarios')
    pkg_localization = get_package_share_directory('localization')
    urdf_file  = os.path.join(pkg_scenarios, 'urdf', 'amr_robot.urdf.xacro')
    world_file = os.path.join(pkg_scenarios, 'worlds', 'simple_room.sdf')

    # xacro -> urdf 변환 (ROS 표준이라 중간 단계로)
    urdf_out = '/tmp/amr_robot.urdf'
    subprocess.run(['xacro', urdf_file, '-o', urdf_out], check=True)
    # subprocess 이유: ROS2 런치 시스템은 비동기적으로 동작하는데 xacro 변환은 다음 단계(SDF 변환)가
    # 의존하므로 동기적으로 먼저 끝내야 함
    # urdf -> sdf 변환 (Gazebo 시뮬레이터가 물리 연산을 잘 수행하기 위해 최종적으로 변환)
    sdf_out = '/tmp/amr_robot.sdf'
    subprocess.run(['ign', 'sdf', '-p', urdf_out],
                   stdout=open(sdf_out, 'w'), check=True)
    # ParameterValue로 감싸서 string으로 명시, 타입 불일치 오류 방지
    # 명렁어(cat)를 실행해 얻은 URDF 파일의 텍스트 내용을 
    # ROS2 파라미터에서 사용할 수 있는 문자열 값으로 변환
    robot_description = ParameterValue(
        Command(['cat ', urdf_out]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Fortress는 ign gazebo 명령 사용
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_file],
        output='screen'
    )
    # -s /world/simple_room/create  → 호출할 서비스 이름
    # --reqtype EntityFactory       → 요청 메시지 타입 (protobuf)
    # --reptype Boolean             → 응답 메시지 타입
    # --timeout 5000                → 5000ms 안에 응답 없으면 실패
    # --req '...'                   → 요청 내용 (protobuf text format)
    # --req 내용 분석:
    # sdf_filename: "/tmp/amr_robot.sdf"   ← 어떤 로봇을 생성할지
    # name: "amr_robot"                    ← Gazebo 내 인스턴스 이름
    # pose: {position: {x:0, y:0, z:0.3}} ← 초기 위치 (z=0.3: 바닥에서 살짝 띄움)
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ign', 'service',
                    '-s', '/world/simple_room/create',
                    '--reqtype', 'ignition.msgs.EntityFactory',
                    '--reptype', 'ignition.msgs.Boolean',
                    '--timeout', '5000',
                    '--req',
                    f'sdf_filename: "{sdf_out}", name: "amr_robot", '
                    f'pose: {{position: {{x: 0, y: 0, z: 0.3}}}}'
                ],
                output='screen'
            )
        ]
    )
    # bridge는 gazebo의 메시지를 ROS가 이해할 수 있는 메시지 형식으로 변환하여 전달
    # Gazebo → ROS: 시뮬레이션 속 로봇의 센서 데이터(카메라, 라이다, IMU 등)를 ROS 토픽으로 보내줌
    # ROS → Gazebo: ROS에서 계산한 제어 명령(cmd_vel 등)을 Gazebo로 전달하여 로봇을 움직이게 함
    # 파서는 기호 위치를 기준으로 앞 = ROS2 타입, 뒤 = Ignition(Gazebo) 타입으로 고정 파싱
    # 브리지 방향 기호 핵심:
    # 기호              의미                      데이터 흐름
    #  ]      ROS2 → Ignition (단방향)  ROS2가 퍼블리시, Gazebo가 수신
    #  [      Ignition → ROS2 (단방향)  Gazebo가 퍼블리시, ROS2가 수신
    #  @      양방향                    양쪽 모두 가능
    # 토픽별 정리:
    # 토픽            기호        방향                       용도
    # /cmd_vel        ]    ROS2 → Gazebo    MPC가 보내는 속도 명령 → 로봇 구동
    # /odom           [    Gazebo → ROS2    시뮬 오도메트리 → EKF 입력
    # /imu            [    Gazebo → ROS2    시뮬 IMU 데이터 → EKF 입력
    # /scan           [    Gazebo → ROS2    LiDAR 스캔 → 로컬라이제이션/장애물 감지
    bridge = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='ros_ign_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                    '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                    '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                    '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                    # Ground Truth: Gazebo 내부 로봇 실제 위치
                    # → EKF 추정값과 비교해서 RMSE 계산에 사용
                    # Pose_V: 모든 모델의 포즈를 한 번에 담은 배열 타입
                    # → TFMessage로 브리지됨, pose_rmse_node에서 amr_robot 포즈만 추출
                    '/world/simple_room/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                ],
                output='screen'
            )
        ]
    )

    # slam 모드 인자: true이면 slam_toolbox 노드를 함께 실행
    # 사용 예: ros2 launch scenarios bringup.launch.py slam:=true
    declare_slam = DeclareLaunchArgument(
        'slam',
        default_value='false',
        description='true이면 slam_toolbox 맵 생성 모드 실행'
    )
    slam = LaunchConfiguration('slam')

    # slam:=true일 때만 slam.launch.py 인클루드
    # TimerAction으로 브리지/센서가 준비된 후 실행되게 지연 (10초)
    slam_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_localization, 'launch', 'slam.launch.py')
                ),
                condition=IfCondition(slam),
            )
        ]
    )

    return LaunchDescription([
        declare_slam,
        robot_state_publisher,
        gazebo,
        bridge,
        spawn_robot,
        slam_launch,
    ])

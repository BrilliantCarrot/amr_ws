import os
import subprocess
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, TimerAction, IncludeLaunchDescription,
    DeclareLaunchArgument, OpaqueFunction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    pkg_scenarios   = get_package_share_directory('scenarios')
    pkg_localization = get_package_share_directory('localization')

    # world 인자를 런치 시간에 문자열로 평가
    # perform(context): LaunchConfiguration 객체 → 실제 문자열로 변환
    # 이렇게 해야 world_name + '.sdf' 같은 파이썬 문자열 연산이 가능함
    world_name = LaunchConfiguration('world').perform(context)
    urdf_file  = os.path.join(pkg_scenarios, 'urdf', 'amr_robot.urdf.xacro')
    world_file = os.path.join(pkg_scenarios, 'worlds', world_name + '.sdf')

    # xacro → urdf 변환 (동기적으로 먼저 끝내야 sdf 변환이 가능)
    urdf_out = '/tmp/amr_robot.urdf'
    subprocess.run(['xacro', urdf_file, '-o', urdf_out], check=True)

    # urdf → sdf 변환 (Gazebo가 sdf를 직접 로드함)
    sdf_out = '/tmp/amr_robot.sdf'
    subprocess.run(
        ['ign', 'sdf', '-p', urdf_out],
        stdout=open(sdf_out, 'w'),
        check=True
    )

    # robot_description 파라미터: string 타입 명시 필수
    # value_type=str 없으면 ROS2가 타입 추론 실패해서 에러 발생
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

    # Gazebo Fortress: ign gazebo 명령 사용 (gz sim 아님)
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_file],
        output='screen'
    )

    # 5초 후 로봇 스폰 (Gazebo 초기화 대기)
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
                    f'pose: {{position: {{x: 0, y: 0.0, z: 0.3}}}}'
                ],
                output='screen'
            )
        ]
    )

    # 8초 후 ros_ign_bridge 실행 (센서/액추에이터 토픽 브릿지)
    # 브릿지 방향 기호:
    #   ]  ROS2 → Ignition (단방향)
    #   [  Ignition → ROS2 (단방향)
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
                    # Ground Truth: EKF RMSE 계산용 실제 로봇 위치
                    '/world/simple_room/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                ],
                output='screen'
            )
        ]
    )

    # slam:=true 일 때만 slam.launch.py 포함 (10초 후, 브릿지 준비 후)
    slam = LaunchConfiguration('slam')
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

    return [
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        slam_launch,
    ]


def generate_launch_description():

    # world 인자: 월드 파일 이름 (확장자 제외)
    # 사용 예:
    #   ros2 launch scenarios bringup.launch.py              → simple_room (장애물 있음)
    #   ros2 launch scenarios bringup.launch.py world:=simple_room_empty  → 장애물 없음
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='simple_room',
        description='사용할 월드 파일 이름 (확장자 제외)'
    )

    # slam 인자: true이면 slam_toolbox 맵 생성 모드 실행
    # 사용 예:
    #   ros2 launch scenarios bringup.launch.py slam:=true   → 맵 생성 모드
    #   ros2 launch scenarios bringup.launch.py              → localization 모드 (기본)
    declare_slam = DeclareLaunchArgument(
        'slam',
        default_value='false',
        description='true이면 slam_toolbox 맵 생성 모드 실행'
    )

    return LaunchDescription([
        declare_world,
        declare_slam,
        # OpaqueFunction: LaunchConfiguration 값을 런치 시간에 문자열로 평가해서
        # 파이썬 연산(문자열 연결 등)이 가능하도록 함
        OpaqueFunction(function=launch_setup),
    ])
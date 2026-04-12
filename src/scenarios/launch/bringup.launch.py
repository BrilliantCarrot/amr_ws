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

    pkg_scenarios    = get_package_share_directory('scenarios')
    pkg_localization = get_package_share_directory('localization')

    # world 인자를 런치 시간에 문자열로 평가
    # perform(context): LaunchConfiguration 객체 → 실제 문자열로 변환
    # 이렇게 해야 world_name + '.sdf' 같은 파이썬 문자열 연산이 가능함
    world_name = LaunchConfiguration('world').perform(context).strip()
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

    # --------------------------------------------------------
    # W11 Step1: mock_link 활성화 여부에 따라 브릿지 YAML 분기
    #
    #   mock_link:=false (기본):
    #     ros_gz_bridge.yaml 사용
    #     ROS /cmd_vel → Ignition /cmd_vel 직접 연결
    #
    #   mock_link:=true:
    #     ros_gz_bridge_mock_link.yaml 사용
    #     ROS /cmd_vel_delayed → Ignition /cmd_vel 연결
    #
    #   [중요] config_file + arguments 혼용 금지.
    #     ros_gz_bridge에서 두 방식을 동시에 쓰면 충돌 발생.
    #     Ground Truth 토픽 포함 모든 토픽을 YAML에 명시.
    # --------------------------------------------------------
    mock_link_enabled = LaunchConfiguration('mock_link').perform(context).strip().lower()
    use_mock_link = (mock_link_enabled == 'true')

    if use_mock_link:
        bridge_config = os.path.join(
            pkg_scenarios, 'config', 'ros_gz_bridge_mock_link.yaml')
    else:
        bridge_config = os.path.join(
            pkg_scenarios, 'config', 'ros_gz_bridge.yaml')

    # 8초 후 브릿지 실행
    # config_file 파라미터만 사용 (arguments 없음)
    bridge = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='parameter_bridge',
                output='screen',
                parameters=[{'config_file': bridge_config}],
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

    # mock_link_node: mock_link:=true 일 때만 실행 (10초 후)
    if use_mock_link:
        delay_ms_val     = float(LaunchConfiguration('delay_ms').perform(context).strip())
        drop_rate_val    = float(LaunchConfiguration('drop_rate').perform(context).strip())
        burst_prob_val   = float(LaunchConfiguration('burst_loss_prob').perform(context).strip())
        burst_len_val    = int(LaunchConfiguration('burst_loss_len').perform(context).strip())
        reorder_prob_val = float(LaunchConfiguration('reorder_prob').perform(context).strip())
        reorder_max_val  = float(LaunchConfiguration('reorder_max_delay_ms').perform(context).strip())

        mock_link_action = TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='safety',
                    executable='mock_link_node',
                    name='mock_link_node',
                    output='screen',
                    parameters=[{
                        'enabled':              True,
                        'delay_ms':             delay_ms_val,
                        'drop_rate':            drop_rate_val,
                        'burst_loss_prob':      burst_prob_val,
                        'burst_loss_len':       burst_len_val,
                        'reorder_prob':         reorder_prob_val,
                        'reorder_max_delay_ms': reorder_max_val,
                    }]
                )
            ]
        )

    # --------------------------------------------------------
    # W12: path_planner_node (글로벌 경로계획)
    #
    #   use_planner:=true 일 때만 실행 (15초 후 — SLAM 안정화 대기)
    #   slam_toolbox가 /map을 발행하기까지 보통 10~15초 소요.
    #
    #   goal_x/y: launch 파라미터로 기본 목표 지점 설정
    #   런타임 중 RViz2 "2D Goal Pose" 또는
    #   ros2 topic pub /goal_pose 로 동적 변경 가능.
    # --------------------------------------------------------
    use_planner_str = LaunchConfiguration('use_planner').perform(context).strip().lower()
    # goal_x/y는 use_planner=true일 때만 읽음
    goal_x_val = 2.0
    goal_y_val = 0.0
    if use_planner_str == 'true':
        goal_x_val = float(LaunchConfiguration('goal_x').perform(context).strip())
        goal_y_val = float(LaunchConfiguration('goal_y').perform(context).strip())

    common_nodes = [
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        slam_launch,
    ]

    if use_planner_str == 'true':
        path_planner_action = TimerAction(
            period=15.0,   # SLAM이 /map 발행할 때까지 대기
            actions=[
                ExecuteProcess(
                cmd=[
                '/bin/bash', '-c',
                f'source /opt/ros/humble/setup.bash && '
                f'source /home/lyj/amr_ws/install/setup.bash && '
                f'ros2 run planning path_planner_node '
                f'--ros-args '
                f'-p goal_x:={goal_x_val} '
                f'-p goal_y:={goal_y_val} '
                f'-p robot_radius:=0.7 '
                f'-p replan_period_sec:=2.0 '
                f'-p replan_obs_dist:=0.5 '
                f'-p wp_spacing:=0.05 '
                f'-p goal_tolerance:=0.20'
            ],
            output='screen',
            shell=False
        )
            ]
        )
        common_nodes.append(path_planner_action)

    if use_mock_link:
        common_nodes.append(mock_link_action)

    return common_nodes


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

    # W11 Step1: mock_link 관련 인자
    declare_mock_link = DeclareLaunchArgument(
        'mock_link',
        default_value='false',
        description='true이면 mock_link_node 실행 (통신 이상 주입)'
    )
    declare_delay_ms = DeclareLaunchArgument(
        'delay_ms', default_value='0.0',
        description='mock_link: 고정 전파 지연 [ms]'
    )
    declare_drop_rate = DeclareLaunchArgument(
        'drop_rate', default_value='0.0',
        description='mock_link: 독립 드롭 확률 [0.0~1.0]'
    )
    declare_burst_loss_prob = DeclareLaunchArgument(
        'burst_loss_prob', default_value='0.0',
        description='mock_link: burst 손실 시작 확률'
    )
    declare_burst_loss_len = DeclareLaunchArgument(
        'burst_loss_len', default_value='3',
        description='mock_link: burst 연속 드롭 수'
    )
    declare_reorder_prob = DeclareLaunchArgument(
        'reorder_prob', default_value='0.0',
        description='mock_link: reorder 확률'
    )
    declare_reorder_max_delay_ms = DeclareLaunchArgument(
        'reorder_max_delay_ms', default_value='50.0',
        description='mock_link: reorder 최대 추가 지연 [ms]'
    )

    # W12: 글로벌 경로계획 관련 인자
    # 사용 예:
    #   ros2 launch scenarios bringup.launch.py use_planner:=true goal_x:=2.0 goal_y:=1.0
    declare_use_planner = DeclareLaunchArgument(
        'use_planner',
        default_value='false',
        description='true이면 path_planner_node 실행 (A* 글로벌 경로계획)'
    )
    declare_goal_x = DeclareLaunchArgument(
        'goal_x', default_value='2.0',
        description='기본 목표 x 좌표 [m] (map frame)'
    )
    declare_goal_y = DeclareLaunchArgument(
        'goal_y', default_value='0.0',
        description='기본 목표 y 좌표 [m] (map frame)'
    )

    return LaunchDescription([
        declare_world,
        declare_slam,
        declare_mock_link,
        declare_delay_ms,
        declare_drop_rate,
        declare_burst_loss_prob,
        declare_burst_loss_len,
        declare_reorder_prob,
        declare_reorder_max_delay_ms,
        declare_use_planner,   # W12 추가
        declare_goal_x,        # W12 추가
        declare_goal_y,        # W12 추가
        # OpaqueFunction: LaunchConfiguration 값을 런치 시간에 문자열로 평가해서
        # 파이썬 연산(문자열 연결, 분기 등)이 가능하도록 함
        OpaqueFunction(function=launch_setup),
    ])
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 전체 흐름 요약
    #     # bringup.launch.py slam:=true
    #     └── (10초 후) slam.launch.py 인클루드
    #              └── async_slam_toolbox_node 실행
    #                   ├── yaml에서 파라미터 로드
    #                   ├── /scan 구독 (LiDAR)
    #                   ├── /ekf/odom 구독 (EKF 출력)
    #                   ├── /map 발행 (OccupancyGrid)
    #                   └── map→odom TF 발행


    # localization 패키지의 share 디렉토리 경로
    pkg_localization = get_package_share_directory('localization')

    # slam_toolbox 파라미터 yaml 경로
    slam_params_file = os.path.join(
        pkg_localization, 'config', 'slam_toolbox_online_async.yaml'
    )

    # use_sim_time 인자: Gazebo 시뮬 시간 사용 여부
    # use_sim_time이 중요한 이유 — Gazebo는 시뮬레이션 자체 시계(/clock 토픽)를 사용함. 
    # use_sim_time:=true로 설정해야 slam_toolbox가 ROS 시스템 시간이 아닌 Gazebo 시뮬 시간 기준으로 동작함. 
    # 안 하면 타임스탬프 불일치로 TF 오류 발생.
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Gazebo 시뮬 시간 사용 여부'
    )

    # slam_toolbox online async 노드
    # slam_toolbox는 두 가지 모드를 제공함:
    # async_slam_toolbox_node — 비동기 처리. 스캔이 들어오는 속도와 관계없이 처리 가능한 만큼 처리. 실시간 주행에 적합 ✅
    # sync_slam_toolbox_node — 동기 처리. 모든 스캔을 순서대로 처리. 정밀하지만 느릴 수 있음
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file, # yaml 파일 전체를 파라미터로 로드
            {'use_sim_time': use_sim_time} # yaml에 없는 파라미터를 추가로 덮어씀
        ],
    )
    # 각 인자 설명
    # 인자	             의미
    # package	        어떤 패키지에서 실행할지
    # executable	    패키지 안의 실행 파일 이름
    # name	            ROS2 노드 이름 (ros2 node list에 /slam_toolbox로 표시됨)
    # output='screen'	로그를 터미널에 출력 (없으면 파일에만 저장)
    # parameters	    노드에 전달할 파라미터 리스트
    return LaunchDescription([
        declare_use_sim_time,
        slam_node,
    ])

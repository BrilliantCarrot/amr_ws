import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # localization 패키지의 share 디렉토리 경로
    pkg_localization = get_package_share_directory('localization')

    # slam_toolbox 파라미터 yaml 경로
    slam_params_file = os.path.join(
        pkg_localization, 'config', 'slam_toolbox_online_async.yaml'
    )

    # use_sim_time 인자: Gazebo 시뮬 시간 사용 여부
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Gazebo 시뮬 시간 사용 여부'
    )

    # slam_toolbox online async 노드
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        slam_node,
    ])

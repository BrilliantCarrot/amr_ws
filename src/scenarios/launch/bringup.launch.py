import os
import subprocess
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_scenarios = get_package_share_directory('scenarios')
    urdf_file  = os.path.join(pkg_scenarios, 'urdf', 'amr_robot.urdf.xacro')
    world_file = os.path.join(pkg_scenarios, 'worlds', 'simple_room.sdf')

    # xacro -> urdf 변환
    urdf_out = '/tmp/amr_robot.urdf'
    subprocess.run(['xacro', urdf_file, '-o', urdf_out], check=True)

    # urdf -> sdf 변환
    sdf_out = '/tmp/amr_robot.sdf'
    subprocess.run(['ign', 'sdf', '-p', urdf_out],
                   stdout=open(sdf_out, 'w'), check=True)

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
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        bridge,
        spawn_robot,
    ])
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('amiga_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'amiga.urdf')

    # spawn_entity.py node with 3s delay to ensure Gazebo is ready
    spawn_amiga = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-entity', 'amiga', '-file', urdf_file],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        # Start Gazebo with ROS plugins enabled
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so'],
            output='screen'
        ),
        spawn_amiga
    ])

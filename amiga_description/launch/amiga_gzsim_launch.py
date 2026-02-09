from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.actions import TimerAction

import os

def generate_launch_description():
    # Your xacro
    amiga_share = get_package_share_directory('amiga_description')
    xacro_file = os.path.join(amiga_share, 'urdf', 'amiga.urdf.xacro')

    # Gazebo Sim launch
    ros_gz_share = get_package_share_directory('ros_gz_sim')
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            # -r: run, -v 4: verbose, empty.sdf: empty world
            'gz_args': '-r -v 4 empty.sdf'
        }.items()
    )

    # Publish robot_description (still useful for TF and for spawning via topic)
    robot_description = ParameterValue(
        Command(['xacro', ' ', xacro_file]),
        value_type=str
    )
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description
            }]
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Spawn the robot from robot_description topic
    spawn = TimerAction(
        period=3.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=['-name', 'amiga', '-topic', 'robot_description', '-z', '0.3']
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        ),]
    )


    return LaunchDescription([gz_launch, rsp, spawn, clock_bridge])

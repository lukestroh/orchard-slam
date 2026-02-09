#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import UnlessCondition, IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
import os

import rclpy.logging

logger = rclpy.logging.get_logger("orchard_slam.launch")


def launch_setup(context: LaunchContext, *args, **kwargs):
    sim_gazebo = LaunchConfiguration("sim_gazebo")
    gazebo_headless = LaunchConfiguration("gazebo_headless")
    launch_rviz = LaunchConfiguration("launch_rviz")
    world_sdf_file = LaunchConfiguration("world_sdf_file")

    # Package directories
    amiga_description_share = get_package_share_directory("amiga_description")
    orchard_slam_bringup_share = get_package_share_directory("orchard_slam_bringup")
    orchard_slam_gazebo_share = get_package_share_directory("orchard_slam_gazebo")

    # Package launches
    _launch_amiga_description = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                amiga_description_share,
                "launch",
                "amiga.launch.py",
            )
        ),
        launch_arguments={
            "sim_gazebo": sim_gazebo,
        }.items(),
    )

    _launch_gazebo = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                orchard_slam_gazebo_share,
                "launch",
                "orchard_slam_gazebo.launch.py",
            )
        ),
        launch_arguments={
            "gazebo_headless": gazebo_headless,
            "world_sdf_file": world_sdf_file,
        }.items(),
        condition=IfCondition(sim_gazebo),
    )

    _node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(orchard_slam_bringup_share, "rviz", "orchard_slam.rviz")],
        condition=IfCondition(launch_rviz),
    )

    _node_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    _node_diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    _to_run = [
        _launch_amiga_description,
        _launch_gazebo,
        _node_rviz,
        _node_joint_state_broadcaster_spawner,
        _node_diff_drive_controller_spawner,
    ]

    return _to_run


def generate_launch_description():
    declared_configs = [
        dict(name="sim_gazebo", default_value="true", choices=["true", "false"]),
        dict(name="gazebo_headless", default_value="false", choices=["true", "false"]),
        dict(name="launch_rviz", default_value="true", choices=["true", "false"]),
        dict(
            name="world_sdf_file",
            default_value="small_orchard",
            description="Name of the world SDF file (without .sdf extension) to load in Gazebo",
            choices=["room", "small_orchard", "large_orchard"],
        ),
    ]

    declared_args = [
        DeclareLaunchArgument(
            name=config.get("name"),
            default_value=config.get("default_value"),
            choices=config.get("choices"),
            description=config.get("description"),
        )
        for config in declared_configs
    ]

    return LaunchDescription(declared_args + [OpaqueFunction(function=launch_setup)])

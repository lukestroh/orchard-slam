#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction
from launch.conditions import UnlessCondition, IfCondition
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
    launch_gazebo = LaunchConfiguration("launch_gazebo")
    launch_gazebo_gui = LaunchConfiguration("launch_gazebo_gui")
    launch_rviz = LaunchConfiguration("launch_rviz")
    world_sdf_file = LaunchConfiguration("world_sdf_file")



    _launch_gazebo = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("orchard_slam_gazebo"),
                "launch",
                "orchard_slam_gazebo.launch.py",
            )
        ),
        launch_arguments={
            "launch_gui": launch_gazebo_gui,
            "world_sdf_file": world_sdf_file,
        }
        condition=IfCondition(launch_gazebo),
    )

    _to_run = [
        _launch_gazebo,
    ]

    return


def generate_launch_description():
    declared_configs = [
        dict(name="launch_gazebo", default_value="true", choices=["true", "false"]),
        dict(name="launch_gazebo_gui", default_value="true", choices=["true", "false"]),
        dict(name="launch_rviz", default_value="true", choices=["true", "false"]),
        dict(name="world_sdf_file", default_value="orchard_world", description="Name of the world SDF file (without .sdf extension) to load in Gazebo"),
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

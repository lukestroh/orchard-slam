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

logger = rclpy.logging.get_logger("orchard_slam_gazebo.launch")


def launch_setup(context: LaunchContext, *args, **kwargs):
    # Launch configurations
    gazebo_headless = LaunchConfiguration("gazebo_headless")
    world_sdf_file = LaunchConfiguration("world_sdf_file")

    # Package directories
    gz_sim_pkg_share = get_package_share_directory("ros_gz_sim")
    orchard_slam_gazebo_pkg_share = get_package_share_directory("orchard_slam_gazebo")
    gazebo_model_path = os.path.join(orchard_slam_gazebo_pkg_share, "models")

    # Environmental Variables
    _set_env_var_gz_sim_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=orchard_slam_gazebo_pkg_share,
    )

    world_abs_path = os.path.join(
        orchard_slam_gazebo_pkg_share,
        "worlds",
        world_sdf_file.perform(context) + ".sdf",
    )

    _launch_gz_sim = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(gz_sim_pkg_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            # headless: "-r -s" (run + server only)
            # with GUI: "-r" (run + gui)
            "gz_args": f"-r -s {world_abs_path}" if gazebo_headless.perform(context) == "true" else f"-r {world_abs_path}",
        }.items(),
    )



    # _launch_gz_server = IncludeLaunchDescription(
    #     AnyLaunchDescriptionSource(
    #         os.path.join(
    #             gz_sim_pkg_share,
    #             "launch",
    #             "gz_server.launch.py",
    #         )
    #     ),
    #     launch_arguments={
    #         "world_sdf_file": world_abs_path,
    #     }.items(),
    # )

    _node_gz_bridge_clock = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    _node_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-r",
            "-z",
            "1.0",
            "-topic",
            "robot_description",
            "-name",
            "amiga",
            "-allow_renaming",
            "true",
        ],
        output="screen",
    )

    _to_run = [
        _set_env_var_gz_sim_resource_path,
        # _launch_gz_server,
        _launch_gz_sim,
        _node_gz_bridge_clock,
        _node_spawn_robot,
    ]
    return _to_run


def generate_launch_description():
    declared_configs = [
        dict(name="gazebo_headless", default_value="false", choices=["true", "false"]),
        dict(name="world_sdf_file", default_value="small_orchard", choices=["room", "large_orchard", "small_orchard"]),
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

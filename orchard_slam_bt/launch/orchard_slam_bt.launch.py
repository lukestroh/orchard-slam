#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
)
from launch_ros.actions import Node

import rclpy.logging

logger = rclpy.logging.get_logger("bt_orchard_slam.launch")
# logger.set_level(rclpy.logging.LoggingSeverity.WARN)

import datetime as dt


def launch_setup(context: LaunchContext, *args, **kwargs):
    # Launch configurations
    load_map = LaunchConfiguration("load_map")
    map_name = LaunchConfiguration("map_name")
    record_bag = LaunchConfiguration("record_bag")
    use_sim_time = LaunchConfiguration("use_sim_time")
    initial_start_pose = LaunchConfiguration("initial_start_pose")

    # Package directories
    orchard_slam_bt_pkg_share = get_package_share_directory("orchard_slam_bt")

    # Nodes launching commands
    _node_orchard_slam_tree = Node(
        package="orchard_slam_bt",
        executable="orchard_slam_tree",
        name="orchard_slam_tree",
        output="screen",
        emulate_tty=True,  #
        parameters=[
            {"load_map": load_map},
            {"map_name": map_name},
            {"initial_start_pose": initial_start_pose},
            {"record_bag": record_bag},
            # {"use_sim_time": use_sim_time},
        ],
    )

    _to_run = [_node_orchard_slam_tree]

    return _to_run


def generate_launch_description():
    declared_configs = [
        dict(name="load_map", default_value="false", description="Whether to load a map or not"),
        dict(
            name="initial_start_pose",
            default_value="",
            description="Initial pose to start at if loading a map (x, y, theta) in map frame",
        ),
        dict(name="map_name", default_value="orchard_map", description="Name of the map to save"),
        dict(name="record_bag", default_value="false", description="Whether to record a rosbag or not"),
        dict(name="use_sim_time", default_value="true", description="Whether to use simulation time or not"),
    ]

    declared_args = [DeclareLaunchArgument(**config) for config in declared_configs]

    ld = LaunchDescription(declared_args + [OpaqueFunction(function=launch_setup)])

    return ld

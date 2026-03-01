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
logger = rclpy.logging.get_logger('bt_orchard_slam.launch')

import datetime as dt


def launch_setup(context: LaunchContext, *args, **kwargs):
    # Launch configurations
    _map_name = LaunchConfiguration("map_name")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Package directories
    orchard_slam_bt_pkg_share = get_package_share_directory("orchard_slam_bt")

    map_name = _map_name.perform(context) + "_" + dt.datetime.now().strftime("%Y%m%d-%H%M%S")

    # Nodes launching commands
    _node_run_slam_tree = Node(
        package="orchard_slam_bt",
        executable="run_slam_tree",
        name="run_slam_tree",
        output="screen",
        emulate_tty=True,  #
        parameters=[
            {"use_sim_time": use_sim_time},
            {"map_name": map_name}
        ]
    )

    _to_run = [
        _node_run_slam_tree
    ]

    return _to_run


def generate_launch_description():
    declared_configs = [
        dict(name="map_name", default_value="orchard_map", description="Name of the map to save"),
        dict(name="use_sim_time", default_value="false", description="Whether to use simulation time or not"),
    ]

    declared_args = [DeclareLaunchArgument(**config) for config in declared_configs]

    ld = LaunchDescription(declared_args + [OpaqueFunction(function=launch_setup)])

    return ld
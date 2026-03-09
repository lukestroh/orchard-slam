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

import os

import rclpy.logging
logger = rclpy.logging.get_logger('orchard_slam.launch')


def launch_setup(context: LaunchContext, *args, **kwargs):
    # Launch configurations
    load_map = LaunchConfiguration("load_map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")

    # Package directories
    orchard_slam_pkg_share = get_package_share_directory("orchard_slam")
    slam_toolbox_pkg_share = get_package_share_directory("slam_toolbox")

    # localization
    # https://docs.ros.org/en/melodic/api/robot_localization/html/navsat_transform_node.html

    _launch_slam_toolbox_lifelong = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(slam_toolbox_pkg_share, "launch", "lifelong_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            # "odom_frame": "amiga__odom"
        }.items(),
    )

    _node_ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        parameters=[
            os.path.join(orchard_slam_pkg_share, 'config/ekf_local.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('odometry/filtered', 'odometry/filtered/local'),
        ],
    )

    _node_ekf_global = Node( 
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        output='screen',
        parameters=[
            os.path.join(orchard_slam_pkg_share, 'config/ekf_global.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('odometry/filtered', 'odometry/filtered/global'),
        ],
    )

    _node_navsat_transform = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="log",
        parameters=[
            os.path.join(orchard_slam_pkg_share, 'config', 'navsat.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            # ('/imu', '/imu/data'),
            # ('/gps/fix', '/gps/fix'),
            ('/odometry/filtered', '/odometry/filtered/global'),
        ],
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    _to_run = [
        _launch_slam_toolbox_lifelong,
        _node_ekf_local,
        _node_ekf_global,
        _node_navsat_transform,
    ]
    return _to_run

def generate_launch_description():
    declared_configs = [
        dict(name="load_map", default_value="false", choices=["true", "false"], description="Whether to load a map from disk or not"),
        dict(name="use_sim_time", default_value="true", choices=["true", "false"], description="Whether to use simulation time (Gazebo) or real time"),
        dict(name="world", default_value="small_orchard", choices=["small_orchard"], description="The world to load the map for."),
    ]

    declared_args = [DeclareLaunchArgument(**config) for config in declared_configs]

    return LaunchDescription(declared_args + [OpaqueFunction(function=launch_setup)])

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

    # # Nodes launching commands
    # _node_map_saver_server = Node(
    #         package='nav2_map_server',
    #         executable='map_saver_server',
    #         name='map_saver_server',
    #         output='screen',
    #         emulate_tty=True,  # https://github.com/ros2/launch/issues/188
    #         parameters=[{'save_map_timeout': 5.0},
    #                     {'free_thresh_default': 0.25},
    #                     {'occupied_thresh_default': 0.65}]
    # )

    # _node_map_server = Node(
    #     package="nav2_map_server",
    #     executable="map_server",
    #     name="map_server",
    #     parameters=[
    #         {"use_sim_time": use_sim_time},
    #         {
    #             "yaml_filename": os.path.join(
    #                 os.path.join(os.path.expanduser("~"), "maps", world.perform(context=context) + "_map.yaml")
    #             ),
    #         },
    #     ],
    #     condition=IfCondition(load_map),
    # )

    # Build node names list conditionally based on load_map
    node_names = ['map_saver_server']
    if load_map.perform(context) == 'true':
        node_names.append('map_server')

    # _node_nav2_lifecycle_manager = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager',
    #     output='screen',
    #     emulate_tty=True,  # https://github.com/ros2/launch/issues/188
    #     parameters=[{'use_sim_time': use_sim_time},
    #                 {'autostart': True},
    #                 {'node_names': node_names}
    #     ]
    # )

    _to_run = [
        _launch_slam_toolbox_lifelong,
        # _node_nav2_lifecycle_manager,
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

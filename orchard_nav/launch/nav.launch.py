import launch
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable

import rclpy.logging

logger = rclpy.logging.get_logger("orchard_nav.launch")


def launch_setup(context: LaunchContext, *args, **kwargs):
    # Launch configurations
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Package shares
    orchard_navigation_share_dir = get_package_share_directory("orchard_nav")
    nav2_bringup_share_dir = get_package_share_directory("nav2_bringup")

    # Include the Nav2 bringup launch file for the navigation stack
    _launch_nav2_bringup = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(nav2_bringup_share_dir, "launch", "navigation_launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": os.path.join(orchard_navigation_share_dir, "config", "nav2_params.yaml"),
        }.items(),
    )

    _node_orchard_nav = Node(
        package="orchard_nav",
        executable="orchard_nav",
        name="orchard_nav",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    _to_run = [_launch_nav2_bringup, _node_orchard_nav]
    return _to_run


def generate_launch_description():
    # Get the path to the params and launch directories of your packages
    declared_configs = [
        dict(name="use_sim_time", default_value="true", description="Use simulation (Gazebo) clock if true"),
    ]

    declared_args = [DeclareLaunchArgument(**config) for config in declared_configs]

    ld = LaunchDescription(declared_args + [OpaqueFunction(function=launch_setup)])

    return ld

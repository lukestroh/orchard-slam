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
from launch.event_handlers import OnProcessStart, OnProcessExit, OnProcessIO
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
    # General launch configurations
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_slam = LaunchConfiguration("launch_slam")
    world_sdf_file = LaunchConfiguration("world_sdf_file")
    robot_x = LaunchConfiguration("robot_x")
    robot_y = LaunchConfiguration("robot_y")

    # Gazebo-specific launch configurations 
    gazebo_headless = LaunchConfiguration("gazebo_headless")
    sim_gazebo = LaunchConfiguration("sim_gazebo")
    
    # Orchard generation launch configurations
    orchard_name = LaunchConfiguration("orchard_name")
    orchard_seed = LaunchConfiguration("orchard_seed")
    orchard_origin_offset = LaunchConfiguration("orchard_origin_offset")
    tree_namespace = LaunchConfiguration("tree_namespace")
    tree_type = LaunchConfiguration("tree_type")
    num_rows = LaunchConfiguration("num_rows")
    avg_trees_per_row = LaunchConfiguration("avg_trees_per_row")
    avg_tree_spacing = LaunchConfiguration("avg_tree_spacing")
    avg_row_deviation = LaunchConfiguration("avg_row_deviation")
    avg_row_spacing = LaunchConfiguration("avg_row_spacing")
    std_trees_per_row = LaunchConfiguration("std_trees_per_row")
    std_tree_spacing = LaunchConfiguration("std_tree_spacing")
    std_row_deviation = LaunchConfiguration("std_row_deviation")
    std_row_spacing = LaunchConfiguration("std_row_spacing")

    # Package directories
    amiga_description_share = get_package_share_directory("amiga_description")
    orchard_slam_share = get_package_share_directory("orchard_slam")
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
            "orchard_name": orchard_name,
            "orchard_seed": orchard_seed,
            "orchard_origin_offset": orchard_origin_offset,
            "tree_namespace": tree_namespace,
            "tree_type": tree_type,
            "num_rows": num_rows,
            "avg_trees_per_row": avg_trees_per_row,
            "avg_tree_spacing": avg_tree_spacing,
            "avg_row_deviation": avg_row_deviation,
            "avg_row_spacing": avg_row_spacing,
            "std_trees_per_row": std_trees_per_row,
            "std_tree_spacing": std_tree_spacing,
            "std_row_deviation": std_row_deviation,
            "std_row_spacing": std_row_spacing,
            "robot_x": robot_x,
            "robot_y": robot_y,
        }.items(),
        condition=IfCondition(sim_gazebo),
    )

    _launch_orchard_slam = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                orchard_slam_share,
                "launch",
                "slam.launch.py",
            )
        ),
        condition=IfCondition(launch_slam)
    )

    _delay_launch_orchard_slam = TimerAction(
        period=15.0,
        actions=[_launch_orchard_slam]
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
        _delay_launch_orchard_slam,
        _node_rviz,
        _node_joint_state_broadcaster_spawner,
        _node_diff_drive_controller_spawner,
    ]

    return _to_run


def generate_launch_description():
    declared_configs = [
        # General launch configs
        dict(name="launch_rviz", default_value="true", choices=["true", "false"]),
        dict(name="launch_slam", default_value="true", choices=["true", "false"]),
        dict(
            name="world_sdf_file",
            default_value="small_orchard",
            description="Name of the world SDF file (without .sdf extension) to load in Gazebo",
            choices=["room", "small_orchard", "large_orchard"],
        ),
        dict(name="robot_x", default_value="0.0", description="Initial x position of the robot in Gazebo (meters)"),
        dict(name="robot_y", default_value="0.0", description="Initial y position of the robot in Gazebo (meters)"),

        # Gazebo-specific launch configs
        dict(name="gazebo_headless", default_value="false", choices=["true", "false"]),
        dict(name="sim_gazebo", default_value="true", choices=["true", "false"]),

        # Orchard generation launch configs
        dict(name="orchard_name", default_value="small_orchard", description="Name of the orchard configuration to generate (without .yaml extension)"),
        dict(name="orchard_seed", default_value="12", description="Random seed for orchard generation"),
        dict(name="orchard_origin_offset", default_value="(2.0,2.0)", description="Offset of the orchard origin from the world origin (x,y)"),
        dict(name="tree_namespace", default_value="lpy", description="Namespace for all tree entities in Gazebo"),
        dict(name="tree_type", default_value="envy", choices=['envy', 'ufo'], description="Type of tree to generate (e.g. apple, orange, etc.)"),
        dict(name="num_rows", default_value="5", description="Number of rows in the orchard"),
        dict(name="avg_trees_per_row", default_value="10", description="Average number of trees per row in the orchard"),
        dict(name="avg_tree_spacing", default_value="3.0", description="Average spacing between trees in the orchard (meters)"),
        dict(name="std_trees_per_row", default_value="2.0", description="Standard deviation of number of trees per row in the orchard"),
        dict(name="std_tree_spacing", default_value="1.0", description="Standard deviation of spacing between trees in the orchard (meters)"),
        dict(name="avg_row_deviation", default_value="0.2", description="Average deviation from straight rows in the orchard (meters)"),
        dict(name="std_row_deviation", default_value="0.1", description="Standard deviation of row deviation in the orchard (meters)"),
        dict(name="avg_row_spacing", default_value="5.0", description="Average spacing between rows in the orchard (meters)"),
        dict(name="std_row_spacing", default_value="0.5", description="Standard deviation of row spacing in the orchard (meters)"),
    ]

    declared_args = [DeclareLaunchArgument(**config) for config in declared_configs]

    return LaunchDescription(declared_args + [OpaqueFunction(function=launch_setup)])

#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.conditions import UnlessCondition, IfCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node

import orchard_slam_gazebo.spawn_trees as st

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

    _node_gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/gps@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat"
        ],
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

    orchard_config = st.generate_orchard_config(
        orchard_name="small_orchard",
        tree_namespace="lpy",
        tree_type="envy",
        n_rows=5,
        orchard_seed=42,
        avg_trees_per_row=10,
        trees_per_row_std=2,
        avg_tree_spacing=3.0,
        tree_spacing_std=0.5,
        avg_row_deviation=0.2,
        std_row_deviation=0.1,
        avg_row_spacing=5.0,
        row_spacing_std=0.5,
        initial_offset=(2.0, 2.0),
    )
    orchard_sdfs = st.generate_all_tree_sdfs(orchard_config)

    # # Get tree data for spawning
    # test_tree_id = 'lpy_envy_00094'
    # tree_data = orchard_sdfs[test_tree_id]
    # tree_pose = tree_data['pose']  # [x, y]

    tree_spawners = []
    for tree_id, tree_sdf in orchard_sdfs.items():
        _node_spawn_tree = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                '-world', world_sdf_file.perform(context),
                '-string', tree_sdf['sdf'],
                '-x', str(tree_sdf['pose'][0]),
                '-y', str(tree_sdf['pose'][1]),
                '-z', '0',
            ],
            output="screen",
        )
        tree_spawners.append(_node_spawn_tree)
    
    _register_event_handler_delay_tree_spawners_after_gz_launch = []
    for tree_spawner in tree_spawners:
        # _register_event_handler_delay_tree_spawners_after_gz_launch.append(
        #     RegisterEventHandler(
        #         event_handler = OnProcessStart(
        #             target_action=tree_spawner,
        #             on_start=[
        #                 TimerAction(
        #                     period=10.0,
        #                     actions=[tree_spawner],
        #                 ),
        #             ]
        #         ),
        #     )
        # )
        _register_event_handler_delay_tree_spawners_after_gz_launch.append(
            TimerAction(
                period=6.0,
                actions=[tree_spawner],
            ),
        )

    _to_run = [
        _set_env_var_gz_sim_resource_path,
        # _launch_gz_server,
        _launch_gz_sim,
        _node_gz_ros2_bridge,
        _node_spawn_robot,
        # _node_spawn_tree
    ] + _register_event_handler_delay_tree_spawners_after_gz_launch
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

#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import UnlessCondition, IfCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node

import orchard_slam_gazebo.spawn_trees as st

import ast
import os

import rclpy.logging

logger = rclpy.logging.get_logger("orchard_slam_gazebo.launch")


def launch_setup(context: LaunchContext, *args, **kwargs):
    # Launch configurations
    gazebo_headless = LaunchConfiguration("gazebo_headless")
    world_sdf_file = LaunchConfiguration("world_sdf_file")

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
    robot_x = LaunchConfiguration("robot_x")
    robot_y = LaunchConfiguration("robot_y")

    # Package directories
    gz_sim_pkg_share = get_package_share_directory("ros_gz_sim")
    orchard_slam_gazebo_pkg_share = get_package_share_directory("orchard_slam_gazebo")
    orchard_slam_sensors_pkg_share = get_package_share_directory("orchard_slam_sensors")

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
        AnyLaunchDescriptionSource(os.path.join(gz_sim_pkg_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            # headless: "-r -s" (run + server only)
            # with GUI: "-r" (run + gui)
            "gz_args": (
                f"-r -s {world_abs_path}" if gazebo_headless.perform(context) == "true" else f"-r {world_abs_path}"
            ),
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
            "/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
        ],
        output="screen",
        parameters=[
            {
                "qos_overrides./imu.publisher.reliability": "best_effort",
                "qos_overrides./gps/fix.publisher.reliability": "best_effort",
            }
        ],
    )

    _node_static_tf_gps_frame_fix = Node(  # this fixes a weird issue where the gps frame is published as "amiga/amiga__base/navsat_sensor" instead of "amiga__gps_link", which causes the navsat_transform node to not find the gps frame and throw errors
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_gps_frame_fix",
        arguments=["0", "0", "0", "0", "0", "0", "amiga__gps_link", "amiga/amiga__base/navsat_sensor"],
        output="screen",
    )

    _node_imu_covariance_fixer = Node(  # this node subscribes to the raw IMU data from Gazebo, sets the covariance values to something non-zero, and republishes it. This is necessary because Gazebo IMU messages have all covariance values set to zero, which causes issues with many ROS nodes that expect non-zero covariance (e.g. EKFs)
        package="orchard_slam_sensors",
        executable="imu_covariance_fixer_node",
        name="imu_covariance_fixer",
        output="screen",
    )

    # same for gps
    _node_gps_covariance_fixer = Node(
        package="orchard_slam_sensors",
        executable="gps_covariance_fixer_node",
        name="gps_covariance_fixer",
        output="screen",
    )

    _delay_covariance_fixers_after_gz_launch = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=_node_gz_ros2_bridge,
            on_start=[
                TimerAction(
                    period=1.0,
                    actions=[_node_imu_covariance_fixer, _node_gps_covariance_fixer],
                )
            ],
        )
    )

    _node_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-r",
            "-x",
            robot_x,
            "-y",
            robot_y,
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
        orchard_name=orchard_name.perform(context),
        tree_namespace=tree_namespace.perform(context),
        tree_type=tree_type.perform(context),
        n_rows=int(num_rows.perform(context)),
        orchard_seed=int(orchard_seed.perform(context)),
        avg_trees_per_row=float(avg_trees_per_row.perform(context)),
        trees_per_row_std=float(std_trees_per_row.perform(context)),
        avg_tree_spacing=float(avg_tree_spacing.perform(context)),
        tree_spacing_std=float(std_tree_spacing.perform(context)),
        avg_row_deviation=float(avg_row_deviation.perform(context)),
        std_row_deviation=float(std_row_deviation.perform(context)),
        avg_row_spacing=float(avg_row_spacing.perform(context)),
        row_spacing_std=float(std_row_spacing.perform(context)),
        initial_offset=ast.literal_eval(orchard_origin_offset.perform(context)),
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
            name=f"spawn_{tree_id}",
            arguments=[
                "-world",
                world_sdf_file.perform(context),
                "-string",
                tree_sdf["sdf"],
                "-x",
                str(tree_sdf["pose"][0]),
                "-y",
                str(tree_sdf["pose"][1]),
                "-z",
                "0",
            ],
            output="screen",
        )
        tree_spawners.append(_node_spawn_tree)

    _delay_tree_spawners_after_gz_launch = []
    for tree_spawner in tree_spawners:
        _delay_tree_spawners_after_gz_launch.append(
            TimerAction(
                period=6.0,
                actions=[tree_spawner],
            ),
        )

    _to_run = [
        _set_env_var_gz_sim_resource_path,
        # _launch_gz_server,
        _delay_covariance_fixers_after_gz_launch,
        _node_static_tf_gps_frame_fix,
        _launch_gz_sim,
        _node_gz_ros2_bridge,
        _node_spawn_robot,
    ] + _delay_tree_spawners_after_gz_launch
    return _to_run


def generate_launch_description():
    declared_configs = [
        dict(name="gazebo_headless", default_value="false", choices=["true", "false"]),
        dict(name="world_sdf_file", default_value="orchard_template", choices=["orchard_template"]),
        dict(
            name="orchard_name",
            default_value="orchard",
            description="Name of the orchard configuration to generate (without .yaml extension)",
        ),
        dict(name="orchard_seed", default_value="12", description="Random seed for orchard generation"),
        dict(
            name="orchard_origin_offset",
            default_value="(2.0,2.0)",
            description="Offset of the orchard origin from the world origin (x,y)",
        ),
        dict(name="tree_namespace", default_value="lpy", description="Namespace for all tree entities in Gazebo"),
        dict(
            name="tree_type",
            default_value="envy",
            choices=["envy", "ufo"],
            description="Type of tree to generate (e.g. apple, orange, etc.)",
        ),
        dict(name="num_rows", default_value="5", description="Number of rows in the orchard"),
        dict(
            name="avg_trees_per_row", default_value="10", description="Average number of trees per row in the orchard"
        ),
        dict(
            name="avg_tree_spacing",
            default_value="3.0",
            description="Average spacing between trees in the orchard (meters)",
        ),
        dict(
            name="std_trees_per_row",
            default_value="2.0",
            description="Standard deviation of number of trees per row in the orchard",
        ),
        dict(
            name="std_tree_spacing",
            default_value="1.0",
            description="Standard deviation of spacing between trees in the orchard (meters)",
        ),
        dict(
            name="avg_row_deviation",
            default_value="0.2",
            description="Average deviation from straight rows in the orchard (meters)",
        ),
        dict(
            name="std_row_deviation",
            default_value="0.1",
            description="Standard deviation of row deviation in the orchard (meters)",
        ),
        dict(
            name="avg_row_spacing",
            default_value="5.0",
            description="Average spacing between rows in the orchard (meters)",
        ),
        dict(
            name="std_row_spacing",
            default_value="0.5",
            description="Standard deviation of row spacing in the orchard (meters)",
        ),
        dict(name="robot_x", default_value="0.0", description="Robot spawn X position (meters)"),
        dict(name="robot_y", default_value="0.0", description="Robot spawn Y position (meters)"),
    ]

    declared_args = [DeclareLaunchArgument(**config) for config in declared_configs]

    return LaunchDescription(declared_args + [OpaqueFunction(function=launch_setup)])

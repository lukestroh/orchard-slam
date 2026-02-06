#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch.actions import TimerAction
import os

import rclpy.logging

logger = rclpy.logging.get_logger("amiga.launch")


def setup_launch(context, *args, **kwargs):
    # Launch configurations
    amiga_prefix = LaunchConfiguration("amiga_prefix")
    sim_gazebo = LaunchConfiguration("sim_gazebo")
    view_robot = LaunchConfiguration("view_robot")

    # Your xacro
    amiga_share = get_package_share_directory("amiga_description")
    xacro_file = os.path.join(amiga_share, "urdf", "amiga.urdf.xacro")

    # Publish robot_description (still useful for TF and for spawning via topic)
    robot_description = ParameterValue(
        Command(
            [
                "xacro",
                " ",
                xacro_file,
                " ",
                "amiga_prefix:=",
                amiga_prefix.perform(context),
                " ",
                "sim_gazebo:=",
                sim_gazebo.perform(context),
            ]
        ),
        value_type=str,
    )
    # logger.info(f"Robot description: {robot_description.value[0].perform(context)}")
    amiga_controllers = os.path.join(amiga_share, "config", "amiga_controllers.yaml")

    _node_ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            ParameterFile(amiga_controllers, allow_substs=True),
        ],
        condition=UnlessCondition(sim_gazebo),
        output="screen",
    )

    _node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    _node_joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui", executable="joint_state_publisher_gui", condition=IfCondition(view_robot)
    )

    _node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(get_package_share_directory("orchard_slam_bringup"), "rviz", "orchard_slam.rviz"),
        ],
        parameters=[{"robot_description": robot_description}],
        condition=IfCondition(view_robot),
    )

    _to_run = [
        _node_ros2_control,
        _node_robot_state_publisher,
        _node_joint_state_publisher_gui,
        _node_rviz,
    ]

    return _to_run


def generate_launch_description():

    declared_configs = [
        dict(name="amiga_prefix", default_value="amiga", description="Prefix for the robot name"),
        dict(
            name="sim_gazebo",
            default_value="true",
            choices=["true", "false"],
            description="Whether we are simulating in Gazebo",
        ),
        dict(
            name="view_robot",
            default_value="false",
            choices=["true", "false"],
            description="Whether to launch RViz to view the robot",
        ),
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

    ld = LaunchDescription(declared_args + [OpaqueFunction(function=setup_launch)])
    return ld

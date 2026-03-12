import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    # Get the path to the params and launch directories of your packages
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    orchard_navigation_share_dir = get_package_share_directory('orchard_nav')
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    param_dir = LaunchConfiguration(
        'params',
        default=os.path.join(orchard_navigation_share_dir, 'params', 'nav2_params.yaml')
    )

    # Declare the launch arguments
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_SEVERITY', 'DEBUG'), 

        # Use simulation time (for Gazebo)
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # Parameter file to be loaded
        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to parameter file to load'
        ),

        # Include the Nav2 bringup launch file for the navigation stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': param_dir
            }.items(),
        ),

        # Launch the robot behavior node (if you have a custom robot behavior script)
        # Node(
        #     package='orchard_nav',
        #     executable='robot_behavior',
        #     name='robot_behavior',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time}],
        # ),

    ])
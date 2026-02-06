from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.substitutions import Command
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import FindExecutable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("amiga_description")

    # Use xacro as the source of truth
    # xacro_file = os.path.join(pkg_share, 'urdf', 'amiga.urdf.xacro')
    world_file = "/usr/share/gazebo-11/worlds/empty.world"

    # Workspace root that contains the folder "amiga_description/"
    # This makes Gazebo resolve model://amiga_description/...
    # ws_root = os.path.abspath(os.path.join(pkg_share, '..', '..', '..'))  # .../install -> workspace-ish
    # If the above doesn't point to your src workspace, hardcode it:
    ws_root = "/home/chris/Documents/void/homeworks/MobileRob/Project/orchard-slam"

    # Build robot_description by running xacro
    xacro_file = PathJoinSubstitution([FindPackageShare("amiga_description"), "urdf", "amiga.urdf.xacro"])

    robot_description = Command([FindExecutable(name="xacro"), " ", xacro_file])

    # Start Gazebo server (stable)
    gzserver = ExecuteProcess(
        cmd=["gzserver", "--verbose", "-s", "libgazebo_ros_init.so", "-s", "libgazebo_ros_factory.so", world_file],
        output="screen",
    )

    # Optional GUI. If your GUI keeps failing, comment this out and run gzclient manually.
    gzclient = ExecuteProcess(cmd=["gzclient"], output="screen")

    # Publish robot_description (gazebo_ros2_control is waiting for this)
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True, "robot_description": robot_description}],
    )

    # Spawn from robot_description topic
    spawn_amiga = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=["-entity", "amiga", "-topic", "robot_description", "-x", "0", "-y", "0", "-z", "0.3"],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            # Avoid Fuel DB hang + make model:// resolve
            SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
            SetEnvironmentVariable(
                "GAZEBO_MODEL_PATH",
                "/usr/share/gazebo-11/models:" + ws_root + ":" + os.environ.get("GAZEBO_MODEL_PATH", ""),
            ),
            gzserver,
            gzclient,  # comment out if it crashes
            rsp,
            spawn_amiga,
        ]
    )

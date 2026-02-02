# TODO

1. Add diff drive to robot. This doesn't seem well defined in ros2. Links below
    - https://robotics.stackexchange.com/questions/107952/how-to-make-4-wheeled-robot-work-with-diffdrive-plugin-and-ros2-control
    - https://control.ros.org/humble/doc/ros2_control_demos/example_2/doc/userdoc.html
    - https://github.com/ros-controls/ros2_controllers/blob/humble/diff_drive_controller/src/diff_drive_controller_parameter.yaml
  Found a couple useful links (Eva):
    - https://www.reddit.com/r/ROS/comments/1hayci3/differential_drive_robot_with_ros_2_jazzy_jalisco/
    - https://docs.ros.org/en/rolling/p/gazebo_plugins/generated/classgazebo__plugins_1_1GazeboRosDiffDrive.html
1. Integrate with Gazebo plugin.
    - I have started this in the URDF. Not sure how to connect it to the Gazebo launch yet. 
1. NOT FOR FINAL PROJECT: Write new hardware interface:
    - https://github.com/ros-controls/ros2_control_demos/blob/humble/example_2/hardware/diffbot_system.cpp

    

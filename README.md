# Orchard Slam

This is a set of ROS2 Kilted packages that allow a Farm-ng Amiga to navigate through an orchard.

NOTE: Currently only developing for Gazebo.

## Helpful CLI args:

**Start the telop twist keyboard**
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r cmd_vel:=/diff_drive_controller/cmd_vel
```

## Installation
### Installing `slam_toolbox`
```
git clone -b humble_lifecycle git@github.com:SteveMacenski/slam_toolbox.git
```
Next, go into the CMakeLists.txt and comment out the following line:
```
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  # PATTERN "lifelong_launch.py" EXCLUDE
)
```

### Adding tree meshes
Unzip the meshes.zip file under orchard_slam_gazebo/meshes.

### Other installations
Add any other missing packages using:

```
rosdep install --from-paths src --ignore-src -r -y
```


## Launching the files
To run the complete package, all launches should be run from the `orchard_slam_bringup` package.
```
ros2 launch orchard_slam_bringup orchard_slam_bringup.launch.py
```

## Package structure

### amiga_description
Processes the Amiga xacro and launches the relevant ros2_control controllers.

### orchard_slam
Runs the relevant SLAM and navigation processes.

### orchard_slam_bringup
High level package that launches all subpackages.

### orchard_slam_bt
Behavior tree package for executing action clients. Uses PyTrees.

### orchard_slam_gazebo
Launches gz_server and optionally, gz_sim GUI. This package also retains LPy tree information for creating orchards. Orchards are defined in the 'worlds' directory.

### orchard_msgs

### ros2bag_msgs



## External packages

### slam_toolbox
`git@github.com:SteveMacenski/slam_toolbox.git`

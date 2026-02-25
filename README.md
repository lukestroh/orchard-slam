# Orchard Slam

This is a set of ROS2 Kilted packages that allow a Farm-ng Amiga to navigate through an orchard.

NOTE: Currently only developing for Gazebo.

## Installation
### Installing `slam_toolbox`
```
git clone -b humble_lifecycle git@github.com:SteveMacenski/slam_toolbox.git
```
Next, go into the CMakeLists.txt and uncomment the following line:
```
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  # PATTERN "lifelong_launch.py" EXCLUDE
)
```

### Other installations
Add any other missing packages using:

```
rosdep install --from-paths src --ignore-src -r -y
```


## Launching the files
To run the complete package, all launches should be run from the `orchard_slam_bringup` package.
```
ros2 launch orchard_slam_bringup orchard_slam_brinup.launch.py
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

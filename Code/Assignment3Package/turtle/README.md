# Turtle ROS2 Package Structure

## Package Overview
This ROS2 package implements a complete autonomous robot system with navigation, SLAM, and object detection capabilities. Below is a detailed breakdown of each directory and its contents:

## Directory Structure

### ColourDetect
- Contains OpenCV-based image recognition programs
- Implements visual object detection and classification
- Processes camera feed for target identification

### config
Configuration files for various components:
- **slam_toolbox**: Parameters for SLAM implementation
- **nav2**: Navigation stack configuration
- **twist_mux**: Velocity multiplexer parameters for control source selection

### gazebo
- Contains sensor definitions for the robot
- Includes gazebo plugin configurations
- Defines sensor properties and placements

### launch
- **Rlaunch.py**: Main launch file
  - Initializes all necessary nodes
  - Sets up robot configuration
  - Launches required tools and utilities

### meshes
- STL files for each robot link
- Used for visual representation and collision detection
- Referenced in URDF definitions

### rviz
- RViz configuration files
- Defines initial visualization setup
- Specifies which components and displays to load
- Configures visualization parameters

### scripts
- Autonomous exploration implementation
- Alternative pure simulation target detection
- Navigation programs
- Python-based control scripts

### urdf
- Complete robot URDF definitions
- Motion plugin configurations
- Joint and link specifications
- Sensor mount definitions

### worlds
- Various experimental Gazebo world files
- Original Blender-exported DAE model files
- Different testing scenarios
- Environmental setups for various experiments

## Usage
The package is designed to be launched primarily through the main launch file:
```
ros2 launch turtle Rlaunch.py
```

## Dependencies
- ROS2 Navigation Stack (Nav2)
- SLAM Toolbox
- Gazebo
- OpenCV
- RViz2
- Twist Multiplexer 
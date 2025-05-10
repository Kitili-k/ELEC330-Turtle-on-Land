# turtle_on_land_demo ROS 2 Package Instructions

This document provides instructions for setting up and running the turtle_on_land_demo ROS 2 package.

## Prerequisites

- ROS 2 installed (Humble or newer recommended)
- Gazebo simulator installed
- ros_gz package installed

## Setup and Build

1. Navigate to your ROS 2 workspace root:
   ```
   cd ~/ros2_ws
   ```

2. Build the package:
   ```
   colcon build
   ```

3. Source the workspace:
   ```
   source ~/ros2_ws/install/setup.bash
   ```

## Running the Demo

1. Launch the simulation:
   ```
   ros2 launch turtle_on_land_demo launch.py
   ```

## Notes

- If you encounter resource path issues, check if the GZ_SIM_RESOURCE_PATH environment variable is set correctly.

## Troubleshooting

If you encounter issues:
1. Verify all dependencies are correctly installed.
2. Ensure ROS 2 and Gazebo versions are compatible.
3. Check ROS 2 and Gazebo log outputs for error messages.



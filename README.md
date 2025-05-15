# ELEC330_TurtleOnLand
# Robotic turtle on Land

## Assignment 3 briefing

This assignment focuses on developing an autonomous robot system with advanced navigation and object recognition capabilities. The key requirements include:

- Collaborative teamwork to integrate various robotic functionalities
- Building upon concepts implemented in assignments one and two
- Enhanced robot design for safe navigation and object recognition
- Implementation of autonomous navigation within a defined arena using sensors
- Object detection and parking functionality - the robot must identify and park next to a specified object
- ROS integration for sensor data broadcasting and processing
- Environmental mapping capabilities
- Safe autonomous navigation implementation
- Demonstration of robust object recognition
- Development of a comprehensive bill of materials including suppliers, catalogue codes, quantities, and pricing
- Individual contribution reports detailing team members' roles and their impact on overall robot performance

The project demonstrates these capabilities through simulated environments where our robot, Shelly, showcases autonomous navigation, object detection, mapping, and safe interaction with its surroundings.

## Navigation result    
![Navigation result](Reports/Assignment3/supportMaterial/Image/Pasted%20image%20(4).png)

## GitHub arrangment

- #### Code folder:
&emsp;&emsp;The Assignment 3 package is under Assignment3Package folder called 'turtle'

- #### Report folder:
&emsp;&emsp; The report for assignemnt 3 is under the Assignment3 folder where also contains the supporting evidence in Assignment3/supportMaterial folder.

- #### Videos folder:
&emsp;&emsp; The videos for this assignment is under the Assignment3 folder.

## requirement to build

- Move the 'turtle' folder into ~/your_workspace/src
- Run the following command in the terminal:

```
cd ~/your_workspace
colcon build
```

- Used ROS2 dependencies

1. Core ROS2 Dependencies:
   - robot_state_publisher
   - rclpy
   - geometry_msgs
   - vision_msgs

2. Simulation Dependencies:
   - ros_gz_sim
   - ros_gz_bridge

3. Navigation Stack (Nav2):
   - nav2_bringup
   - nav2_bt_navigator
   - nav2_costmap_2d
   - nav2_controller
   - nav2_planner
   - nav2_behaviors
   - nav2_regulated_pure_pursuit_controller
   - nav2_map_server
   - nav2_lifecycle_manager
   - nav2_waypoint_follower
   - nav2_velocity_smoother
   - nav2_common
   - nav2_smoother
   - nav2_msgs

4. SLAM and Transformation:
   - slam_toolbox
   - tf2
   - tf2_ros
   - tf2_msgs

5. Auto explore
    - inspired by https://github.com/AniArka/Autonomous-Explorer-and-Mapper-ros2-nav2


## Run instruction

- Run the following command in the terminal:

```
cd ~/your_workspace
source install/setup.bash
ros2 launch turtle Rlaunch.py
```
Open another terminal and run the following command:

```
cd ~/your_workspace/src/turtle/scripts
ros2 run turtle auto_explore.py
```


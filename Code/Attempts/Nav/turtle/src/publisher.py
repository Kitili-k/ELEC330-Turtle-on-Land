#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, TransformStamped
import time

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')

        # Publisher for the /joint_vel/commands topic
        self.publisher = self.create_publisher(Float64MultiArray, '/joint_vel/commands', 15)

        # Subscriber for the /scan2 topic (LiDAR)
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan2',
            self.lidar_callback,
            10
        )

        # Subscriber for the /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Predefined velocity patterns for the robot actuators
        self.velocities = {
            'stop': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'straight_forward': [-20.0, 0, 20.0, 0, -20.0, 0.0, 20.0, 0.0],
            'straight_reverse': [10.0, -1.0, -10.0, 1.0, 10.0, -1.0, -10.0, 1.0],
            'retract': [15.0, -7.0, -15.0, 7.0, 15.0, -7.0, -15.0, 7.0],
            'pushdown': [-0.0, 16.0, 0.0, -16.0, 0.0, 16.0, 0.0, -16.0],
            'left_forward': [-30.0, 0.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0],
            'left_reverse': [15.0, -1.0, 0.0, 0.0, -15.0, 1.0, 0.0, 0.0],
            'left_retract': [15.0, -10.0, 0, 0, 15.0, -10.0, 0, 0],
            'left_pushdown': [-0.0, 20.0, 0.0, 0, 0.0, 20.0, 0.0, 0],
            'right_forward': [0, 0, 30, 0, 0, 0, -30.0, 0],
            'right_reverse': [0, 0, -15, 1.0, 0, 0, -15.0, 1.0],
            'right_retract': [0, 0, -15.0, 10.0, 0, 0, -15.0, 15.0],
            'right_pushdown': [-0.0, 0, 0.0, -20.0, 0.0, 0, 0.0, -20.0],
        }

        # variables
        self.direction = 'straight_forward'
        self.object_detected = False
        self.toggle_state = True
        self.flag = 0

        # Timer to publish velocities
        self.timer = self.create_timer(0.5, self.update)  # Publish at 2 Hz
        self.get_logger().info('Velocity Publisher Node has started.')

    def lidar_callback(self, msg):
        """Callback to process LiDAR data."""
        min_distance = min(msg.ranges)  # Get the closest object distance
        self.get_logger().info(f'Minimum distance from object: {min_distance} meters')

        if min_distance < 2:  # If object is within 2 meters
            self.direction = 'right_forward'
        else:
            self.direction = 'straight_forward'

    def cmd_vel_callback(self, msg):
        """Callback to handle /cmd_vel messages."""
        linear = msg.linear.x
        angular = msg.angular.z

        # Map linear and angular velocities to predefined directions
        if linear > 0:
            self.direction = 'straight_forward'
        elif linear < 0:
            self.direction = 'straight_reverse'
        elif angular > 0:
            self.direction = 'left_forward'
        elif angular < 0:
            self.direction = 'right_forward'
        else:
            self.direction = 'stop'


    def update(self):
        """Publish velocities and ensure proper sequence."""
        velocity_command = Float64MultiArray()

        if self.direction in ['straight_forward', 'straight_reverse']:
            if self.toggle_state:
                velocity_command.data = self.velocities['straight_forward']
                self.flag = 1
                self.toggle_state = not self.toggle_state
            elif not self.toggle_state and self.flag == 1:
                velocity_command.data = self.velocities['pushdown']
                self.flag = 2
            elif not self.toggle_state and self.flag == 2:
                velocity_command.data = self.velocities['straight_reverse']
                self.flag = 0
            elif not self.toggle_state and self.flag == 0:
                velocity_command.data = self.velocities['retract']
                self.toggle_state = not self.toggle_state
        elif self.direction in ['left_forward', 'left_reverse']:
            if self.toggle_state:
                velocity_command.data = self.velocities['left_forward']
                self.flag = 1
                self.toggle_state = not self.toggle_state
            elif not self.toggle_state and self.flag == 1:
                velocity_command.data = self.velocities['left_pushdown']
                self.flag = 2
            elif not self.toggle_state and self.flag == 2:
                velocity_command.data = self.velocities['left_reverse']
                self.flag = 0
            elif not self.toggle_state and self.flag == 0:
                velocity_command.data = self.velocities['left_retract']
                self.toggle_state = not self.toggle_state
        elif self.direction in ['right_forward', 'right_reverse']:
            if self.toggle_state:
                velocity_command.data = self.velocities['right_forward']
                self.flag = 1
                self.toggle_state = not self.toggle_state
            elif not self.toggle_state and self.flag == 1:
                velocity_command.data = self.velocities['right_pushdown']
                self.flag = 2
            elif not self.toggle_state and self.flag == 2:
                velocity_command.data = self.velocities['right_reverse']
                self.flag = 0
            elif not self.toggle_state and self.flag == 0:
                velocity_command.data = self.velocities['right_retract']
                self.toggle_state = not self.toggle_state
        else:
            velocity_command.data = self.velocities['stop']

        self.publisher.publish(velocity_command)
        time.sleep(0.7)  # Ensure command stability

def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher()

    try:
        rclpy.spin(velocity_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        velocity_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

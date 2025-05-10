#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist


class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')

        # Publisher for the /joint_vel/commands topic
        self.publisher = self.create_publisher(Float64MultiArray, '/joint_vel/commands', 10)

        # Subscriber for the /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        # Joints Front_right1, Front_right2, Front_left1, Front_left2, Back_right1, Back_right2, Back_left1, Back_left2
        # Predefined velocity patterns
        self.velocities = {
            'stop': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'straight_forward': [5.0, 5.0, 5.0, 5.0, 3.0, 0.0, 3.0, 0.0],  # Symmetric velocities for forward movement
            'straight_reverse': [-4.0, -4.0, -4.0, -4.0, -3.0, 0.0, -3.0, 0.0],  # Symmetric velocities for reverse movement
            'left_forward': [4.0, 4.0, 3.0, 3.0, -3.0, -3.0, 3.0, 3.0],
            'left_reverse': [-4.0, -4.0, -3.0, -3.0, 3.0, 3.0, -3.0, -3.0],
            'right_forward': [3.0, 3.0, 4.0, 4.0, 3.0, 3.0, -3.0, -3.0],
            'right_reverse': [-3.0, -3.0, -4.0, -4.0, -3.0, -3.0, 3.0, 3.0]
        }

        # Current direction
        self.direction = 'stop'

        # State toggle for flipping effect
        self.toggle_state = True

        # Timer to publish velocities
        self.timer = self.create_timer(0.5, self.publish_velocity)  # Publish at 2 Hz
        self.get_logger().info('Velocity Publisher Node has started.')

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

        self.get_logger().info(f'Direction changed to: {self.direction}')

    def publish_velocity(self):
        """Publish velocities based on the current direction."""
        velocity_command = Float64MultiArray()

        # Applying flipping effect for straight, left, and right directions
        if self.direction in ['straight_forward', 'straight_reverse']:
            if self.toggle_state:
                velocity_command.data = self.velocities['straight_forward']
            else:
                velocity_command.data = self.velocities['straight_reverse']
        elif self.direction in ['left_forward', 'left_reverse']:
            if self.toggle_state:
                velocity_command.data = self.velocities['left_forward']
            else:
                velocity_command.data = self.velocities['left_reverse']
        elif self.direction in ['right_forward', 'right_reverse']:
            if self.toggle_state:
                velocity_command.data = self.velocities['right_forward']
            else:
                velocity_command.data = self.velocities['right_reverse']
        else:
            # Default stop velocity
            velocity_command.data = self.velocities['stop']

        # Toggle the state for the next cycle
        self.toggle_state = not self.toggle_state

        # Publish the velocity command
        self.get_logger().info(f'Publishing velocity command: {velocity_command.data}')
        self.publisher.publish(velocity_command)


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

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster
import math


class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')

        # Publisher for the /odom topic
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber for velocity commands
        self.velocity_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.velocity_callback,
            10
        )

        # Odometry variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.last_time = self.get_clock().now()

        # Timer for periodic updates
        self.timer = self.create_timer(0.05, self.update_odometry)  # 20 Hz

        self.get_logger().info("Odometry Publisher Node has started.")

    def velocity_callback(self, msg: Twist):
        """Callback to update velocities from /cmd_vel topic."""
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
        self.get_logger().info(f"Received velocity: linear={self.linear_velocity}, angular={self.angular_velocity}")

    def update_odometry(self):
        """Compute and publish odometry based on the received velocity commands."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        # Compute change in position
        delta_x = self.linear_velocity * math.cos(self.theta) * dt
        delta_y = self.linear_velocity * math.sin(self.theta) * dt
        delta_theta = self.angular_velocity * dt

        # Update odometry
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # Velocity
        odom.twist.twist.linear.x = self.linear_velocity
        odom.twist.twist.angular.z = self.angular_velocity

        self.odom_publisher.publish(odom)

        # Broadcast transform
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = math.sin(self.theta / 2.0)
        transform.transform.rotation.w = math.cos(self.theta / 2.0)

        self.tf_broadcaster.sendTransform(transform)

        # Update time
        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

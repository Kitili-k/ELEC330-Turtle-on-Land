#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import numpy as np


class SimpleSLAMNode(Node):
    def __init__(self):
        super().__init__('simple_slam_node')

        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publishers and subscribers
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.process_scan, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Timer for broadcasting transforms
        self.create_timer(0.01, self.broadcast_transform)  # 100 Hz update rate

        # Variable to store current odometry data
        self.current_odom = None

        self.get_logger().info("Simple SLAM Node Initialized")

    def odom_callback(self, odom_msg):
        # Store the current odometry data
        self.current_odom = odom_msg.pose.pose

    def process_scan(self, scan_msg):
        if not scan_msg.ranges:
            self.get_logger().error("LaserScan message has no range data!")
            return

        # Create the OccupancyGrid message
        map_msg = OccupancyGrid()
        map_msg.header.stamp = scan_msg.header.stamp
        map_msg.header.frame_id = "map"
        map_msg.info.resolution = 0.1
        map_msg.info.width = 100
        map_msg.info.height = 100
        map_msg.info.origin.position.x = -5.0
        map_msg.info.origin.position.y = -5.0

        # Initialize map data with -1 (unknown)
        map_data = [-1] * (map_msg.info.width * map_msg.info.height)

        # Populate map data using LaserScan ranges
        angle = scan_msg.angle_min
        for r in scan_msg.ranges:
            if scan_msg.range_min < r < scan_msg.range_max:
                x = int((r * math.cos(angle)) / map_msg.info.resolution + map_msg.info.width / 2)
                y = int((r * math.sin(angle)) / map_msg.info.resolution + map_msg.info.height / 2)
                if 0 <= x < map_msg.info.width and 0 <= y < map_msg.info.height:
                    index = x + y * map_msg.info.width
                    map_data[index] = 100
            angle += scan_msg.angle_increment

        if all(value == -1 for value in map_data):
            self.get_logger().warn("Map data is empty. Ensure valid LaserScan data is received.")

        # Publish the map
        map_msg.data = map_data
        self.map_publisher.publish(map_msg)
        self.get_logger().info("Map updated with LaserScan data.")

    def broadcast_transform(self):
        if self.current_odom is not None:
            # Synchronized timestamp
            timestamp = self.get_clock().now().to_msg()

            # Log timestamps for debugging
            self.get_logger().debug(f"Broadcasting transforms with timestamp: {timestamp.sec}.{timestamp.nanosec}")

            # Dynamic transform: map -> odom
            map_to_odom_transform = TransformStamped()
            map_to_odom_transform.header.stamp = timestamp
            map_to_odom_transform.header.frame_id = "map"
            map_to_odom_transform.child_frame_id = "odom"
            map_to_odom_transform.transform.translation.x = self.current_odom.position.x
            map_to_odom_transform.transform.translation.y = self.current_odom.position.y
            map_to_odom_transform.transform.translation.z = self.current_odom.position.z
            map_to_odom_transform.transform.rotation = self.current_odom.orientation
            self.tf_broadcaster.sendTransform(map_to_odom_transform)

            # Dynamic transform: odom -> Sensor_link
            sensor_link_transform = TransformStamped()
            sensor_link_transform.header.stamp = timestamp
            sensor_link_transform.header.frame_id = "odom"
            sensor_link_transform.child_frame_id = "Sensor_link"
            sensor_link_transform.transform.translation.x = 0.0
            sensor_link_transform.transform.translation.y = 0.0
            sensor_link_transform.transform.translation.z = 0.0
            sensor_link_transform.transform.rotation.x = 0.0
            sensor_link_transform.transform.rotation.y = 0.0
            sensor_link_transform.transform.rotation.z = 0.0
            sensor_link_transform.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(sensor_link_transform)

            self.get_logger().info("Broadcasted dynamic transform for Sensor_link.")


def main(args=None):
    rclpy.init(args=args)
    node = SimpleSLAMNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
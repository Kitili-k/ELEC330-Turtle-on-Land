#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray

class BoundingBoxController(Node):
    def __init__(self):
        super().__init__('bounding_box_controller')
        
        # create subscriber
        self.subscription = self.create_subscription(
            Detection2DArray,
            '/camera',
            self.bounding_box_callback,
            10)
        
        # create publisher
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel_target',
            10)
        
        # image size parameters (match boundary box camera configuration)
        self.image_width = 800
        self.image_height = 600
        
        # control parameters
        self.max_linear_speed = 0.3  # maximum linear speed (m/s)
        self.max_angular_speed = 2  # maximum angular speed (rad/s)
        self.target_label = 10  # target object label ID
        
        self.get_logger().info('Bounding box controller started')
        
    def bounding_box_callback(self, msg):
        """process bounding box message and calculate control command"""
        # check if there is any detection result
        if not msg.detections:
            return
        
        # find the bounding box with matching target label
        target_detection = None
        for detection in msg.detections:
            target_detection = detection
            break
        
        if target_detection is None:
            return
        
        # get center point directly from center field
        center_x = target_detection.bbox.center.position.x
        center_y = target_detection.bbox.center.position.y
        width_x = target_detection.bbox.size_x        
        # self.get_logger().info(f'target center point: ({center_x}, {center_y})')
        
        # calculate control command
        linear_vel, angular_vel = self.calculate_velocity(center_x, center_y, width_x)
        
        # publish velocity command
        self.publish_velocity(linear_vel, angular_vel)
    
    def calculate_velocity(self, center_x, center_y, width_x):
        """calculate robot movement speed based on the center point of the bounding box"""
        # calculate the offset of the center point relative to the image center
        x_offset = center_x - (self.image_width / 2)
        
        # normalize the offset to the range of [-1, 1]
        normalized_x_offset = x_offset / (self.image_width / 2)
        
        # calculate the target height (for estimating distance)
        # the larger the y value, the closer the target is to the bottom of the image
        normalized_y = center_y / self.image_height
        
        estimate_distance = width_x/(0.4*self.image_width )
        
        # calculate the angular velocity - negative value represents right turn, positive value represents left turn
        # when the target is in the center, the angular velocity is close to zero
        angular_vel = -self.max_angular_speed * normalized_x_offset
        linear_vel = self.max_linear_speed * (1.0 - estimate_distance)

        if normalized_x_offset<=-0.5 or normalized_x_offset >= 0.5:
            linear_vel = self.max_linear_speed*(1- abs(normalized_x_offset))
        
        return linear_vel, angular_vel
    
    def publish_velocity(self, linear_vel, angular_vel):
        """publish velocity command message"""
        msg = Twist()
        
        # set linear velocity (forward/backward)
        msg.linear.x = linear_vel
        
        # set angular velocity (steering)
        msg.angular.z = angular_vel
        
        self.publisher.publish(msg)
        # self.get_logger().info(f'publish velocity command: linear velocity={linear_vel:.2f}, angular velocity={angular_vel:.2f}')
    
    def stop_robot(self):
        """stop the robot"""
        self.publish_velocity(0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = BoundingBoxController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

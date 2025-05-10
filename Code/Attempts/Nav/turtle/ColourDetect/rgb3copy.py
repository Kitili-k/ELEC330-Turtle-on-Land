#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import statistics

class ImageProcessingNode(Node):
    def __init__(self):
        super().__init__('image_processing_node')
        
        # Create subscriber for camera image
        self.image_sub = self.create_subscription(
            Image,
            '/camera_image',
            self.image_callback,
            10
        )
        
        # Create publisher for processed image
        self.result_pub = self.create_publisher(
            Image,
            '/imageResult',
            10
        )
        
        # Initialize CV bridge for ROS <-> OpenCV conversion
        self.bridge = CvBridge()
        
        # Define color detection parameters (pink color range)
        self.lower_bound = np.array([0, 0, 141])
        self.upper_bound = np.array([255, 115, 255])
        
        # Initialize image processing parameters
        self.min_area = 100  # Minimum contour area to filter noise
        
        self.get_logger().info('Image processing node initialized')

    def image_callback(self, msg):
        """
        Callback function for processing incoming camera images
        Args:
            msg: ROS Image message from camera
        """
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Create a copy of original image for drawing
            result_image = cv_image.copy()
            
            # Apply color mask to detect target color
            mask = cv2.inRange(cv_image, self.lower_bound, self.upper_bound)
            
            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Process detected contours
            if contours:
                # Find the largest contour
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                
                if area > self.min_area:
                    # Calculate contour center
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # Draw contour and center point
                        cv2.drawContours(result_image, [largest_contour], -1, (0, 255, 0), 2)
                        cv2.circle(result_image, (cx, cy), 5, (0, 0, 255), -1)
                        
                        # Add text with area information
                        cv2.putText(result_image, 
                                  f'Area: {area:.2f}', 
                                  (cx - 50, cy - 20),
                                  cv2.FONT_HERSHEY_SIMPLEX,
                                  0.5,
                                  (255, 255, 255),
                                  2)
                        
                        self.get_logger().debug(f'Target detected - Area: {area:.2f}, Center: ({cx}, {cy})')
            
            # Convert processed image back to ROS message
            result_msg = self.bridge.cv2_to_imgmsg(result_image, "bgr8")
            result_msg.header = msg.header  # Preserve original timestamp and frame_id
            
            # Publish processed image
            self.result_pub.publish(result_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    # Initialize ROS node
    rclpy.init(args=args)
    
    # Create and spin the node
    node = ImageProcessingNode()
    rclpy.spin(node)
    
    # Clean up
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
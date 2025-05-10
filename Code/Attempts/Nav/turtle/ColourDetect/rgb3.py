#!/usr/bin/env python3
from asyncio import sleep
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
import PIL
import statistics
from std_msgs.msg import String
import matplotlib.pyplot as plt
import argparse
import time

# define the list of RBG boundaries that pixels are positivly identified within
                              #Changing this will basically loosed and tighten the colour requirements
                              #to be detected, in theory this needs to be kept tight, but you could
                              #expand it to include more colours if had to loosen the confidence interval
                              #below for some reason

boundaries = [([0, 0, 141], [255, 115, 255])]
LR = 0
width = 800
height = 600

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
            Int32,
            '/imageResult',
            10
        )
        
        # Initialize CV bridge for ROS <-> OpenCV conversion
        self.bridge = CvBridge()
        
        self.get_logger().info('Image processing node initialized')
        #cv2.namedWindow("My image", cv2.WINDOW_NORMAL)
        #cv2.namedWindow("Check image", cv2.WINDOW_NORMAL)

    def image_callback(self, msg):
        #self.get_logger().info(f'Image data received! it has {msg.encoding}')
        #self.get_logger().info(f'OpenCV backend {cv2.getBuildInformation()}')
        try:
            """
                  Callback function for processing incoming camera images
                  Args:
                        msg: ROS Image message from camera
                  """
                              # Convert ROS Image message to OpenCV format
                        

            image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            #self.get_logger().info(f' imageshape {image.shape} and dtype {image.dtype}')
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            #This is just to check if it's subscribing right
            #camera_check = PIL.Image.new(mode="RGB", size=(width, height))
            #self.result_pub = camera_check.load()
            #for i in range(10):
            
            #cv2.imwrite("debug.png", image)

            cv2.imshow("My image", image)
            cv2.waitKey(1)
              #    time.sleep(5)

                              # loop over the boundaries
        except Exception as e:
            self.get_logger().error(f"Error converting image {e}")
        
        global LR

        for (lower, upper) in boundaries:
		# create NumPy arrays from the boundaries
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")
		#find the values within the colour range
            mask = cv2.inRange(image, lower, upper)
		#create an image that displays the isolated pixels only
            output = cv2.bitwise_and(image, image, mask = mask)
		# show the images
            #cv2.imshow("images", np.hstack([image, output]))
            #cv2.waitKey(3)

		#I'm going to try and use output and scan each pixel to see
		#if it's black or not. If it's not, I'm going to log it coordinates in an array
            colour = (0, 0, 0)
            target = np.where(output != colour)
		#creates an array of coordinate touples
            coordinates = zip(target[0], target[1]) 
		#reduces the pixels to one instance for each coordinate
            filt = list(set(list(coordinates)))

	# creating a new image object with RGB mode and size matching the original
            check = np.zeros((600, 800, 3), dtype=np.uint8)
            check = PIL.Image.new(mode="RGB", size=(width, height))
            pixelMap = check.load()
            setfilt = set(filt)

            cargshecklen = len(filt) -1
            for pixelMap in filt:
                pixelMap[(filt[checklen][1]), (filt[checklen][0])] = (255, 255, 255)
                checklen = checklen -1

            cv2.imshow("final", np.hstack([image, check]))
            cv2.waitKey(3)

	
            #red_image = np.zeros((600, 800, 3), dtype=np.uint8)

            #check[:] = (0, 0, 255)

            #check.show()
            #cv2.imshow("Check image", output)
            #cv2.waitKey(1)
			# Publish processed image
            #message_to_publish = Int32()
            #message_to_publish.data = LR

            #self.result_pub.publish(message_to_publish)
            #sleep(1)
            
		
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
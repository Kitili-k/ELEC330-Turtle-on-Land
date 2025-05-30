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

#12/10/2025 need to sample the camera as a mask every however many seconds, and then publish this to be viewed, not try and use the livstream for processing

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
        cv2.namedWindow("My image", cv2.WINDOW_NORMAL)
        #cv2.namedWindow("Check image", cv2.WINDOW_NORMAL)

    def image_callback(self, msg):
        self.get_logger().info(f'Image data received! it has {msg.encoding}')
        self.get_logger().info(f'OpenCV backend {cv2.getBuildInformation()}')
        try:
            """
                  Callback function for processing incoming camera images
                  Args:
                        msg: ROS Image message from camera
                  """
                              # Convert ROS Image message to OpenCV format
                        

            image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            self.get_logger().info(f' imageshape {image.shape} and dtype {image.dtype}')
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            #This is just to check if it's subscribing right
            #camera_check = PIL.Image.new(mode="RGB", size=(width, height))
            #self.result_pub = camera_check.load()
            #for i in range(10):
            
            #cv2.imwrite("debug.png", image)

            cv2.imshow("My image", image)
            cv2.waitKey(1)
              #    time.sleep(5)
                        
                              # define the list of RBG boundaries that pixels are positivly identified within
                              #Changing this will basically loosed and tighten the colour requirements
                              #to be detected, in theory this needs to be kept tight, but you could
                              #expand it to include more colours if had to loosen the confidence interval
                              #below for some reason

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
            color = (0, 0, 0)
            target = np.where(output != color)
		#creates an array of coordinate touples
            coordinates = zip(target[0], target[1]) 
		#reduces the pixels to one instance for each coordinate
            unique_coordinates = list(set(list(coordinates)))


	#----------------------Part 2-----------------------------
	#This part is basically getting rid of the useless pixels that are just a little bit here and
	#there being detected at random. This includes creating a list of x and y values to find percentile
	#certainties of them relative to a normal distribution curve

		#This is the value of the coordinated identified pixels
		#for the purposes of filtering the x values of the identified pixels
		#For some reason I cannot find a way to trim this, so I have to make a new 
		#lencoord every time I want to do something like this
            lenxcoord1 = len(unique_coordinates) -1
            xvals = [0]
            xtotal = 0
            xmean = 0
            ymean = 0

		#creates a list of the x values called xvals
            for x in unique_coordinates:
                xvals.append(unique_coordinates[lenxcoord1][0])
                xtotal = xtotal + unique_coordinates[lenxcoord1][0]
                lenxcoord1 = lenxcoord1 -1

		#creates a mean of the x values to use in the standard deviation
		#calculation later
                xmean = xtotal / len(xvals)

		#This is the value of the coordinated identified pixels
		#for the purposes of filtering the y values of the identified pixels
            lenxcoord01 = len(unique_coordinates) -1
            yvals = [0]
            ytotal = 0

		#creates a list of the x values called yvals
            for x in unique_coordinates:
                yvals.append(unique_coordinates[lenxcoord01][1])
                ytotal = ytotal + unique_coordinates[lenxcoord01][1]
                lenxcoord01 = lenxcoord01 -1

		#creates a mean of the y values to use in the standard deviation
		#calculation later
            ymean = ytotal / (len(yvals) +1)

		#This is the z score relating to 90% and 10% confidence interval
		#when assuming normal distribution
            zscoreUp = 1.645 
            zscoreDown = -1.282
		#These effectively creates a little bubble around the detected pixels
		#in order to weed out the ones that are mistakes. If you're casting your
		#colour net too wide, this isn't going to do much or you have to make
		#the confidence internval really small. You use this interval to derive
		#the centre value you later pass on to the navigation program, so make
		#sure it's where you want it to be

		#These lines of code are to determine the sandard deviation of the
		#x and y values using the statistics library
            xstd = np.std(xvals)
            ystd = np.std(yvals)

		#determine the upper and lower bounds of the x and y values within
		#the 2nd and 98th percentile by using the equation
		#Percentiule value = mean + (zscore*standarddev)
            perchighx = xmean + zscoreUp*xstd
            perclowx = xmean + zscoreDown*xstd
            perchighy = ymean + zscoreUp*ystd
            perclowy = ymean + zscoreDown*ystd

		#Assigning the values of unique_coordinates which lay within the upper
		#and lower bounds, calculated above, to be within the new array "filt"
            filt = [(0, 0), (0, 0)]
            lenxcoord2 = len(unique_coordinates) -1
            for x in unique_coordinates:
                if (unique_coordinates[lenxcoord2][0] <= perchighx):
                    if (unique_coordinates[lenxcoord2][0] >= perclowx):
                        if (unique_coordinates[lenxcoord2][1] <= perchighy):
                            if (unique_coordinates[lenxcoord2][1] >= perclowy):
                                filt.append(unique_coordinates[lenxcoord2])
                lenxcoord2 = lenxcoord2 -1
            del(filt[0])
            del(filt[0])


	#----------------------Part 3-----------------------------
	#This part is to find the max, min, and average values of the pixels that have been filtered
	#In order to calculate the central point, which will then be passed to the
	#Navigation algorithm. This is basically just scaning the filtered pixel array for the
	#maximum and minimum in x and y, then combining them to be the four corners of the detected area

		#Within the filtered group, find the maximum and minimum to
		#average them later for use in navigation
		#Setting xmax and xmin to use in the next seciton on the code
            Xmax = 0
            Xmin = 0
            Ymax = 0
            Ymin = 0
		#Scan through the colour corrected indices and 
		#find the maximum x value by comparison
		#Need to find a way to scan through only uniqu_cordinate[index][0]
		#to specifically look for x, but i need to do more research

		#Truelen has to be re-initialised for each loop and tells the loop
		#to read through each index in the lis of pixels
            truelenxmax = len(filt) -1
		#print(f"filt: {filt}, truelenxmax: {truelenxmax}, filt[truelenxmax]: {filt[truelenxmax]}")
		#This loop scans each loop index
            for x in filt:
			#print(f"filt: {filt}, truelenxmax: {truelenxmax}, filt[truelenxmax]: {filt[truelenxmax]}")
			#print(filt[truelenxmax][0])
                if filt[truelenxmax][0] >= Xmax:
				#This line compares the current value of Xmax to the 
				#index of the list of identified pixels and combs
				# through to find the highest amount 
                    Xmax = filt[truelenxmax][0]
			#Here the index number is decreased for the next loop
                truelenxmax = truelenxmax -1


            truelenxmin = len(filt) -1
		#This loop scans each loop index
            Xmin = Xmax
            for x in filt:
                if filt[truelenxmin][0] <= Xmin:
			#This line compares the current value of Xmin to the 
				#index of the list of identified pixels and combs
				# through to find the lowest amount
                    Xmin = filt[truelenxmin][0]
			#Here the index number is decreased for the next loop
                    truelenxmin = truelenxmin -1
                if truelenxmin <= 1:
                    break

            truelenymax = len(filt) -1
		#This loop scans each loop index
            for x in filt:
                if filt[truelenymax][1] >= Ymax:
			#This line compares the current value of Xmax to the 
				#index of the list of identified pixels and combs
				# through to find the highest amount
                    Ymax = filt[truelenymax][1]
			#Here the index number is decreased for the next loop
                truelenymax = truelenymax -1

            truelenymin = len(filt) -1
		#This loop scans each loop index
            Ymin = Ymax
            for x in filt:
               if filt[truelenymin][1] <= Ymin:
				#This line compares the current value of Ymin to the 
				#index of the list of identified pixels and combs
				# through to find the lowest amount
                    Ymin = filt[truelenymin][1]
			#Here the index number is decreased for the next loop
                    truelenymin = truelenymin -1
                    if truelenymin <= 1:
                        break


		#Next step is to make the four corners of the box that define the outer
		#boundaries of the pixel locations and find the area within it
            TopLeft = [(Ymax, Xmin)]
            TopRight = [(Ymax, Xmax)]
            BotLeft = [(Ymin, Xmin)]
            BotRight = [(Ymin, Xmin)]

		#This is the central pixel based on the max and min of the filt coordinates
            CentPix = [(((Ymax+Ymin)/2), ((Xmax+Xmin)/2))]

		#This part is to change the relevant values into the correct form to compare
		#The x value of the central pixel with half the width of the image.
            halfwidth = round((width/2), 0)
            halfwidth = int(halfwidth)
            LoR = CentPix[0][0]
            LoR = int(LoR)
		#If the pixel is to the left, the output will be a 0 to the navigaiton program
		#If the pixel is to the right, the output will be a 1
            if LoR < (halfwidth):
                LR = 0
			#Left
                print(LR)
            else:
                LR = 1
			#Right
                print(LR)

	#----------------------Part 4-----------------------------
	#This part is basically just creating an image output of the remaining detected
	#pixels, basically created to verify that what was being detected was correct
	#and also to demonstrate how this program works

	# creating a new image object with RGB mode and size matching the original
            check = PIL.Image.new(mode="RGB", size=(width, height))
            #check = np.zeros((600, 800, 3), dtype=np.uint8)
            pixelMap = check.load()
            setfilt = set(filt)

            cargshecklen = len(filt) -1
            for pixelMap in filt:
                pixelMap[(filt[checklen][1]), (filt[checklen][0])] = (255, 255, 255)
                checklen = checklen -1

            cv2.imshow("final", np.hstack([image, check]))
            cv2.waitKey(3)

	#Need to make the actual centre more visible try using this loop to do it
            for i in range(50): #this should normally be just 5
                pixelMap[((CentPix[0][0])+i), ((CentPix[0][1])+1)] = (255, 0, 0)

            #red_image = np.zeros((600, 800, 3), dtype=np.uint8)

            #check[:] = (0, 0, 255)

            #check.show()
           # cv2.imshow("Check image", check)
            #cv2.waitKey(1)
			# Publish processed image
            message_to_publish = Int32()
            message_to_publish.data = LR

            self.result_pub.publish(message_to_publish)
            sleep(1)
            
		
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
            

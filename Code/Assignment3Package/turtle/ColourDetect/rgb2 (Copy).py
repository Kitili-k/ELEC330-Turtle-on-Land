# import the necessary packages
import numpy as np
import matplotlib.pyplot as plt
import argparse
import cv2
import PIL
import statistics
#----------------------Part 1-----------------------------
#Part one is all about loading the image, identifying the colours you want to isolate
#and then finding the pixel locations of those colours 

# construct the argument parse and parse the arguments to find the image you want
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help = "path to the image")
args = vars(ap.parse_args())
# load the image
image = cv2.imread("/home/turtle_on_land/Downloads/BeachTest1.jpg")
#get the dimensions of the image for replication later in the negative test image
height, width, channel = image.shape
print("The image height is ", height)
print("The image width is ", width)

# define the list of RBG boundaries that pixels are positivly identified within
boundaries = [
	#([0, 0, 141], [255, 115, 255])
	([0, 0, 141], [255, 115, 255])
	#Changing this will basically loosed and tighten the colour requirements
	#to be detected, in theory this needs to be kept tight, but you could
	#expand it to include more colours if had to loosen the confidence interval
	#below for some reason
]

# loop over the boundaries
for (lower, upper) in boundaries:
	# create NumPy arrays from the boundaries
	lower = np.array(lower, dtype = "uint8")
	upper = np.array(upper, dtype = "uint8")
	#find the values within the colour range
	mask = cv2.inRange(image, lower, upper)
	#create an image that displays the isolated pixels only
	output = cv2.bitwise_and(image, image, mask = mask)
	# show the images
	cv2.imshow("images", np.hstack([image, output]))
	cv2.waitKey(0)

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
	print("Xmax is", Xmax)


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
	print("Xmin is", Xmin)

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
	print("Ymax is", Ymax)

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
	print("Ymin is", Ymin)

	#Next step is to make the four corners of the box that define the outer
	#boundaries of the pixel locations and find the area within it
	TopLeft = [(Ymax, Xmin)]
	TopRight = [(Ymax, Xmax)]
	BotLeft = [(Ymin, Xmin)]
	BotRight = [(Ymin, Xmin)]

	#This is the central pixel based on the max and min of the filt coordinates
	CentPix = [(((Xmax+Xmin)/2), ((Ymax+Ymin)/2))]
	print("The central pixel, the final deliverable, is ", CentPix)

	#okay this works, next step is to verify it by creating an image
	#by using the pixel coordinates in filt

#----------------------Part 4-----------------------------
#This part is basically just creating an image output of the remaining detected
#pixels, basically created to verify that what was being detected was correct
#and also to demonstrate how this program works

# creating a new image object with RGB mode and size matching the original
check = PIL.Image.new(mode="RGB", size=(width, height))
pixelMap = check.load()
setfilt = set(filt)

checklen = len(filt) -1
for pizelMap in filt:
		pixelMap[(filt[checklen][1]), (filt[checklen][0])] = (255, 255, 255)
		checklen = checklen -1

check.show()


	#Once I figure out how that works, I need to move on to making it
	#into an angle. Unfortunately, this is kind of the point where my
	#330 Project and my Y3P diverge, but its still helpful 
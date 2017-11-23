#!/usr/bin/env python
## AK
## explorer_node_py.py
##
## BLG456E Assignment 1 skeleton
##
## Instructions: Change the laser_callback function to make the robot explore more
## intelligently, using its sensory data (the laser range array).
##
## Advanced: If you want to make use of the robot's mapping subsystem then you can
## make use of the map in the mapping_callback function.
##
## 

## Common ROS headers.
import rospy
## Required for some printing options
import sys
import numpy as np

import roslib
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

## This is needed for the data structure containing the motor command.
from geometry_msgs.msg import Twist
## This is needed for the data structure containing the laser scan
from sensor_msgs.msg import LaserScan
## This is needed for the data structure containing the map (which you may not use).
from nav_msgs.msg import OccupancyGrid
from math import isnan
import time
righthalf_tmp = 0.0
lefthalf_tmp = 0.0
SLOW = 3

## The following function is a "callback" function that is called back whenever a new laser scan is available.
## That is, this function will be called for every new laser scan.
##
## --------------------------------------------------------------------------------
## ----------CHANGE THIS FUNCTION TO MAKE THE ROBOT EXPLORE INTELLIGENTLY----------
## --------------------------------------------------------------------------------
##
class image_converter:
  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/robot2/camera/rgb/image_raw",Image,self.callback)

  def callback(self,data):
	print("h 	sdfsdfsdfdsfeelo")
	try:
	  cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
	  print(e)



	#hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

	# define range of white color in HSV
	#lower_blue = np.array([185,185,185])
	#upper_blue = np.array([185,185,185])
	#print("heel33333o")

	#mask = cv2.inRange(hsv, lower_green, upper_green)
	output_image = np.array([[[ 0 for k in range(3)]
			 for n in range(int(len(cv_image[0])/5))]
			  for m in range (int(len(cv_image)/5))] ) 

	for x in  range(0,len(cv_image),5):
		for y in  range(0, len(cv_image[0]),5):
			acc1 =0
			acc2 =0
			acc3 =0
			for m in range(x,x+5,1):
				for n in range(y,y+5,1):
					acc1 = cv_image[m][n][0] +acc1
					acc2 = cv_image[m][n][1] +acc2
					acc3 = cv_image[m][n][2] +acc3
			output_image[x/5][y/5][0] = acc1/25;
			output_image[x/5][y/5][1] = acc2/25;
			output_image[x/5][y/5][2] = acc2/25;

	for x in range(len(output_image)):
	  for y in range(len(output_image[0])):
		  if (output_image[x][y][0] <= 255 and output_image[x][y][0] > 155)  and (output_image[x][y][1] <= 255 and output_image[x][y][1] > 155) and (output_image[x][y][2] <= 255 and output_image[x][y][2] > 150):
			  print("HEY, I HAVE SEEN RED\n")
			  
	cv2.imshow("Image window", cv_image)
	cv2.waitKey(3)
	

def laser_callback(data):
	## Lets fill a twist message for motor command
    ##motor_command = Twist()
    ## For now let us just set it up to drive forward ...
    ##motor_command.linear.x = 0.2
    ## And left a little bit
    ##motor_command.angular.z = 0.7
    
    global laser
    laser = data
    motor_command = Twist()
    
    print(data.ranges[len(data.ranges)/2])
    #slightly left
    if (laser.ranges[0] < 0.7 and laser.ranges[40] < 0.7):
        motor_command.linear.x = 0.2
        motor_command.angular.z = 2
        print("slightly left")
    #sharp left
    elif (laser.ranges[200] < 0.7 and laser.ranges[240] < 0.7):
        motor_command.linear.x = 0.1
        motor_command.angular.z = 5
        print("sharp left")
    #sharp right
    elif (laser.ranges[440] < 1 and laser.ranges[480] < 1):
        motor_command.linear.x = 0.1
        motor_command.angular.z = -5
        print("sharp right")
    #slightly right
    elif (laser.ranges[599] < 0.7 and laser.ranges[639] < 0.7):
        motor_command.linear.x = 0.2
        motor_command.angular.z = -2
        print("slightly right")
    #back
    elif (laser.ranges[300] < 1 and laser.ranges[340] < 1 ):
       motor_command.linear.x = -0.2
       motor_command.angular.z = -0.1
       print("back")
    else: #forward
        motor_command.linear.x = 1
        print("forward")
    


    ## Lets publish that command so that the robot follows it
    global motor_command_publisher
    motor_command_publisher.publish(motor_command)
    
def map_callback(data):
    chatty_map = False
    if chatty_map:
        print "-------MAP---------"
        ## Here x and y has been incremented with five to make it fit in the terminal
        ## Note that we have lost some map information by shrinking the data
        for x in range(0,data.info.width-1,5):
            for y in range(0,data.info.height-1,5):
                index = x+y*data.info.width
                if data.data[index] > 50:
                    ## This square is occupied
                    sys.stdout.write('X')
                elif data.data[index] >= 0:
                    ## This square is unoccupied
                    sys.stdout.write(' ')
                else:
                    sys.stdout.write('?')
            sys.stdout.write('\n')
        sys.stdout.flush()
        print "-------------------"
    
## This is the method we initilize everything
def explorer_node():
    ## We must always do this when starting a ROS node - and it should be the first thing to happen
    #rospy.init_node('amble')
    rospy.init_node('robot2')
    ic = image_converter()
    
    ## Here we declare that we are going to publish "Twist" messages to the topic /cmd_vel_mux/navi. It is defined as global because we are going to use this publisher in the laser_callback.
    global motor_command_publisher
    motor_command_publisher = rospy.Publisher('/robot2/cmd_vel', Twist, queue_size = 10)
    
    ## Here we set the function laser_callback to recieve new laser messages when they arrive
    rospy.Subscriber("/robot2/scan", LaserScan, laser_callback, queue_size = 1000)
    
    ## Here we set the function map_callback to recieve new map messages when they arrive from the mapping subsystem
    rospy.Subscriber("/robot2/map", OccupancyGrid, map_callback, queue_size = 1000)
    
    ## spin is an infinite loop but it lets callbacks to be called when a new data available. That means spin keeps this node not terminated and run the callback when nessessary. 
    rospy.spin()
    
if __name__ == '__main__':
    explorer_node()

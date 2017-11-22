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
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)

  def callback(self,data):
	print("heelo")
	try:
	  cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
	  print(e)



	hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

	# define range of white color in HSV
	lower_blue = np.array([185,185,185])
	upper_blue = np.array([185,185,185])
	print("heel33333o")

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
			  print("HEY, I HAVE SEEN WHITE\n")
			  
	cv2.imshow("Image window", cv_image)
	cv2.waitKey(3)
	

def laser_callback(data):
    global laser
    global righthalf_tmp
    global lefthalf_tmp
    laser = data
    ## Lets fill a twist message for motor command
    motor_command = Twist()

    
    hitWall = False #bool for 'has it hit the wall?'

    righthalf=laser.ranges[160] #the point in the middle of laser's right part
    lefthalf=laser.ranges[480] #the point in the middle of laser's left part
    
    deltaRight = 0
    deltaLeft = 0    

    if not isnan(righthalf) and righthalf != righthalf_tmp: #delta calculator for right part of laser
        deltaRight = righthalf_tmp - righthalf
        righthalf_tmp = righthalf

    if not isnan(lefthalf) and lefthalf != lefthalf_tmp: #delta calculator for left part of laser
        deltaLeft = lefthalf_tmp - lefthalf
        lefthalf_tmp = lefthalf        

    #some verbose outputs according to deltas
    if not isnan(righthalf) and deltaRight > 0:
        print 'right getting closer !!! '
    elif not isnan(righthalf) and deltaRight < 0:
        print 'right getting far away!!! '    
    if not isnan(lefthalf) and deltaLeft > 0:
        print 'left getting closer !!! '
    elif not isnan(lefthalf) and deltaLeft < 0:
        print 'left getting far away!!! '
    
    #if it was getting closer and it is nan now, that means it has hit the wall
    #otherwise, it is a big space to go fast; not a wall.
    if isnan(laser.ranges[len(laser.ranges)/2]) and ((isnan(righthalf) or isnan(lefthalf)) or (isnan(laser.ranges[0]) or isnan(laser.ranges[-1]))) and (deltaLeft > 0 or deltaRight > 0):
        hitWall = True
    else:
        hitWall = False
        
    
    if isnan(laser.ranges[len(laser.ranges)/2]): #if laser value is nan, check if it has hit the wall, otherwise go fast
        if hitWall is True:
            motor_command.linear.x = -0.3/SLOW
            motor_command.angular.z = 1/SLOW
        else:
            motor_command.linear.x = 0.7/SLOW
     
    elif righthalf < 0.6 and lefthalf < 0.6: #if turtlebot is close to wall, make a decision
        if righthalf > lefthalf: #left is nearer, turn right
            motor_command.angular.z = 1.2/SLOW  #rotate left
        else: #right is nearer, turn left
            motor_command.angular.z = -1.2/SLOW  #rotate right
            
    elif righthalf < 0.6 : #if just right is close, turn left a bit
        motor_command.angular.z = 0.3/SLOW  #rotate left

    elif lefthalf < 0.6: #if just left is close, turn right a bit
        motor_command.angular.z = -0.3/SLOW  #rotate right

    elif (laser.ranges[len(laser.ranges)/2]) < 1.0: #if close to a wall
        if laser.ranges[0] < 0.6 and laser.ranges[-1] < 0.5: #if rightmost an leftmost points are close, just do a left turn
            motor_command.angular.z = 1/SLOW
        else: #else make a reverse attempt
            motor_command.angular.z = -0.3/SLOW    
    
    else: #else, just drive forward!
        motor_command.linear.x = 0.3/SLOW         
        
            
    global motor_command_publisher
    motor_command_publisher.publish(motor_command)
    
    ## Alternatively we could have looked at the laser scan BEFORE we made this decision
    ## Well Lets see how we might use a laser scan
    ## Laser scan is an array of distances
    print 'Number of points in laser scan is: ', len(data.ranges)
    print 'The distance to the rightmost scanned point is: ', data.ranges[0]
    print 'The distance to the leftmost scanned point is: ', data.ranges[-1]
    print 'The distance to the middle scanned point is: ', data.ranges[len(data.ranges)/2]
    ## You can use basic trigonometry with the above scan array and the following information to find out exactly where the laser scan found something
    print 'The minimum angle scanned by the laser is: ', data.angle_min
    print 'The maximum angle scanned by the laser is: ', data.angle_max
    print 'The increment in the angles scanned by the laser is: ', data.angle_increment
    ## angle_max = angle_min+angle_increment*len(data.ranges)
    print 'The minimum range (distance) the laser can perceive is: ', data.range_min
    print 'The maximum range (distance) the laser can perceive is: ', data.range_max
    
## You can also make use of the map which is being built by the "gslam_mapping" subsystem
## There is some code here to help but you can understand the API also by looking up the OccupancyGrid message and its members (this is the API for the message)
## If you want me to explain the data structure, I will - just ask me in advance of class
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

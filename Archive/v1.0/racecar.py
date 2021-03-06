#!/usr/bin/env python
"""
racecar(GIT).py

MIT RACECAR 2016

This class will import all algorithm modules and
should be imported by the main python script that
activates the node.

"""

# IMPORTS

import rospy
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped



# CLASS DECLARATION

class racecar:

    def __init__(self):
        self.DrivePub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped,queue_size=10)
        self.SafetyPub = rospy.Publisher('vesc/ackermann_cmd_mux/input/safety', AckermannDriveStamped,queue_size=10)        

        # Add any other topic variables here


 
    # Function: drive
    # Parameters: speed (float), angle (float)
    #
    # This function will publish a drive command to
    # the /navigation topic in ROS

    def drive(self, speed, angle):
	msg = AckermannDriveStamped()           # Initializes msg variable
	msg.drive.speed = speed                 # Sets msg speed to entered speed
    	msg.drive.acceleration = 0              # Sets msg acceleration to 0
	msg.drive.jerk = 1                      # Sets msg jerk to 1
	msg.drive.steering_angle = angle        # Sets msg steering angle to entered angle
	msg.drive.steering_angle_velocity = 1   # Sets msg angle velocity to 1
	self.DrivePub.publish(msg)              # Publishes the message


 
    # Function: bbWallFollow
    # Parameters: ranges (list), d_desired (float), speed (float)
    #
    # This function uses a bang-bang control system
    # to let the car follow a line

    def bbWallFollow(self, ranges, d_desired, speed):

        distance = min(ranges[:360])          # Finds the minimum range
	error = d_desired - distance    # Calculates error
        steering_angle = 0              # Initializes steering_angle

        THRESHOLD = 0.1                 # Sets threshold to 0.1m
        
	if abs(error) < THRESHOLD:      # If error within threshold:
	    steering_angle = 0          #       Kill steering

        elif error > 0:                 # If too far to the right:
            steering_angle = 1        #       Turn left

	elif error < 0:                 # If too far to the left:
	    steering_angle = -1       #       Turn right

	self.drive(speed, steering_angle)    # Execute drive function



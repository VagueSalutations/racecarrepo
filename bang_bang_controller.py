#!/usr/bin/env python


"""
bang_bang_controller.py

MIT RACECAR 2016

This program implements an upgraded bang bang
control system for wall following.

"""


# IMPORTS
#–––––––––––––––––––––––––––––––––––––––––––––––––––––––

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


# VARIABLES
#–––––––––––––––––––––––––––––––––––––––––––––––––––––––

d_desired = .5                          # Desired distance 0.5 m from wall
steering_angle = 0                      # Initialize steering angle to 0°
error = 0                               # Initialize error variable to 0
distance = 0.0                          # Initialize distance variable to 0

THRESHOLD = 0.1


# SUPPORTING FUNCTION
#–––––––––––––––––––––––––––––––––––––––––––––––––––––––

def callBack(msg):

        ranges = msg.ranges             # Imports ranges (array of floats)
	distance = min(ranges)          # Finds the minimum range

	error = d_desired - distance    # Calculates error

	if abs(error) > THRESHOLD:      # If error within threshold:
		steering_angle = 0      #       Kill steering

        elif error > 0:                 # If too far to the right:
                steering_angle = 0.5    #       Turn left

	elif error < 0:                 # If too far to the left:
		steering_angle = -0.5   #       Turn right

	drive()                         # Execute drive function


# SUPPORTING FUNCTION
#–––––––––––––––––––––––––––––––––––––––––––––––––––––––

def drive():
        msg = AckermannDriveStamped();
        #msg.header.stamp = rospy.Time.now();
        #msg.header.frame_id = "base_link";
        msg.drive.speed = 3
        msg.drive.acceleration = 0
        msg.drive.jerk = 1
        msg.drive.steering_angle = steering_angle
        msg.drive.steering_angle_velocity = 1
        pub.publish(msg)


# MAIN()      
#–––––––––––––––––––––––––––––––––––––––––––––––––––––––

scanResult = rospy.Subscriber("racecar/laser/scan",LaserScan,callBack)
pub = rospy.Publisher('/racecar/ackermann_cmd_mux/input/safety', AckermannDriveStamped,queue_size=10)
rospy.init_node('one_point')
rospy.spin()

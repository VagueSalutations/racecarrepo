#!/usr/bin/env python


"""
bbWallFollowNode.py

MIT RACECAR 2016

This program implements the bbWallFollow (bang bang
wall follow) method defined in the racecar.py class.

"""


# IMPORTS

import rospy
import math
import racecar.py
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped



# VARIABLES
SUBSCRIBE_TO_THREAD = "racecar/laser/scan"
NODE_NAME = 'bbWallFollow'

D_DESIRED = 0.5
SPEED = 1.0

racecar = racecar()


# CALLBACK

def callBack(msg):

    racecar.bbWallFollow(msg.ranges, D_DESIRED, SPEED)



# MAIN()      

scanResult = rospy.Subscriber(SUBSCRIBE_TO_THREAD,LaserScan,callBack)
rospy.init_node(NODE_NAME)
rospy.spin()

#!/usr/bin/env python


"""
bbWallFollowNode(local).py

MIT RACECAR 2016

This program implements the bbWallFollow (bang bang
wall follow) method defined in the racecar.py class.

"""

# IMPORTS

import rospy
import math
from racecar import racecar
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped



# VARIABLES

SUBSCRIBE_TO_THREAD = "/scan"
NODE_NAME = 'bbWallFollow'

D_DESIRED = 0.2
SPEED = 1.0

side = "L"
kevin = racecar()


# CALLBACK

def callBack(msg):

    # Query for side
    # <implement here>
    
    # Query for safety
    # <implement here>

    # Query for Bang Bang
    kevin.bbWallFollow(msg.ranges, D_DESIRED, SPEED, side)



# MAIN()      

rospy.init_node(NODE_NAME)
scanResult = rospy.Subscriber(SUBSCRIBE_TO_THREAD,LaserScan,callBack)

rospy.spin()

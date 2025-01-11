#! /usr/bin/env python

## @package assignment2_exprob
#
# \file laser_point_pub.py
# \brief laser_point_pub
# \author Alberto Bono 3962994
# \date 04/01/2025
#
# Subscribes to:
#    /scan
#
# Subscribes to:
#    /laser_direction
#
# Description:
# This node allows the robot to find the closest point to the robot in the laser scan data.
# It subscribes to the /scan topic to get the laser scan data.
# When the robot finds the closest point, it publishes a message with the direction and the distance of the closest point.


# Import the necessary libraries
import rospy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import *
from assignment2_exprob.msg import laser_direction

# Global variables for managing the regions of the laser scan data
pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}

# Callback for the /scan topic: find the closest point to the robot in the laser scan data
def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
    }
    
   
    # Store the key of the minimum value of the regions in the variable min_key
    min_key = min(regions_, key=regions_.get)
    # Store the minimum value of the regions in the variable min_value
    min_value = regions_[min_key]
    # Publish the direction and the distance of the closest point to the robot
    msg = laser_direction()
    msg.direction = min_key
    msg.distance = min_value
    pub_.publish(msg)
    
# Main function
def main():
    global pub_
    rospy.init_node('laser_point_pub')
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    pub_ = rospy.Publisher('/laser_direction', laser_direction, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    main()

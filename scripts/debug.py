#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
from assignment2_exprob.msg import laser_direction
import math

active_ = False

pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}




def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
    }
    min_key = min(regions_, key=regions_.get)
    print("il punto più vicino sta : ", min_key)
    min_value = regions_[min_key]
    print("la distanza è: ", min_value)

    # Crea e pubblica il messaggio personalizzato
    msg = laser_direction()
    msg.direction = min_key
    msg.distance = min_value
    pub_.publish(msg)
    

   


def main():
    global pub_, active_

    rospy.init_node('reading_laser')

    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    pub_ = rospy.Publisher('/laser_direction', laser_direction, queue_size=10)
    rospy.spin()



if __name__ == '__main__':
    main()

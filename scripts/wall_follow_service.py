#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

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
state_ = 0
state_dict_ = {
    0: 'go forward',
    1: 'retro',
    2: 'turn right',
    3: 'turn left',
}


def wall_follower_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res


def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
    }

    take_action()


def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print ('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state


def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    state_description = ''
    min_key=min(regions, key=regions.get)
    min_value=regions[min_key]
    d0 = 0.4
    d = 3
    if min_key == 'front':
        if min_value < d0:
            state_description = 'case 1 - retro'
            change_state(1)
        elif min_value < d:
            state_description = 'case 2 - go_forward'
            change_state(0)
    elif min_key == 'fright' or min_key == 'right':
        state_description = 'case 4 - turn_right'
        change_state(2)
    
    elif min_key == 'fleft' or min_key == 'left':
        state_description = 'case 3 - turn_left'
        change_state(3)
    
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)
def go_forward():
    global regions_

    msg = Twist()
    msg.linear.x = 0.5
    return msg
def retro():
    msg = Twist()
    msg.linear.x = -0.5
    return msg
def turn_right():
    msg = Twist()
    msg.angular.z = -0.3
    return msg

def turn_left():
    msg = Twist()
    msg.angular.z = 0.3
    return msg


def main():
    global pub_, active_

    rospy.init_node('reading_laser')

    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)

    srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            rate.sleep()
            continue
        else:
            msg = Twist()
            if state_ == 0:
                msg = go_forward()
            elif state_ == 1:
                msg = retro()
            elif state_ == 2:
                msg = turn_right()
            elif state_ == 3:
                msg = turn_left()
            else:
                rospy.logerr('Unknown state!')

            pub_.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()

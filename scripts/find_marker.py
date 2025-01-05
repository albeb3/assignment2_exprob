#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Pose, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import actionlib
import actionlib.msg
import assignment2_exprob.msg
from tf import transformations
from std_srvs.srv import *
import time

srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180)  # 5 degrees
position_ = Point()
pose_ = Pose()
desired_position_ = Point()
desired_position_.z = 0
regions_ = None
state_desc_ = ['Go to point', 'wall following', 'done']
state_ = 0
# 0 - stop service
# 1 - find marker
# 2 - done
# 3 - canceled
# callbacks
def clbk_marker_detected(msg):

def clbk_odom(msg):
    global position_, yaw_, pose_

    # position
    position_ = msg.pose.pose.position
    pose_ = msg.pose.pose

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }


def change_state(state):
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_go_to_point_
    state_ = state
    log = "state changed: %s" % state_desc_[state]
    rospy.loginfo(log)
    if state_ == 0:
        resp = srv_client_wall_follower_(False)
    if state_ == 1:
        resp = srv_client_wall_follower_(True)



def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)
    
    
def planning(goal):
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_, act_s, pose_
    change_state(0)
    rate = rospy.Rate(20)
    success = True
    
    desired_position_.x = goal.target_pose.pose.position.x
    desired_position_.y = goal.target_pose.pose.position.y
    rospy.set_param('des_pos_x', desired_position_.x)
    rospy.set_param('des_pos_y', desired_position_.y)
    
    
    feedback = assignment2_exprob.msg.FindMarkerFeedback()
    result =  assignment2_exprob.msg.FindMarkerResult()
    
    while not rospy.is_shutdown():
        err_pos = math.sqrt(pow(desired_position_.y - position_.y, 2) +
                        pow(desired_position_.x - position_.x, 2))
      
        if state_ == 0:
            feedback.stat = "State 0: end look for marker"
            act_s.publish_feedback(feedback)
        elif state_ == 1:
            feedback.stat = "State 1: search for marker"
            act_s.publish_feedback(feedback)
            if min(regions_,key=regions_.get)=='front': # and marker detected
                change_state(0)
        
        elif state_== 2:
            break
            
            
        else:
            rospy.logerr('Unknown state!')

        rate.sleep()
    
    if success:
        rospy.loginfo('Goal: Succeeded!')
        act_s.set_succeeded(result)
    
    

def main():
    time.sleep(2)
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_, act_s, pub

    rospy.init_node('bug0')
    
    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    sub_marker_detected = rospy.Subscriber(
        '/robot4_xacro/marker_id_detected', assignment2_exprob::Marker_id_pos, clbk_marker_detected)

    
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    
    srv_client_wall_follower_ = rospy.ServiceProxy(
        '/wall_follower_switch', SetBool)
    act_s = actionlib.SimpleActionServer('/reaching_goal', assignment2_exprob.msg.FindMarkerAction, findmarker, auto_start=False)
    act_s.start()
   
    # initialize going to the point
    

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()
    
if __name__ == "__main__":
    main()

#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Pose, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import actionlib
import actionlib.msg
import assignment_2_2023.msg
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



def clbk_odom(msg):
    global position_, yaw_, pose_

    # position
    position_ = msg.pose.pose.position
    pose_ = msg.pose.pose

    
    
def planning(goal):
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_, act_s, pose_
    rate = rospy.Rate(20)
    success = True
    
    desired_position_.x = goal.target_pose.pose.position.x
    desired_position_.y = goal.target_pose.pose.position.y
    rospy.set_param('des_pos_x', desired_position_.x)
    rospy.set_param('des_pos_y', desired_position_.y)
    
    feedback = assignment_2_2023.msg.PlanningFeedback()
    result = assignment_2_2023.msg.PlanningResult()
    
    while not rospy.is_shutdown():
        err_pos = math.sqrt(pow(desired_position_.y - position_.y, 2) +
                        pow(desired_position_.x - position_.x, 2))
        if err_pos < 0.5:
            feedback.stat = "Target reached!"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            break       
     
        rate.sleep()
    
    if success:
        rospy.loginfo('Goal: Succeeded!')
        act_s.set_succeeded(result)
    
def main():
    time.sleep(2)
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_, act_s, pub

    rospy.init_node('bug0')
    desired_position_.x = 0.0
    desired_position_.y = 1.0
    rospy.set_param('des_pos_x', desired_position_.x)
    rospy.set_param('des_pos_y', desired_position_.y)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    act_s = actionlib.SimpleActionServer('/reaching_goal', assignment_2_2023.msg.PlanningAction, planning, auto_start=False)
    act_s.start()
   
    # initialize going to the point
    

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()
    
if __name__ == "__main__":
    main()

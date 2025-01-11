#! /usr/bin/env python

## @package assignment2_exprob
#
# \file check_arrive.py
# \brief check_arrive_node
# \author Alberto Bono 3962994
# \date 04/01/2025
#
# Subscribes to:
#    /odom
#
# SimpleActionClient:
#	/reaching_goal
#
# Description:
# This node allows the robot to check if it has reached the target position. 
# I have to made this node because the /move_base dosn't provide a feedback message when the robot reaches the target position.
# It subscribes to the /odom topic to get the current position of the robot. 
# When the robot reaches the target position, it sends a feedback message to the action client /reaching_goal and sets the result as succeeded.


import rospy
from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import Odometry
import math
import actionlib
import actionlib.msg
import assignment_2_2023.msg
import time

# globals variables for managing the position of the robot
position_ = Point()
pose_ = Pose()
desired_position_ = Point()
desired_position_.z = 0

# callback for the odom topic: fill the position_ variable with the current position of the robot
def clbk_odom(msg):
    global position_,  pose_
    # position
    position_ = msg.pose.pose.position
    pose_ = msg.pose.pose

    
 # callback for the action client /reaching_goal: check if the robot has reached the target position
 # if the robot has reached the target position, it sends a feedback message to the action client /reaching_goal 
 # and sets the result as succeeded
def check_goal_cllback(goal):
    global  position_, desired_position_, act_s, pose_
 
    rate = rospy.Rate(20)
    success = True
    
    desired_position_.x = goal.target_pose.pose.position.x
    desired_position_.y = goal.target_pose.pose.position.y
    
    
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
    global  act_s
    rospy.init_node('check_arrive_node')
    
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    act_s = actionlib.SimpleActionServer('/reaching_goal', assignment_2_2023.msg.PlanningAction, check_goal_cllback, auto_start=False)
    act_s.start()

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()
    
if __name__ == "__main__":
    main()

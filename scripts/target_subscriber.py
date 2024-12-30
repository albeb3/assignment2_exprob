#!/usr/bin/env python

'''
File: last_target.py
Node: last_target_service
Author: Alberto Bono 3962994
Date: 04/03/2024

Subscribes to:
    /reaching_goal/goal

Server:
    /last_target_service

Description:
When called, returns the coordinates of the last target set by the user.
'''

import rospy
from assignment_2_2023.srv import Last_target, Last_targetResponse
from assignment_2_2023.msg import PlanningActionGoal

x = 0.0
y = 0.0  # Variables for the target position

def target_position(msg):
    global x, y
    x = msg.goal.target_pose.pose.position.x
    y = msg.goal.target_pose.pose.position.y
    rospy.loginfo("pos subscriber@[%f %f ]",x, y)



if __name__ == '__main__':
    rospy.init_node('last_target_service')
    rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, target_position)  # Subscriber for the target position from /reaching_goal/goal
   
    rospy.spin()


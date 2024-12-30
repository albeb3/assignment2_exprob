#!/usr/bin/env python3

import rospy
import actionlib
from assignment_2_2023.msg import PlanningAction, PlanningGoal, Posvel
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Twist
import threading  # Assicurati di importare il modulo threading

# Variabili per la posizione e la velocità del robot
robot_x = 0.0
robot_y = 0.0
robot_vel_x = 0.0
robot_vel_y = 0.0
busy = False  # Variabile per gestire il blocco della condizione

# Callback per il subscriber di odom
def odom_callback(msg):
    global robot_x, robot_y, robot_vel_x, robot_vel_y
    robot_x = msg.pose.pose.position.x
    robot_y = msg.pose.pose.position.y
    robot_vel_x = msg.twist.twist.linear.x
    robot_vel_y = msg.twist.twist.linear.y

# Funzione per gestire l'input dell'utente per inviare obiettivi a un server di azioni ROS
def user_input(client):
    global busy
    while not rospy.is_shutdown():
        if client.wait_for_result(rospy.Duration(0.1)) and busy:
            state = client.get_state()
            rospy.loginfo("Action finished: %s", state)
            busy = False
        else:
            if not busy:
                goal_x = float(input("INSERT THE GOAL COORDINATE (x): "))
                goal_y = float(input("INSERT THE GOAL COORDINATE (y): "))
                rospy.loginfo("Sending the goal")
                goal = PlanningGoal()
                goal.target_pose.pose.position.x = goal_x
                goal.target_pose.pose.position.y = goal_y
                client.send_goal(goal)
                busy = True
            else:
                print("-> Press 'x' to cancel the goal, then press Enter:")
                print("-> Press 'c' to check if the goal has been reached, then press Enter:")
                input_val = input()
                if input_val == 'x':
                    client.cancel_goal()
                    rospy.loginfo("Goal cancelled!")
                    busy = False
                elif input_val == 'c' and busy:
                    rospy.loginfo("Wait for the result, try again")
                else:
                    rospy.loginfo("Invalid input!")

# Funzione per pubblicare la posizione e la velocità del robot
def publish_posvel():
    rospy.init_node('goal_sender_node', anonymous=True)
    pos_publisher = rospy.Publisher('/posvel', Posvel, queue_size=1000)
    sub = rospy.Subscriber('/odom', Odometry, odom_callback)
    client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    rospy.loginfo("Waiting for action server to start...")
    client.wait_for_server()
    rospy.loginfo("Action server started!")

    user_input_thread = threading.Thread(target=user_input, args=(client,))
    user_input_thread.start()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pos_msg = Posvel()
        pos_msg.x = robot_x
        pos_msg.y = robot_y
        pos_msg.vel_x = robot_vel_x
        pos_msg.vel_y = robot_vel_y
        pos_publisher.publish(pos_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_posvel()
    except rospy.ROSInterruptException:
        pass


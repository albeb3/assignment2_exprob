#!/usr/bin/env python

## @package assignment2_exprob
#
# \file plan_services.py
# \brief plan_services
# \author Alberto Bono 3962994
# \date 04/01/2025
#
# Server:
#    /rosplan_problem_interface/problem_generation_server
#    /rosplan_planner_interface/planning_server
#    /rosplan_parsing_interface/parse_plan
#    /rosplan_plan_dispatcher/dispatch_plan
#
# Description:
# This node allows the robot to call the services for generating the problem, planning, parsing the plan, and dispatching the plan.


# Import the necessary libraries
import rospy
from std_srvs.srv import Empty
from rosplan_dispatch_msgs.srv import DispatchService
# Main function
def main():
    # Initialize the node
    rospy.init_node('plan_services', anonymous=True)

    # Wait for the services to be available
    rospy.sleep(5)

    # Initialize the services
    problem_generation = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server', Empty)
    planner = rospy.ServiceProxy('/rosplan_planner_interface/planning_server', Empty)
    parser = rospy.ServiceProxy('/rosplan_parsing_interface/parse_plan', Empty)
    dispatcher = rospy.ServiceProxy('/rosplan_plan_dispatcher/dispatch_plan', DispatchService)
    # Call the services
    try:
        # Call the problem generation service
        rospy.loginfo("Calling problem generation service...")
        problem_generation()

        # Call the planner service
        rospy.loginfo("Calling planner service...")
        planner()

        # Call the parser service
        rospy.loginfo("Calling parser service...")
        parser()

        # Call the dispatcher service
        rospy.loginfo("Calling dispatcher service...")
        disp_srv = DispatchService._request_class()  
        dispatcher(disp_srv)
        rospy.loginfo("Dispatcher service called successfully.")
    # Handle the exceptions
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return 1

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
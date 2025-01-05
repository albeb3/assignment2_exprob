#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty
from rosplan_dispatch_msgs.srv import DispatchService

def main():
    # Inizializza il nodo ROS
    rospy.init_node('logic_node', anonymous=True)

    # Pausa per consentire ai nodi di avviarsi
    rospy.sleep(5)

    # Definizione dei client dei servizi
    problem_generation = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server', Empty)
    planner = rospy.ServiceProxy('/rosplan_planner_interface/planning_server', Empty)
    parser = rospy.ServiceProxy('/rosplan_parsing_interface/parse_plan', Empty)
    dispatcher = rospy.ServiceProxy('/rosplan_plan_dispatcher/dispatch_plan', DispatchService)

    try:
        # Chiamata al servizio di generazione del problema
        rospy.loginfo("Calling problem generation service...")
        problem_generation()

        # Chiamata al servizio del planner
        rospy.loginfo("Calling planner service...")
        planner()

        # Chiamata al servizio del parser
        rospy.loginfo("Calling parser service...")
        parser()

        # Chiamata al servizio del dispatcher
        rospy.loginfo("Calling dispatcher service...")
        disp_srv = DispatchService._request_class()  # Inizializza una richiesta vuota
        dispatcher(disp_srv)
        rospy.loginfo("Dispatcher service called successfully.")

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return 1

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
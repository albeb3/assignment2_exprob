/**
* \file marker_detection_client.cpp
* \brief my_rosplan_find_marker_client_action Node
* \author Alberto Bono 3962994
* \date 04/01/2025 
* 
* \details
* SimpleActionClient:<BR>
*		/find_marker<BR>
*
* Description:<BR>
* This node receives the goal from the planner via the marker message
* sends it to the find_marker action server.
* It then waits for the result.
**/
// Include some necessary libraries
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
// Include the message header files
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>
// Include the action header files
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include "assignment2_exprob/planning_action.h"
#include <assignment2_exprob/MarkerDetectionAction.h>
// Define the MarkerDetectionClient
typedef actionlib::SimpleActionClient<assignment2_exprob::MarkerDetectionAction> MarkerDetectionClient;
// Implement the MyActionInterface class
namespace KCL_rosplan {
    MyActionInterface::MyActionInterface(ros::NodeHandle &nh) {
    // here the initialization
    }
    /**
    * \brief Callback function for the /rosplan_plan_dispatcher/action_dispatch topic
    * \param msg(const rosplan_dispatch_msgs::ActionDispatch)
    * \result sends the robot to the selected marker's position
    */
    bool MyActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) 
    {
        // here the implementation of the action
        std::cout << "Trying to find marker " << msg->parameters[2].value << std::endl;
        // Creating the action client
        MarkerDetectionClient fm("find_marker", true);
        // Creating the goal for the find_marker action
	    assignment2_exprob::MarkerDetectionGoal id_marker;
        // Waiting for the action server to come up
        while(!fm.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the find_marker action server to come up");
        }
        // Setting the goal for the marker_detection action
        if(msg->parameters[2].value == "m1"){
            id_marker.goal = 11;
        }
        else if(msg->parameters[2].value == "m2"){
            id_marker.goal = 12;
        }
        else if(msg->parameters[2].value == "m3"){
            id_marker.goal = 13;
        }
        else if(msg->parameters[2].value == "m4"){
            id_marker.goal = 15;
        }
        // Sending the goal to the action server
        fm.sendGoal(id_marker);
        fm.waitForResult();
        // Checking the result
        if(fm.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Marker found");
            return true;
        }
        else
        {
            ROS_INFO("Marker not found");
            return false;
        }
    }
}
/**
* \brief Main function
*/
int main(int argc, char **argv) {
    ros::init(argc, argv, "my_rosplan_find_marker_client_action", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    KCL_rosplan::MyActionInterface my_fm(nh);
    my_fm.runActionInterface();
    return 0;
}
    

    
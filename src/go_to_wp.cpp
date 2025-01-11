/** 
* \file go_to_wp.cpp
* \brief my_rosplan_go_to_wp_action Node
* \author Alberto Bono 3962994
* \date 04/01/2025
*
* \details
* Subscribes to:<BR>
*		/rosplan_plan_dispatcher/action_dispatch<BR>
*
* SimpleActionClient:<BR>
*		/reaching_goal<BR>
*		/move_base<BR>
*
* Description:<BR>
* This node receives the goal from the planner via the waypoint message
* and sends it to both the move_base action server and the reaching_goal action server.
**/
// Include some necessary libraries
#include <unistd.h>
#include <cstring>
#include <string>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
// Include the message header files
#include <geometry_msgs/PoseStamped.h>
// Include the action header files
#include "assignment2_exprob/planning_action.h"
#include <assignment2_exprob/PlanningAction.h>
#include <assignment2_exprob/PlanningGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseGoal.h>
/**
* \brief Implementation of the MyActionInterface class
*/
namespace KCL_rosplan {
	MyActionInterface::MyActionInterface(ros::NodeHandle &nh) {
			// here the initialization
	}
	/**
	* \brief Callback function for the /rosplan_plan_dispatcher/action_dispatch topic
	* \param msg(const rosplan_dispatch_msgs::ActionDispatch)
	* \result sends the robot to the selected waypoint's position
	 */
	bool MyActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {//legge dal topic /rosplan_plan_dispatcher/action_dispatch
		// here the implementation of the action 
		//std::cout << "Going from " << msg->parameters[1].value << " to " << msg->parameters[2].value << std::endl;
		// Creating the action clients
		actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true); // Client for the move_base action
		actionlib::SimpleActionClient<assignment2_exprob::PlanningAction> client("/reaching_goal", true); // Client for the reaching_goal action
		actionlib::SimpleClientGoalState state = client.getState(); // receive the status
		move_base_msgs::MoveBaseGoal goal; // Goal for the move_base action
		assignment2_exprob::PlanningGoal goal1; // Goal for the reaching_goal action
		// Waiting for the action servers to come up
		while(!ac.waitForServer(ros::Duration(5.0)) && !client.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }
		// Setting the goal for the move_base action
		goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
		// Routine to set the goal based on the selected waypoint
		// the planner should sends the waypoint ID 
		// and the goal is set based on the selected waypoint
		if(msg->parameters[2].value == "wp1"){
		goal.target_pose.pose.position.x = 6.0;
		goal.target_pose.pose.position.y = 2.0;
		goal.target_pose.pose.orientation.w = 1.0;
		goal1.target_pose.pose.position.x = 6.0;
        goal1.target_pose.pose.position.y = 2.0;
		}
		else if (msg->parameters[2].value == "wp2"){
		goal.target_pose.pose.position.x = 7.0;
		goal.target_pose.pose.position.y = -5.0;
		goal.target_pose.pose.orientation.w = 1.0;
		goal1.target_pose.pose.position.x = 7.0;
		goal1.target_pose.pose.position.y = -5.0;
		}
		else if (msg->parameters[2].value == "wp3"){
		goal.target_pose.pose.position.x = -3.0;
		goal.target_pose.pose.position.y = -8.0;
		goal.target_pose.pose.orientation.w = 1.0;
		goal1.target_pose.pose.position.x = -3.0;
		goal1.target_pose.pose.position.y = -8.0;
		}
		else if (msg->parameters[2].value == "wp4"){
		goal.target_pose.pose.position.x = -7.0;
		goal.target_pose.pose.position.y = 1.0;
		goal.target_pose.pose.orientation.w = 1.0;
		goal1.target_pose.pose.position.x = -7.0;
		goal1.target_pose.pose.position.y = 1.0;
		}
		else if (msg->parameters[2].value == "wp0"){
		goal.target_pose.pose.position.x = 1.0;
		goal.target_pose.pose.position.y = 2.0;
		goal.target_pose.pose.orientation.w = 1.0;
		goal1.target_pose.pose.position.x = 1.0;
		goal1.target_pose.pose.position.y = 2.0;
		}
		// Sending the goals to the action servers
		client.sendGoal(goal1);
		ac.sendGoal(goal);
		// Waiting for the action servers to complete the goals
		client.waitForResult();
		ROS_INFO("Action state: %s \n",state.toString().c_str());
		if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
			ac.cancelGoal(); 
            ROS_INFO("Goal reached");
            return true;
        }
        else
        {
            ROS_INFO("The base failed to reach the goal");
            return false;
        }
	}
}
/**
* \brief Main function
*/
int main(int argc, char **argv) {
	ros::init(argc, argv, "my_rosplan_go_to_wp_action", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	KCL_rosplan::MyActionInterface my_aci(nh);
	my_aci.runActionInterface();
	return 0;
}

/**
* \file go_to_marker.cpp
* \brief my_rosplan_go_to_marker_action Node
* \author Alberto Bono 3962994
* \date 04/01/2025
*
* \details
* Subscribes to:<BR>
*		/detected_markers<BR>
*
* SimpleActionClient:<BR>
*		 /reaching_goal<BR>
*		 /move_base<BR>	
*
* Description:<BR>
* This node receives the goal from the planner via the marker message
* then associates the goal with the relative detected marker's position 
* and sends it to both the move_base action server and the reaching_goal action server.
**/
// Include some necessary libraries
#include <unistd.h>
#include <cstring>
#include <string>
#include <vector>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
// Include the message header file
#include <geometry_msgs/PoseStamped.h>
#include <assignment2_exprob/Marker_id_pos.h>
#include <assignment2_exprob/MarkerArray.h>
// Include the action header file
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include "assignment2_exprob/planning_action.h" 
#include <assignment2_exprob/PlanningAction.h>
#include <assignment2_exprob/PlanningGoal.h>
// Global variables
const int MAX_MARKERS = 4;///< Maximum number of markers that can be detected
int marker_ids[MAX_MARKERS];///< Array of marker IDs detected
float marker_positions_x[MAX_MARKERS];///< Array of marker  x positions 
float marker_positions_y[MAX_MARKERS];///< Array of marker y positions
int marker_count = 0; ///< Counter for the number of markers detected
int selected_marker_id = -1; ///< Initialising the selected marker ID to -1 in case no marker is detected
/**
* \brief Callback function for the /detected_markers topic
* \param msg(const assignment2_exprob::MarkerArray) 
* \result fills the marker_ids, marker_positions_x, marker_positions_y arrays with the detected markers' IDs and positions
*/
void markerCallback(const assignment2_exprob::MarkerArray::ConstPtr& msg) {
    for (const auto& marker : msg->markers) {
        if (marker_count < MAX_MARKERS+1) {
        marker_ids[marker_count] = marker.marker_id;
        marker_positions_x[marker_count] = marker.pose.position.x;
        marker_positions_y[marker_count] = marker.pose.position.y;
        marker_count++;
    } else {
        ROS_WARN("Maximum number of markers reached. Ignoring further markers.");
        break;
    }
    }
}
/**
* \brief Implementation of the action interface
*/
namespace KCL_rosplan {
	MyActionInterface::MyActionInterface(ros::NodeHandle &nh) {
			// here the initialization
	}
	/**
	* \brief Callback function for the /rosplan_plan_dispatcher/action_dispatch topic
	* \param msg(const rosplan_dispatch_msgs::ActionDispatch)
	* \result sends the robot to the selected marker's position
	 */
	bool MyActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {//legge dal topic /rosplan_plan_dispatcher/action_dispatch
		// here the implementation of the action
		//std::cout << "Going from " << msg->parameters[2].value << " to " << msg->parameters[3].value << std::endl;
		// Creating the action clients
		actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true); // Client for the move_base action
		actionlib::SimpleActionClient<assignment2_exprob::PlanningAction> client("/reaching_goal", true); // Client for the reaching_goal action
		actionlib::SimpleClientGoalState state = client.getState(); // Receive the status
		// Creating the goals
		move_base_msgs::MoveBaseGoal goal; // Goal for the move_base action
		assignment2_exprob::PlanningGoal goal1; // Goal for the reaching_goal action
		// Waiting for the action servers to come up
		 while(!ac.waitForServer(ros::Duration(5.0)) && !client.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }
		// Setting the goal for the move_base action
		goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
		// Routine to set the goal based on the selected marker
		// the planner should sends the marker ID from the lowest to the highest
		if(msg->parameters[1].value == "m1"){
			selected_marker_id =11;
		}
		else if (msg->parameters[1].value == "m2"){
			selected_marker_id =12;
		}
		else if (msg->parameters[1].value == "m3"){
			selected_marker_id =13;
		
		}
		else if (msg->parameters[1].value == "m4"){
			selected_marker_id =15;
		}
		// Setting the goal based on the selected marker
		// In order to achieve the goal, two goals are set: one for the move_base action and one for the reaching_goal action
		// The move_base action is used to move the robot to the selected marker's position
		// The reaching_goal action is used to verify if the robot has reached the selected marker's position
		for (int i = 0; i<MAX_MARKERS+1; i++) {
			if (marker_ids[i] == selected_marker_id) {
				goal.target_pose.pose.position.x = marker_positions_x[i];
				goal.target_pose.pose.position.y = marker_positions_y[i];
				goal.target_pose.pose.orientation.w = 1.0;
				goal1.target_pose.pose.position.x = marker_positions_x[i];
				goal1.target_pose.pose.position.y = marker_positions_y[i];
				break;
			}
		}
		// Sending the goals to the action servers
		if (selected_marker_id != -1) {
			client.sendGoal(goal1);
			ac.sendGoal(goal);
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
}

/**
* \brief Main function
*/
int main(int argc, char **argv) {
	ros::init(argc, argv, "my_rosplan_go_to_marker_action", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	// Subscribing to the detected_markers topic
	ros::Subscriber detected_markers_sub = nh.subscribe("/detected_markers", 1, markerCallback);
	// Creating the action interface
	KCL_rosplan::MyActionInterface my_aci(nh);
	my_aci.runActionInterface();
	return 0;
}


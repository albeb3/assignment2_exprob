#include "assignment2_exprob/planning_action.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <geometry_msgs/PoseStamped.h>
#include <assignment2_exprob/Marker_id_pos.h>
#include <assignment2_exprob/MarkerArray.h>
#include <assignment2_exprob/PlanningAction.h>
#include <assignment2_exprob/PlanningGoal.h>
#include <cstring>
#include <string>
#include <vector>
const int MAX_MARKERS = 4;

int marker_ids[MAX_MARKERS];
float marker_positions_x[MAX_MARKERS];
float marker_positions_y[MAX_MARKERS];
int marker_count = 0; 
int selected_marker_id = -1;

void markerCallback(const assignment2_exprob::MarkerArray::ConstPtr& msg) {
		// here the implementation of the callback
		//std::cout << "Received " << msg->markers.size() << " markers" << std::endl;
		// Itera sui marker ricevuti
    for (const auto& marker : msg->markers) {
        if (marker_count < MAX_MARKERS) {
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
namespace KCL_rosplan {

	MyActionInterface::MyActionInterface(ros::NodeHandle &nh) {
			// here the initialization
			
	}

	
	bool MyActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {//legge dal topic /rosplan_plan_dispatcher/action_dispatch
			// here the implementation of the action 
		std::cout << "Going from " << msg->parameters[2].value << " to " << msg->parameters[3].value << std::endl;
		
		actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
		actionlib::SimpleActionClient<assignment2_exprob::PlanningAction> client("/reaching_goal", true);
		actionlib::SimpleClientGoalState state = client.getState();//receive the status
		
		move_base_msgs::MoveBaseGoal goal;
		 while(!ac.waitForServer(ros::Duration(5.0)) && !client.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }

		assignment2_exprob::PlanningGoal goal1;

		goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
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

		for (int i = 0; i<MAX_MARKERS; i++) {
			if (marker_ids[i] == selected_marker_id) {
				goal.target_pose.pose.position.x = marker_positions_x[i];
				goal.target_pose.pose.position.y = marker_positions_y[i];
				goal.target_pose.pose.orientation.w = 1.0;
				goal1.target_pose.pose.position.x = marker_positions_x[i];
				goal1.target_pose.pose.position.y = marker_positions_y[i];
				break;
			}
		}
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
        }}
}}

	int main(int argc, char **argv) {
		ros::init(argc, argv, "my_rosplan_action_2", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");

		// Definizione del subscriber
		ros::Subscriber detected_markers_sub = nh.subscribe("/detected_markers", 1, markerCallback);

		KCL_rosplan::MyActionInterface my_aci(nh);
		my_aci.runActionInterface();
		return 0;
	}


#include "assignment2_exprob/planning_action.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <geometry_msgs/PoseStamped.h>

#include <assignment2_exprob/PlanningAction.h>
#include <assignment2_exprob/PlanningGoal.h>
#include <cstring>
#include <string>



namespace KCL_rosplan {

	MyActionInterface::MyActionInterface(ros::NodeHandle &nh) {
			// here the initialization
	}

	bool MyActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {//legge dal topic /rosplan_plan_dispatcher/action_dispatch
			// here the implementation of the action 
		std::cout << "Going from " << msg->parameters[1].value << " to " << msg->parameters[2].value << std::endl;
		
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
}}

	int main(int argc, char **argv) {
		ros::init(argc, argv, "my_rosplan_action", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");
		KCL_rosplan::MyActionInterface my_aci(nh);
		my_aci.runActionInterface();
		return 0;
	}

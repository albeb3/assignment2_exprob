#include "assignment2_exprob/planning_action.h"
#include <ros/ros.h>
#include <assignment2_exprob/PlanningAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <std_msgs/Float64.h>
#include <unistd.h>

namespace KCL_rosplan {

	MyActionInterface::MyActionInterface(ros::NodeHandle &nh) {
			// here the initialization
	}

	bool MyActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {//legge dal topic /rosplan_plan_dispatcher/action_dispatch
			// here the implementation of the action 
		std::cout << "Going from " << msg->parameters[1].value << " to " << msg->parameters[2].value << std::endl;
		
		actionlib::SimpleActionClient<assignment2_exprob::PlanningAction> ac("reaching_goal", true);
		assignment2_exprob::PlanningGoal goal;
		ac.waitForServer();
		
		//INSERIRE SUBSCRIBER CHE PRENDA POSIZIONE E ID MARKER
		//Sviluppo routine che ordina le posizioni dei marker in ordine di numero id crescente
		double pos_x[4] = {-7.0, -3.0, 6.0, 7.0};
		double pos_y[4] = {-1.5, -8.0, 2.0, -5.0};

		 
		if(strcmp(msg->name.c_str(), "path_following") == 0){  
			for(int i=0;i<4;i++){
				//we'll send a goal to move the robot
				goal.target_pose.header.frame_id = "base_link";
				goal.target_pose.header.stamp = ros::Time::now();

				goal.target_pose.pose.position.x = pos_x[i];
				goal.target_pose.pose.position.y = pos_y[i];
				goal.target_pose.pose.orientation.w = 0.0;

				ROS_INFO("Sending goal");
				ac.sendGoal(goal);

				ac.waitForResult();

				if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
					ROS_INFO("Hooray, target reached!");
				else
					ROS_INFO("The base failed to reach the target for some reason");
			  }
		}
	 }
}




int main(int argc, char** argv){
  ros::init(argc, argv, "path_following_action", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  KCL_rosplan::MyActionInterface my_aci(nh);
		my_aci.runActionInterface();
		return 0;
}
  


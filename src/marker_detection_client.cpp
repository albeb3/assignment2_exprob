#include "assignment2_exprob/planning_action.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>
#include <assignment2_exprob/MarkerDetectionAction.h>

typedef actionlib::SimpleActionClient<assignment2_exprob::MarkerDetectionAction> MarkerDetectionClient;

namespace KCL_rosplan {

    MyActionInterface::MyActionInterface(ros::NodeHandle &nh) {
    // here the initialization
    }

    bool MyActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) 
    {
        // here the implementation of the action
        std::cout << "Trying to find marker " << msg->parameters[2].value << std::endl;
        
        MarkerDetectionClient fm("find_marker", true);

	    assignment2_exprob::MarkerDetectionGoal id_marker;

        while(!fm.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the find_marker action server to come up");
        }
        
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
        
        fm.sendGoal(id_marker);
        fm.waitForResult();
        
        
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

int main(int argc, char **argv) {
    ros::init(argc, argv, "find_marker_client", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    KCL_rosplan::MyActionInterface my_fm(nh);
    my_fm.runActionInterface();
    return 0;
}
    

    
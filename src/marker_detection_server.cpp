#include <ros/ros.h>
#include <unistd.h>
#include <iostream>
#include <actionlib/server/simple_action_server.h>
#include <assignment2_exprob/MarkerDetectionAction.h>
#include <std_msgs/Int32.h>
#include <assignment2_exprob/Marker_id_pos.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
//#include <assignment2_exprob/laser_direction.h>
#include <assignment2_exprob/SetVelocityControl.h>

class MarkerDetectionActionServer {
public:
    MarkerDetectionActionServer(ros::NodeHandle nh) : nh_(nh), as_(nh, "find_marker", boost::bind(&MarkerDetectionActionServer::executeCallback, this, _1), false) {
        as_.start();
        marker_pub_ = nh_.advertise<std_msgs::Int32>("/rosbot/search_id", 10);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        found_sub_ = nh_.subscribe("/robot4_xacro/marker_id_detected", 1, &MarkerDetectionActionServer::foundCallback, this);
       // laser_dir_sub_ = nh_.subscribe("/laser_direction", 1, &MarkerDetectionActionServer::laserCallback, this);
        client = nh_.serviceClient<assignment2_exprob::SetVelocityControl>("/set_velocity_control");
    }

    void executeCallback(const assignment2_exprob::MarkerDetectionGoalConstPtr &goal) {
        // Publish marker request
        ROS_INFO("Action server started and waiting for clients...");
        assignment2_exprob::SetVelocityControl srv;
        id.data = goal->goal;
        std::cout << "Finding " << id.data << std::endl;
        marker_pub_.publish(id);

        marker_found_ = false;
        srv.request.control = true;
        
        // Loop until marker is found
        while (!marker_found_) {
            if (!client.call(srv)) {
                ROS_ERROR("Failed to call service /set_velocity_control");
                break;
            }
            ros::spinOnce();
            ros::Duration(0.1).sleep();  // Adjust sleep duration as needed
        }

        std::cout << "Found " << marker_id_detected.data << std::endl;

        as_.setSucceeded();
    }
    
    void foundCallback(const assignment2_exprob::Marker_id_pos msg) {
        if (msg.marker_id != -1) {
            marker_found_ = true;
            marker_id_detected.data = msg.marker_id;
            // Additional actions on marker found can go here
        }
    }

private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<assignment2_exprob::MarkerDetectionAction> as_;
    ros::Publisher marker_pub_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber found_sub_;
    bool marker_found_ = false;
    std_msgs::Int32 id;
    std_msgs::Int32 marker_id_detected;
    //ros::Subscriber laser_dir_sub_;
    std::string global_direction;
    float global_distance;
    ros::ServiceClient client;  // Client for SetVelocityControl service
    
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "find_marker_server");
    ros::NodeHandle nh;
    ROS_INFO("Action server started and waiting for clients...");
    MarkerDetectionActionServer server(nh);

    ros::spin();

    return 0;
}

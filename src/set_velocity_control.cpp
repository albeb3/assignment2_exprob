/**
 * \file set_velocity_control.cpp
 * \brief set_velocity_control Node
 * \author Alberto Bono 3962994
 * \date 04/01/2025
 * 
 * \details
 * ActionServer:<BR>
 *    /set_velocity_control<BR>
 * 
 * Subscribes to:<BR>
 *   /laser_direction<BR>
 *  /robot4_xacro/marker_id_detected<BR>
 * 
 * Publishes to:<BR>
 *  /cmd_vel<BR>
 * 
 * Description:<BR>
 * This node receives the direction and distance from the laser via the laser_direction message<BR>
 * and sends it to the marker_detection_server action server.<BR>
 */
// Include some necessary libraries
#include <ros/ros.h>
#include <assignment2_exprob/SetVelocityControl.h>
#include <geometry_msgs/Twist.h>
// Include the message header file
#include <assignment2_exprob/laser_direction.h>
#include <assignment2_exprob/Marker_id_pos.h>
// Declare the global variables
ros::Publisher cmd_vel_pub;  ///< Publisher for the cmd_vel topic
ros::Subscriber laser_dir_sub_;  ///< Subscriber for the laser_direction topic
ros::Subscriber found_sub_; ///< Subscriber for the marker_id_detected topic
std::string global_direction; ///< Variable to store the global direction
float global_distance; ///< Variable to store the global distance
bool marker_found_ = false; ///< Variable to check if the marker is found
bool direction = false; ///< Variable to check the direction of the robot

// Function to rotate the robot based on the angular velocity
void rotate_rosbot(double ang_z)
{
    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.angular.z = ang_z;
    cmd_vel_pub.publish(cmd_vel_msg);
}
// Function to move the robot based on the linear velocity
void move_rosbot(double lin_x)
{
    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = lin_x;
    cmd_vel_pub.publish(cmd_vel_msg);
}
/**
 * \brief Callback function for the /set_velocity_control service
 * \param req(assignment2_exprob::SetVelocityControl::Request)<BR>
 * \param res(assignment2_exprob::SetVelocityControl::Response)<BR>
 * \result controls the robot based on the direction and distance from the object
 */
bool setVelocityControl(assignment2_exprob::SetVelocityControl::Request &req,
                        assignment2_exprob::SetVelocityControl::Response &res)
{
    // Check if the control is enabled
    if (req.control) {
        // Routine to control the robot based on the direction and distance from the object
        if ((global_direction == "left" || global_direction == "fleft") && global_distance < 1.4) {
            // Rotate the robot to the right if the distance is less than 1.5
            rotate_rosbot(0.4);
        } 
        else if ((global_direction == "right" || global_direction == "fright") && global_distance < 1.4) {
            // Rotate the robot to the left if the distance is less than 1.5
            rotate_rosbot(-0.4);
        } 
        else if (global_direction == "front" && global_distance < 1.4 ) {
            // Stop rotate the robot if the distance is less than 1.5 and the object is in front
            rotate_rosbot(0.0);
            // If the distance is less than 0.6, move the robot back for 3 seconds
            if (global_distance < 0.6 )
            {
                ros::Time start_time = ros::Time::now();
                while (ros::Time::now() - start_time < ros::Duration(3.0)) {
                    move_rosbot(-0.1);
                    ros::Duration(0.1).sleep();  // Sleep for 100ms between commands
                }
            }
            // If the distance is greater than 0.6, move the robot forward for 3 seconds
            else if ( global_distance > 0.6 )
            {
                ros::Time start_time = ros::Time::now();
                while (ros::Time::now() - start_time < ros::Duration(3.0)) {
                    move_rosbot(0.1);
                    ros::Duration(0.1).sleep();  // Sleep for 100ms between commands
                }
            }
        }
        // If the object is not in front, rotate the robot 
        else {
            rotate_rosbot(0.4);
        }
        res.success = true;
    }
    // Disable the velocity control
    else {
        ROS_INFO("Velocity control disabled.");
        move_rosbot(0.0);
        rotate_rosbot(0.0);
        res.success = true;
    }

    return true;
}
// Callback function for the /laser_direction topic
void laserCallback(const assignment2_exprob::laser_direction::ConstPtr& msg)
{
    // Read the direction and distance from the laser_direction message and store it in the global variables
    global_direction = msg->direction;
    global_distance = msg->distance;
}
void foundCallback(const assignment2_exprob::Marker_id_pos msg) {
    if (msg.marker_id != -1) {
        marker_found_ = true;
    }
}
/**
 * \brief Main function
 */
int main(int argc, char** argv)
{
    // Initialize the node
    ros::init(argc, argv, "velocity_control_server");
    ros::NodeHandle nh;
    // Publisher for the cmd_vel topic
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    // Service server for the set_velocity_control service
    ros::ServiceServer service = nh.advertiseService("/set_velocity_control", setVelocityControl);
    // Subscriber for the laser_direction topic
    laser_dir_sub_ = nh.subscribe("/laser_direction", 1, laserCallback);
    // Subscriber for the marker_id_detected topic
    found_sub_ = nh.subscribe("/robot4_xacro/marker_id_detected", 1, foundCallback);
    ROS_INFO("Velocity control service ready.");
    ros::spin();
    return 0;
}

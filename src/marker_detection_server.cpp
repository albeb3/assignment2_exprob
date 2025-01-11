/**
 * \file marker_detection_server.cpp
 * \brief find_marker_server_action Node
 * \author Alberto Bono 3962994
 * \date 04/01/2025
 * 
 * \details
 * ActionServer:<BR>
 *     /find_marker<BR>
 *
 * SimpleActionServer:<BR>
 *    /set_velocity_control<BR>
 * 
 * Subscribes to:<BR>
 *    /robot4_xacro/marker_id_detected<BR>
 * 
 * Publishes to:<BR>
 *    /cmd_vel<BR>
 *    /detected_markers<BR>
 * 
 * Description:<BR>
 * This node receives the goal from the action client via the find_marker message.
 * It then activates the velocity control service to search for the marker,
 * It finally receives the detected marker_id and position from the robot4_xacro/marker_id_detected topic,
 * and fills the detected_markers list with the detected markers and their positions.
 * When the list has 4 markers, it publishes the list to the detected_markers topic.
 */
// Include some necessary libraries
#include <ros/ros.h>
#include <unistd.h>
#include <iostream>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <vector>
// Include the action header file
#include <assignment2_exprob/MarkerDetectionAction.h>
// Include the message header file
#include <assignment2_exprob/Marker_id_pos.h> 
#include <assignment2_exprob/MarkerArray.h>     
// Include the service message header file
#include <assignment2_exprob/SetVelocityControl.h> 
// Define the MarkerDetectionActionServer
class MarkerDetectionActionServer {
    public:
        // Constructor
        MarkerDetectionActionServer(ros::NodeHandle nh) : nh_(nh), as_(nh, "find_marker", boost::bind(&MarkerDetectionActionServer::executeCallback, this, _1), false) {
            // Start the action server
            as_.start();
            // Create a client for the SetVelocityControl service
            client = nh_.serviceClient<assignment2_exprob::SetVelocityControl>("/set_velocity_control");
            // Subscribe to the marker_id_detected topic
            found_sub_ = nh_.subscribe("/robot4_xacro/marker_id_detected", 1, &MarkerDetectionActionServer::foundCallback, this);
            // Publish to the cmd_vel topic
            cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
            // Publish to the detected_markers topic
            detected_markers_pub = nh_.advertise<assignment2_exprob::MarkerArray>("/detected_markers", 10);
        }
        /**
        * \brief Callback function for the /find_marker action server
        * \param goal(const assignment2_exprob::MarkerDetectionGoalConstPtr)
        */
        void executeCallback(const assignment2_exprob::MarkerDetectionGoalConstPtr &goal) {
            // here the implementation of the action
            ROS_INFO("Action server started and waiting for clients...");
            // Creating the service message
            assignment2_exprob::SetVelocityControl srv;
            srv.request.control = true;

            marker_found_ = false; // variable to check if the marker is found
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
        /**
        * \brief Callback function for the /robot4_xacro/marker_id_detected topic
        * \param msg(const assignment2_exprob::Marker_id_pos)
        * \details
        * This function receives the detected marker_id and position from the robot4_xacro/marker_id_detected topic
        * and fills the detected_markers list with the detected markers and their positions.
        * When the list has 4 markers, it publishes the list to the detected_markers topic.
        * If the marker_id is already present in the list, it is ignored.
        * \result fills the detected_markers list with the detected markers and their positions
        */
        void foundCallback(const assignment2_exprob::Marker_id_pos msg) {
            if (msg.marker_id == 11 || msg.marker_id == 12 || msg.marker_id == 13 || msg.marker_id == 15){
           // if (msg.marker_id != -1) {
                marker_found_ = true;
                marker_id_detected.data = msg.marker_id;
                bool already_present = false;
                // Check if the marker_id is already present in the list
                for (const auto& marker : detected_markers_list) {
                    if (marker.marker_id == msg.marker_id) {
                        already_present = true;
                        break;
                    }
                }
                // If the marker_id is not present, add it to the list
                if (!already_present) {
                    detected_markers_list.push_back(msg);
                    ROS_INFO("Inserito marker_id: %d nella lista.", msg.marker_id);
                } else {
                    ROS_WARN("Marker_id: %d già presente nella lista. Ignorato.", msg.marker_id);
                }
                // Check if the list has 4 markers
                if (detected_markers_list.size() == 4) {
                    ROS_INFO("Raggiunti 4 marker! Pronto per pubblicare.");
                    // Create a message of type MarkerArray
                    assignment2_exprob::MarkerArray marker_array_msg;   
                    // Fill the message with the detected markers
                    marker_array_msg.markers = detected_markers_list;
                    // Publish the message to the detected_markers topic
                    detected_markers_pub.publish(marker_array_msg); // Se hai un publisher configurato
                    // Clear the list                
                    detected_markers_list.clear();
                }
            }
        }
    private:
        // Private variables
        ros::NodeHandle nh_; ///< Node handle
        actionlib::SimpleActionServer<assignment2_exprob::MarkerDetectionAction> as_;  ///< Action server
        ros::Publisher cmd_vel_pub_; ///< Publisher for the cmd_vel topic
        ros::Publisher detected_markers_pub; ///< Publisher for the detected_markers topic
        ros::Subscriber found_sub_; ///< Subscriber for the marker_id_detected topic
        ros::ServiceClient client;  ///< Client for the SetVelocityControl service
        bool marker_found_ = false; ///< Variable to check if the marker is found
        std_msgs::Int32 id; ///< Message of type Int32 to store the marker_id
        std_msgs::Int32 marker_id_detected; ///< Message of type Int32 to store the detected marker_id
        std::string global_direction;  ///< Variable to store the global direction
        float global_distance;   ///< Variable to store the global distance
        std::vector<assignment2_exprob::Marker_id_pos> detected_markers_list;  ///< Vector to store the detected markers
};
/**
* \brief Main function 
*/
int main(int argc, char** argv) {
    ros::init(argc, argv, "find_marker_server_action");
    ros::NodeHandle nh;
    ROS_INFO("Action server started and waiting for clients...");
    ///< Create the MarkerDetectionActionServer
    MarkerDetectionActionServer server(nh);
    ros::spin();
    return 0;
}

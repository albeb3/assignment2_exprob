/*****************************
 Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are
 permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of
 conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list
 of conditions and the following disclaimer in the documentation and/or other materials
 provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are those of the
 authors and should not be interpreted as representing official policies, either expressed
 or implied, of Rafael Muñoz Salinas.
 ********************************/
/**
 * @file marker_publish.cpp
 * @author Bence Magyar
 * @date June 2014
 * @brief Modified copy of simple_single.cpp to publish all markers visible
 * (modified by Josh Langsfeld, 2014)
 */
/**
 * \file marker_publish.cpp
 * \author Alberto Bono 3962994
 * \date 04/01/2025
 * \brief Modified copy of marker_publish.cpp to publish the detected markers
 * 
 * \details
 * Publishes to:<BR>
 *   /robot4_xacro/marker_id_detected<BR>
 * 
 * Description:<BR>
 * This node is a modified copy of marker_publish.cpp to publish the detected markers.
 * The modification consists of publishing the detected markers to the /robot4_xacro/marker_id_detected topic.
 * The robot's position is also published along with the marker_id.
 */

///< Include some necessary libraries
#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int32.h>
#include <assignment2_exprob/Marker_id_pos.h>
#include <unistd.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
class ArucoMarkerPublisher
{
private:
  // Aruco stuff
  aruco::MarkerDetector mDetector_; ///< Marker detector
  std::vector<aruco::Marker> markers_; ///< Vector of detected markers
  aruco::CameraParameters camParam_; ///< Camera parameters

  // node params
  double marker_size_; ///< Marker size
  bool useCamInfo_; ///< Use camera info

  // Initialising the robot's position
  double robot_x_;  ///< Robot's x position
  double robot_y_;  ///< Robot's y position
  
  // ROS pub-sub
  ros::NodeHandle nh_; ///< Node handle
  image_transport::ImageTransport it_; ///< Image transport
  image_transport::Subscriber image_sub_; ///< Subscriber for the image topic
  ros::Subscriber search_sub; ///< Subscriber for the search topic

  ros::Publisher marker_pub_= nh_.advertise<assignment2_exprob::Marker_id_pos>("/robot4_xacro/marker_id_detected", 10);   ///< Publisher for the marker_id_detected topic
  ros::Subscriber sub; ///< Subscriber for the odom topic
  image_transport::Publisher image_pub_;  ///< Publisher for the image topic
  image_transport::Publisher debug_pub_; ///< Publisher for the debug topic
  cv::Mat inImage_; ///< Image variable
  
public:
  // Constructor
  ArucoMarkerPublisher() :
      nh_("~"), it_(nh_), useCamInfo_(true)
  {
    // Subscribe to the camera image topic
    image_sub_ = it_.subscribe("/robot4_xacro/camera1/image_raw", 1, &ArucoMarkerPublisher::image_callback, this);
    // Subscribe to the odom topic
    sub = nh_.subscribe("/odom", 10, &ArucoMarkerPublisher::odomCallback, this);
    // Publisher for the image topic
    image_pub_ = it_.advertise("result", 1);
    debug_pub_ = it_.advertise("debug", 1);
    nh_.param<bool>("use_camera_info", useCamInfo_, false);
    camParam_ = aruco::CameraParameters();
  }
  /**
   * \brief Callback function for the /odom topic
   * \param msg(const nav_msgs::Odometry)
   * \details
   * This function reads the robot's position from the odometry message
   * and stores it in the robot_x_ and robot_y_ variables.
   * 
   */
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Read the robot's position from the odometry message and store it in the robot_x_ and robot_y_ variables
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    }
  /**
  * \brief Callback function for the /robot4_xacro/camera1/image_raw topic
  * \param msg(const sensor_msgs::ImageConstPtr)
  * \details
  * This function reads the image from the camera and detects the markers in it.
  * It then publishes the detected markers to the /robot4_xacro/marker_id_detected topic.
  */
  void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    bool publishImage = image_pub_.getNumSubscribers() > 0;
    bool publishDebug = debug_pub_.getNumSubscribers() > 0;

    ros::Time curr_stamp = msg->header.stamp;
    cv_bridge::CvImagePtr cv_ptr;
    
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      inImage_ = cv_ptr->image;
   
      // clear out previous detection results
      markers_.clear();

      // ok, let's detect
      mDetector_.detect(inImage_, markers_, camParam_, marker_size_, false);

      // Define the Marker_id_pos message
      assignment2_exprob::Marker_id_pos marker_id_pos;

      if (markers_.empty()) {
          // If no marker is detected, set marker_id to -1 and publish the position
          marker_id_pos.marker_id = -1;  // No marker detected
          marker_id_pos.pose.position.x = robot_x_;
          marker_id_pos.pose.position.y = robot_y_;
          marker_pub_.publish(marker_id_pos);
      } else {
          // If markers are detected, publish each one with the robot's position
          for (std::size_t i = 0; i < markers_.size(); ++i)
          {
            marker_id_pos.marker_id = markers_.at(i).id;
            marker_id_pos.pose.position.x = robot_x_;
            marker_id_pos.pose.position.y = robot_y_;
            marker_pub_.publish(marker_id_pos);
          }
      }
      std::cout << std::endl;

      // draw detected markers on the image for visualization
      for (std::size_t i = 0; i < markers_.size(); ++i)
      {
        markers_[i].draw(inImage_, cv::Scalar(0, 0, 255), 2);
      }
      // publish input image with markers drawn on it
      if (publishImage)
      {
        // show input with augmented information
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = curr_stamp;
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = inImage_;
        image_pub_.publish(out_msg.toImageMsg());
      }

      // publish image after internal image processing
      if (publishDebug)
      {
        // show also the internal image resulting from the threshold operation
        cv_bridge::CvImage debug_msg;
        debug_msg.header.stamp = curr_stamp;
        debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
        debug_msg.image = mDetector_.getThresholdedImage();
        debug_pub_.publish(debug_msg.toImageMsg());
      }

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }
};
/**
 * \brief Main function
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_marker_publisher");

  ArucoMarkerPublisher node;

  ros::spin();
}


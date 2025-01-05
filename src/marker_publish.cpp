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
  // ArUco stuff
  aruco::MarkerDetector mDetector_;
  std::vector<aruco::Marker> markers_;
  aruco::CameraParameters camParam_;

  // node params
  double marker_size_;
  bool useCamInfo_;

  //modifiche

  
  double robot_x_;
    double robot_y_;
    //double robot_theta_;  
  // ROS pub-sub
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Subscriber search_sub;
  
  //modifiche
  ros::Publisher marker_pub_= nh_.advertise<assignment2_exprob::Marker_id_pos>("/robot4_xacro/marker_id_detected", 10);
  ros::Subscriber sub;
 
  image_transport::Publisher image_pub_;
  image_transport::Publisher debug_pub_;

  cv::Mat inImage_;
  
public:
  ArucoMarkerPublisher() :
      nh_("~"), it_(nh_), useCamInfo_(true)
  {
    image_sub_ = it_.subscribe("/image", 1, &ArucoMarkerPublisher::image_callback, this);
    
   
    sub = nh_.subscribe("/odom", 10, &ArucoMarkerPublisher::odomCallback, this);
    
    image_pub_ = it_.advertise("result", 1);
    debug_pub_ = it_.advertise("debug", 1);
    
    nh_.param<bool>("use_camera_info", useCamInfo_, false);
    camParam_ = aruco::CameraParameters();
  }


  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Legge la posizione dal messaggio di odometria
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
   

     // Converti il quaternion in un angolo di Eulero per l'orientamento
        //tf::Quaternion quat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
          //                   msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        //tf::Matrix3x3(quat).getRPY(robot_theta_, 0.0, 0.0);  // Angolo di rotazione sull'asse Z

    }
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

      //modifiche
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_marker_publisher");

  ArucoMarkerPublisher node;

  ros::spin();
}


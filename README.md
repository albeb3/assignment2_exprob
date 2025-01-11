# **assignment2_exprob**

## **Project Description**
This project aims to develop a ROS1 package that enables a mobile robot equipped with a camera and a laser scanner to autonomously navigate to the waypoint with the lowest ID.

The robot has to reach these waypoints:
- **WP 0**: x = -7.0, y = -1.5  
- **WP 1**: x = -3.0, y = -8.0  
- **WP 2**: x = 6.0, y = 2.0  
- **WP 3**: x = 7.0, y = -5.0  

For each waypoint, the robot will attempt to find the marker positioned near it. After visiting each waypoint, the robot will visit all the waypoints starting from the one with the lowest marker ID and proceed in order until it reaches the waypoint with the highest ID.
---
## **Simulation Demo**
<video width="640" height="360" controls>
  <source src="simulation.mp4" type="video/mp4">
</video>
---

## **Requirements**
The following software and dependencies are required to run the project:  
- **Operating System**: Ubuntu 20.04 (compatible with ROS1 Noetic)  
- **ROS Version**: ROS1 Noetic  
- **Main Dependencies**:  
  - `actionlib`  
  - `actionlib_msgs`  
  - `geometry_msgs`  
  - `roscpp`  
  - `rospy`  
  - `rosplan_dispatch_msgs`  
  - `rosplan_knowledge_msgs`  
  - `rosplan_dependencies`  
  - `std_msgs`  
  - `sensor_msgs`  
  - `std_srvs`  
  - `tf`  
  - `message_generation`  
  - `rosplan_planning_system`  
  - `cv_bridge`  
  - `image_transport`  
  - `aruco`  
  - `aruco_msgs`  

---

## **Installation**

### **Creating a ROS1 Workspace**

Follow these steps to create and set up a ROS1 workspace:

### 1. **Create the Workspace Directory**
Create the root directory for your workspace and the `src` folder inside it:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```
### 2.Clone the project repository into your src folder:
```bash
cd ~/catkin_ws/src
git clone https://github.com/albeb3/assignment2_exprob.git
```
### 3.Install ROS Dependencies:
```bash
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
sudo apt-get install libopencv-dev
sudo apt-get install flex bison freeglut3-dev libbdd-dev python3-catkin-tools ros-noetic-tf2-bullet
sudo apt-get install ros-noetic-navigation
```
### 4.Include Additional Repositories
```bash
git clone -b noetic https://github.com/ros-perception/vision_opencv.git
git clone https://github.com/CarmineD8/aruco_ros.git
git clone https://github.com/KCL-Planning/ROSPlan.git
git clone -b noetic https://github.com/CarmineD8/SLAM_packages.git
```

### 5.Linking OpenCV Libraries

You just need to add a dependency on opencv and link OpenCV libraries in
your CMakeLists.txt as you would for any third party package:

find_package(OpenCV Required)
catkin_package(
	…
	DEPENDS OpenCV
)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} …
${OpenCV_LIBRARIES})

### 6.Copy the folder models of the aruco_ros package in /root/.gazebo/models
```bash
cp -r aruco_ros/models /root/.gazebo/models
```

### 7. Modify the CmakeLists.txt from the rosplan_dependencies package

add the following string (“-Wno-error=deprecated-copy”) in line 92 of the CMakeLists.txt file from the
rosplan_dependencies package.

### 8. Build the Workspace
``` bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
### 9. Run the Simulation
``` bash
roslaunch assignment2_exprob sim_aruco2.launch
```
---
## **Overview **

### ROSPlan Workflow
- Generate the problem.
- Generate the plan.
- Parse the plan.
- Dispatch the plan in order to reach the goal of the task.

[Plan](plan.png)


The **`goto_waypoint`** action result from the plan is an action client that takes the waypoint as an argument. Based on fixed coordinates, it sends these coordinates as a goal to the **`move_base`** action server, which moves the robot autonomously to the waypoint. However, since I encountered difficulties receiving the success response from **`move_base`**, I sent the same goal to another action server, which provided feedback indicating whether the goal was reached successfully.

Once the robot reaches the waypoint, the **`marker_detection`** action is activated. **`marker_detection`** is an action client, `my_rosplan_find_marker_client_action`, which sends a goal to the action server **`find_marker_server_action`**. The **`find_marker_server_action`** sends a request to the service **`velocity_control_server`**, which uses laser information to make the robot face towards the marker. Once a marker is found, **`find_marker_server_action`** fills a vector with a custom message containing the marker ID and the position of the robot when the marker was found. Once the vector contains four elements, a custom message with the vector is published.

After all waypoints have been visited, the plan calls another action called **`goto_marker`**. **`goto_marker`** is similar to **`goto_waypoint`** in that it is an action client, `my_rosplan_go_to_marker_action`, with the same functionality. However, instead of fixed waypoints, it subscribes to the **`/detected_markers`** topic. When the marker ID is read from the plan, it associates it with the lowest ID present in the vector of markers that have not been visited yet.



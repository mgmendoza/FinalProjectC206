# FinalProjectC206
This is the final ROS project for UC Berkeley's C206 robotics class. 

This project can be run as a simulation or in real hardware

ROS Packages in the project:
* turtlebot3_slam: To localize and build a map of the environment. This packages uses the ROS node "slam_gmapping" to create a 2D occupancy frid map using laser-based SLAM in simulation. 

* turtlebot3_simulations: This node will be executed using any of the executable scripts. It will bringup and deploy the turtlebot3 model when running on simulation mode.

* turtlebot3_navigation: This package is used to navigate through the environment given a map of the world. The node communicates with the ROS Navigation stack and sends goals for the robot to reach. The ROS navigation stack uses Dijkstra's algorithm to find a path for the robot to reach goals while avoiding obstacle.

* explore_lite: This package is used to autonomously explore an unknown environment using frontier exporation algorithm. This original development of this package can be found on 

Note: turtlebot3 submodules need to be poperly set

## Steps to run this project in the C206 Lab
1. ```roscore```
2. Connect with the robot in the C206 Lab:
```
ssh fruit@fruit
```
3. Add the password of the turtlebot3
4. In ```ssh```, bring up the turtlebot
```
roslaunchturtlebot3_bringupturtlebot3_robot.launch--screen
```
5. In ```ssh```, turn on the camera
```
ros launch realsense2_camera rs_camera.launch mode:=Manual color_width:=424 \ color_height:=240 depth_width:=424 depth_height:=240 align_depth:=true \ depth_fps:=6 color_fps:=6
```
6. Turn on ```tf``` transform
```
rosrun tf static_transform_publisher 0 0 0 0 0 0 base_footprint camera_link 100
```
7. Turn of the ```turtlebot3_slam``` package:
```
roslaunch turtlebot3_slam turtlebot3_slam.launch
```
   
8. Turn of the ```turtlebot3_navigation``` package:
```
roslaunch turtlebot3_navigation move_base.launch
```
9. Turn of the ```perception``` package:
```
rosrun perception object_detector.py
```
10. Turn of the ```explore_lite``` package:
```
roslaunch explore_lite explore.launch
```
## Steps to run this project in simulation. Note that you have to run in each terminal ```export TURTLEBOT3_MODEL=burger``` or add it to the ```~.bashrc``` file:
1. ```roscore```
2. Bringup the turtlebot and its world (you may choose another world):
```
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
3. Turn of the ```turtlebot3_slam``` package:
```
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```
4. Turn of the ```turtlebot3_navigation``` package:
```
roslaunch turtlebot3_navigation move_base.launch
```
5. Turn of the ```perception``` package:
```
rosrun perception object_detector.py
```
6.  Turn of the ```explore_lite``` package:
```
roslaunch explore_lite explore.launch
```

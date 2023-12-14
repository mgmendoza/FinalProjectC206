# Final Project C206: Pirate Bot Treasure Hunt
This is the final ROS project for UC Berkeley's C206 robotics class. Teach a Turtlebot to play "Find the Treasure". A Turtlebot is placed in an unseen scenario with
obstacles to find a specific item. The Turtlebot will survey the scenario, plan a trajectory to go
through all possible hiding places, and search using its onboard camera and Computer Vision (CV) for a
specific item it was ordered to find. This will include algorithms from path planning, CV, and SLAM.

## Project Goals
Our goal is to teach a turtlebot to find an object inside an environment. The environment and the object's location will be unknown to the Turtlebot. The location of the obstacles will also vary every time we command the turtlebot to find the object. The Turtlebot should be able to navigate the environment
and avoid obstacles to find the object of interest. Once the turtle has found the object, it will stop and send a message indicating it has completed the task.

This project can be run in simulation or hardware

## Software and Hardware

Turtlebot with the included lab’s hardware:
* Two main drive wheels DYNAMIXEL
* IMU (Gyroscope 3 axis, Accelerometer 3-axis)
* Power connectors and peripherals
* Single-board computer (Raspberry Pi running Linux, ROS Noetic, and OpenCR)
* Battery (Lithium polymer 11.1V 1800mAh / 19.98Wh 5C)
* Intel RealSense 3D Camera with integrated IMUs and SLAM.

Our software will include the following packages:

1. Linux OS: Ubuntu 20.04 (included in lab’s computer)
   
2. ROS Packages in the project:

* **turtlebot3_slam**: To localize and build a map of the environment. This package uses the ROS node "slam_gmapping" to create a 2D occupancy grid map using laser-based SLAM in simulation. 

* **turtlebot3_simulations**: This node will be executed using any of the executable scripts. It will bring up and deploy the turtlebot3 model when running in simulation mode.

* **turtlebot3_navigation**: This package is used to navigate through the environment given a world map. The node communicates with the ROS Navigation stack and sends goals for the robot to reach. The ROS navigation stack uses Dijkstra's algorithm to find a path for the robot to reach goals while avoiding obstacles.

* **explore_lite**: This package is used to explore an unknown environment using a greedy frontier-based exploration algorithm autonomously. Documentation for this package can be found in this [Wiki Page](https://wiki.ros.org/explore_lite). The original development of this package can be found in [this repository](https://github.com/hrnr/m-explore). Note that we have modified this code to add additional functionality to this project. 

*Note: turtlebot3 submodules need to be properly set*

## Installation and Building

1. Install ROS noetic in Ubuntu 20.04
2. If you do not have Ubuntu 20.04 you can do so using Docker. We created a Dockerfile inside the ```/docker```. See the section Using Docker.
3. Clone this repository
 ``` 
 git clone -b explore_dev git@github.com:mgmendoza/FinalProjectC206.git
 ```
4. Run ```catkin_make``` in the workspace folder
## Running

### Steps to run this project in the C206 Lab
1. ```roscore```
2. Connect with the robot in the C206 Lab:
```
ssh fruit@fruit
```
3. Add the password of the Turtlebot3
4. In ```ssh```, bring up the Turtlebot
```
roslaunch turtlebot3_bringup turtlebot3_robot.launch--screen
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
### Steps to run this project in simulation. 
Note: You might have to run in each terminal ```export TURTLEBOT3_MODEL=burger``` or add it to the ```~.bashrc``` file:
1. ```roscore```
2. Bringup the Turtlebot and its world (you may choose another world):
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

## Using Docker
This section is only for those who do not have Ubunto 20.04 and wish to try this project. 
1. Install Docker on your computer. Follow the directions on the [Docker website](https://docs.docker.com/engine/install/) for installation and testing according to the OS you have.
2. After you clone this repository, go to the ```cd ./docker```

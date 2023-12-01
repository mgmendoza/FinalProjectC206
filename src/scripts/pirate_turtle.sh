#!/bin/sh

xterm -e "rosrun tf static_transform_publisher 0 0 0 0 0 0 base_footprint camera_link 100" &
sleep 1

xterm -e "roslaunch turtlebot3_slam turtlebot3_slam.launch" &
sleep 5

xterm -e "roslaunch turtlebot3_navigation move_base.launch" &
sleep 5

xterm -e "rosrun perception object_detector.py" &
sleep 5

xterm -e "roslaunch explore_lite explore.launch" &

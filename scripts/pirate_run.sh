#!/bin/sh

xterm -e "roslaunch turtlebot3_slam turtlebot3_slam.launch" &

sleep 5

xterm -e "roslaunch turtlebot3_navigation move_base.launch" &

sleep 5

xterm -e "roslaunch explore_lite explore.launch" & 

sleep 5

xterm -e "rosrun perception object_detector.py" 
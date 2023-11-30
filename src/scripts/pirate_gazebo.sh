#!/bin/sh

xterm -e "roslaunch turtlebot3_gazebo turtlebot3_house.launch" &

sleep 5

xterm -e "roslaunch turtlebot3_slam turtlebot3_slam.launch" &

sleep 5

xterm -e "roslaunch turtlebot3_navigation move_base.launch" &

sleep 5

xterm -e "roslaunch explore_lite explore.launch" &

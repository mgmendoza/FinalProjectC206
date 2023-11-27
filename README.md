# FinalProjectC206
This is the final ROS project for UC Berkeley's C206 robotics class.


## Steps to run this project
1. roscore
2. Connect with the robot:
ssh kiwi@kiwi

In ssh: bring up the turtlebot
roslaunchturtlebot3_bringupturtlebot3_robot.launch--screen

turn on the camera
ros launch realsense2_camera rs_camera.launch mode:=Manual color_width:=424 \ color_height:=240 depth_width:=424 depth_height:=240 align_depth:=true \ depth_fps:=6 color_fps:=6

rosrun tf static_transform_publisher 0 0 0 0 0 0 base_footprint camera_link 100

rviz rviz

roslaunch turtlebot3_slam turtlebot3_slam.launch

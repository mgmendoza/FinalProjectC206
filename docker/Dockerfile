FROM osrf/ros:noetic-desktop-full
ARG USER=user
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y git ros-noetic-moveit ros-noetic-ros-controllers ros-noetic-gazebo-ros-control ros-noetic-rosserial ros-noetic-rosserial-arduino ros-noetic-roboticsgroup-upatras-gazebo-plugins ros-noetic-actionlib-tools ros-noetic-gmapping ros-noetic-navigation ros-noetic-turtlebot3 ros-noetic-turtlebot3-msgs terminator python3-pip && rm -rf /var/lib/apt/lists/*
RUN pip install flask flask-ask-sdk ask-sdk opencv-python numpy scikit-learn code
WORKDIR /home/${USER}

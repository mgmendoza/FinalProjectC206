#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import tf2_ros
import tf
import sys
import numpy as np
from geometry_msgs.msg import TransformStamped, PoseStamped, Twist, Point
from tf.transformations import quaternion_from_euler
from tf2_geometry_msgs import do_transform_pose
from trajectory import plan_curved_trajectory
from turtlebot_control import controller



def search_callback(msg):
  
    # print("msg is:", msg)
  
    if not True:
        try:
            msg2send = (msg.x, msg.y)
            print(msg2send,type(msg2send))
            trajectory = plan_curved_trajectory(msg2send) ## TODO: What is the tuple input to this function?
            
            ## TODO: write a loop to loop over our waypoints and call the controller function on each waypoint
            for i in range(len(trajectory)):
                print(trajectory[i])
                controller(trajectory[i])
        except rospy.ROSInterruptException as e:
            print("Exception thrown in planning callback: " + e)
            pass
    else:
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)
        source = 'odom'
        target = 'base_footprint'
      ## TODO: apply a lookup transform between our world frame and turtlebot frame
        trans_odom_to_base_link = tfBuffer.lookup_transform(target, source, rospy.Time(), rospy.Duration(10))

        goalXY = (2.0,0.0)
        trajectory = plan_curved_trajectory(goalXY,frame=1)

        # source = 'odom'
        # # source = 'base_link'
        # target = 'base_footprint'
        # ## TODO: apply a lookup transform between our world frame and turtlebot frame
        # trans_odom_to_base_link = tfBuffer.lookup_transform(target, source, rospy.Time(), rospy.Duration(10))

        # curXY = (trans_odom_to_base_link.transform.translation.X,trans_odom_to_base_link.transform.translation.Y)
        for i in range(len(trajectory)):
            print(trajectory[i])
            controller(trajectory[i])



if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method

  #Run this program as a new node in the ROS computation graph 
  #called /turtlebot_controller.
  rospy.init_node('turtlebot_search', anonymous=True)

  rospy.Subscriber("goal_point", Point, search_callback) ## TODO: what are we subscribing to here?
  
  rospy.spin()
#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point

def talker():
    # pub = rospy.Publisher('chatter', String, queue_size=10)
    pub = rospy.Publisher('chatter', Point, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # hello_str = "Go to object"
        # rospy.loginfo(hello_str)
        # pub.publish(hello_str)
        # pub.publish(point_msg)
        # Create a Point message
        point_msg = Point()
        point_msg.x = -1.2299994230270386
        point_msg.y = 1.72 
        point_msg.z = 0.0  # Replace with your actual z-coordinate

        rospy.loginfo(f"Publishing point: ({point_msg.x}, {point_msg.y}, {point_msg.z})")
        pub.publish(point_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import tf
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header

class ObjectDetector:
    def __init__(self):
        rospy.init_node('object_detector', anonymous=True)

        self.bridge = CvBridge()

        self.cv_color_image = None
        self.cv_depth_image = None

        self.color_image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.color_image_callback)
        self.depth_image_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_image_callback)
        self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.tf_listener = tf.TransformListener()

        self.point_pub = rospy.Publisher("detected_object_position", Point, queue_size=10)
        self.image_pub = rospy.Publisher('processed_image', Image, queue_size=10)

        rospy.spin()

    def camera_info_callback(self, msg):
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]

    def pixel_to_point(self, u, v, depth):
        X = (u - self.cx) * depth / self.fx
        Y = (v - self.cy) * depth / self.fy
        Z = depth
        return X, Y, Z

    def calculate_confidence(self, mask, contour, object_type):
        if object_type == 'basketball':
            (x, y), radius = cv2.minEnclosingCircle(contour)
            radius = int(radius)
            circle_mask = np.zeros(mask.shape[:2], dtype="uint8")
            cv2.circle(circle_mask, (int(x), int(y)), radius, 255, -1)
        elif object_type == 'pink_cup':
            x, y, w, h = cv2.boundingRect(contour)
            circle_mask = np.zeros(mask.shape[:2], dtype="uint8")
            cv2.rectangle(circle_mask, (x, y), (x + w, y + h), 255, -1)

        masked = cv2.bitwise_and(mask, mask, mask=circle_mask)
        total_pixels = cv2.countNonZero(circle_mask)
        matching_pixels = cv2.countNonZero(masked)
        
        if total_pixels == 0:
            return 0
        return (matching_pixels / float(total_pixels)) * 100

    def color_image_callback(self, msg):
        try:
            self.cv_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if self.cv_depth_image is not None:
                self.process_images()
        except Exception as e:
            rospy.logerr("Error in color_image_callback: {}".format(e))

    def depth_image_callback(self, msg):
        try:
            self.cv_depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except Exception as e:
            rospy.logerr("Error in depth_image_callback: {}".format(e))

    def detect_and_calculate_angle(self, image, object_type, color_lower, color_upper):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_image, color_lower, color_upper)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        mask = cv2.erode(mask, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, None, None

        largest_contour = max(contours, key=cv2.contourArea)
        confidence = self.calculate_confidence(mask, largest_contour, object_type)

        if confidence < 40:  # Threshold for confidence
            return None, None, confidence

        if object_type == 'basketball':
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            # Minimum radius size check for basketball
            if radius < 30:  # Adjust this threshold as needed
                return None, None, confidence
            center = (int(x), int(y))
            cv2.circle(image, center, int(radius), (0, 255, 0), 2)
        elif object_type == 'pink_cup':
            x, y, w, h = cv2.boundingRect(largest_contour)
            bottom_y = y + h
            cv2.rectangle(image, (x, y), (x + w, bottom_y), (255, 0, 0), 2)
            center = (x + w // 2, bottom_y)

        return center, largest_contour, confidence

    def process_images(self):
        orange_lower = np.array([10, 100, 20], np.uint8)
        orange_upper = np.array([25, 255, 255], np.uint8)
        pink_lower = np.array([140, 50, 50], np.uint8)
        pink_upper = np.array([170, 255, 255], np.uint8)

        basketball_center, _, basketball_confidence = self.detect_and_calculate_angle(
            self.cv_color_image, 'basketball', orange_lower, orange_upper)
        pink_cup_center, _, pink_cup_confidence = self.detect_and_calculate_angle(
            self.cv_color_image, 'pink_cup', pink_lower, pink_upper)

        for center, confidence in [(basketball_center, basketball_confidence), (pink_cup_center, pink_cup_confidence)]:
            if center and confidence >= 60:
                depth = self.cv_depth_image[center[1], center[0]]
                camera_x, camera_y, camera_z = self.pixel_to_point(center[0], center[1], depth)
                camera_link_x, camera_link_y, camera_link_z = camera_z, -camera_x, -camera_y
                camera_link_x /= 1000  # Convert from mm to m
                camera_link_y /= 1000
                camera_link_z /= 1000

                try:
                    self.tf_listener.waitForTransform("/odom", "/camera_link", rospy.Time(), rospy.Duration(10.0))
                    point_odom = self.tf_listener.transformPoint("/odom", PointStamped(header=Header(stamp=rospy.Time(), frame_id="/camera_link"), point=Point(camera_link_x, camera_link_y, camera_link_z)))
                    self.point_pub.publish(point_odom.point)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logerr("TF Error: " + str(e))
                    return

        ros_image = self.bridge.cv2_to_imgmsg(self.cv_color_image, "bgr8")
        self.image_pub.publish(ros_image)


if __name__ == '__main__':
    try:
        ObjectDetector()
    except rospy.ROSInterruptException:
        pass

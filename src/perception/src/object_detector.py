#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tf
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header


class ObjectDetector:
    def __init__(self):
        rospy.init_node("object_detector", anonymous=True)

        self.bridge = CvBridge()

        self.cv_color_image = None
        self.cv_depth_image = None

        self.color_image_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.color_image_callback
        )
        self.depth_image_sub = rospy.Subscriber(
            "/camera/aligned_depth_to_color/image_raw", Image, self.depth_image_callback
        )

        self.tf_listener = tf.TransformListener()  # Create a TransformListener object

        self.point_pub = rospy.Publisher("goal_point", Point, queue_size=10)
        self.image_pub = rospy.Publisher("detected_object", Image, queue_size=10)

        rospy.spin()

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

    def detect_object_and_calculate_confidence(
        self, image, object_type, color_lower, color_upper
    ):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_image, color_lower, color_upper)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        mask = cv2.erode(mask, kernel, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None, None, None, 0

        largest_contour = max(contours, key=cv2.contourArea)
        circle_mask = np.zeros(mask.shape[:2], dtype="uint8")
        center = None
        radius = 0

        if object_type == "basketball":
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            cv2.circle(circle_mask, (int(x), int(y)), int(radius), 255, -1)
            center = (int(x), int(y))

        masked = cv2.bitwise_and(mask, mask, mask=circle_mask)
        total_pixels = cv2.countNonZero(circle_mask)
        matching_pixels = cv2.countNonZero(masked)
        confidence = (
            (matching_pixels / float(total_pixels)) * 100 if total_pixels > 0 else 0
        )

        if confidence > 15:
            cv2.circle(image, center, int(radius), (0, 255, 0), 3)
        return largest_contour, confidence, center, radius

    def process_images(self):
        orange_lower = np.array([0, 210, 180], np.uint8)
        orange_upper = np.array([30, 250, 255], np.uint8)

        (
            basketball_contour,
            basketball_confidence,
            basketball_center,
            basketball_radius,
        ) = self.detect_object_and_calculate_confidence(
            self.cv_color_image, "basketball", orange_lower, orange_upper
        )

        rospy.loginfo(f"Basketball Confidence: {basketball_confidence}%")

        if basketball_center is not None:
            depth = self.cv_depth_image[basketball_center[1], basketball_center[0]]
            if depth > 0:
                try:
                    camera_link_x, camera_link_y, camera_link_z = self.pixel_to_point(
                        basketball_center[0], basketball_center[1], depth
                    )

                    self.tf_listener.waitForTransform(
                        "/odom", "/camera_link", rospy.Time(), rospy.Duration(10.0)
                    )
                    point_odom = self.tf_listener.transformPoint(
                        "/odom",
                        PointStamped(
                            header=Header(stamp=rospy.Time(), frame_id="/camera_link"),
                            point=Point(camera_link_x, camera_link_y, camera_link_z),
                        ),
                    )
                    X_odom, Y_odom, Z_odom = (
                        point_odom.point.x,
                        point_odom.point.y,
                        point_odom.point.z,
                    )
                    rospy.loginfo(
                        "Real-world coordinates in odom frame: (X, Y, Z) = ({:.2f}m, {:.2f}m, {:.2f}m)".format(
                            X_odom, Y_odom, Z_odom
                        )
                    )

                    if X_odom < 0.001 and X_odom > -0.001:
                        Z_odom = -100
                        X_odom = -100
                        Y_odom = -100
                        rospy.loginfo(
                            "Real-world coordinates not detected: (X, Y, Z) = ({:.2f}m, {:.2f}m, {:.2f}m)".format(
                                X_odom, Y_odom, Z_odom
                            )
                        )
                        self.point_pub.publish(Point(X_odom, Y_odom, Z_odom))
                    else:
                        rospy.loginfo(
                            "Publishing goal point: {:.2f}m, {:.2f}m, {:.2f}m".format(
                                X_odom, Y_odom, Z_odom
                            )
                        )
                        self.point_pub.publish(Point(X_odom, Y_odom, Z_odom))

                        # Draw and publish the detected basketball
                        detected_image = self.cv_color_image.copy()
                        cv2.circle(
                            detected_image,
                            basketball_center,
                            int(basketball_radius),
                            (0, 0, 255),
                            2,
                        )  # Red circle around the basketball
                        cv2.circle(
                            detected_image, basketball_center, 5, (0, 255, 0), -1
                        )  # Green dot at the center
                        ros_image = self.bridge.cv2_to_imgmsg(detected_image, "bgr8")
                        self.image_pub.publish(ros_image)
                except (
                    tf.LookupException,
                    tf.ConnectivityException,
                    tf.ExtrapolationException,
                ) as e:
                    rospy.logerr("TF Error: {}".format(e))
                    return

    def pixel_to_point(self, u, v, depth):
        # Convert pixel coordinates to real-world coordinates
        X = (
            (u - self.cv_color_image.shape[1] / 2)
            * depth
            / self.cv_color_image.shape[1]
        )
        Y = (
            (v - self.cv_color_image.shape[0] / 2)
            * depth
            / self.cv_color_image.shape[0]
        )
        Z = depth
        return X, Y, Z


if __name__ == "__main__":
    try:
        ObjectDetector()
    except rospy.ROSInterruptException:
        pass

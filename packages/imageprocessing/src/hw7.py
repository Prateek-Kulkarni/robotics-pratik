#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class hw7:
    def __init__(self):
        rospy.Subscriber("image", Image, self.callback)
        self.pub = rospy.Publisher("image_cropped", Image, queue_size=10)
        self.pub1 = rospy.Publisher("image_white", Image, queue_size=10)
        self.pub2 = rospy.Publisher("image_yellow", Image, queue_size=10)
        self.bridge = CvBridge()

    def callback(self, data):
        cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        y_size= len(cv_img[:,0,0])
        cv_cropped = cv_img[int(y_size/2):,:,:]
        ros_cropped = self.bridge.cv2_to_imgmsg(cv_cropped, "bgr8")
        self.pub.publish(ros_cropped)

        cv_hsv = cv2.cvtColor(cv_cropped,cv2.COLOR_BGR2HSV)
        cv_white_filter= cv2.inRange(cv_hsv, (1,0,150),(180,30,255))
        cv_white_filtered_image= cv2.bitwise_and(cv_cropped, cv_cropped, mask=cv_white_filter)
        ros_white = self.bridge.cv2_to_imgmsg(cv_white_filtered_image, "bgr8")
        self.pub1.publish(ros_white)

        cv_yellow_filter= cv2.inRange(cv_hsv, (0,80,200),(38,255,255))
        cv_yellow_filtered_image= cv2.bitwise_and(cv_cropped, cv_cropped, mask=cv_yellow_filter)
        ros_yellow = self.bridge.cv2_to_imgmsg(cv_yellow_filtered_image, "bgr8")
        self.pub2.publish(ros_yellow)


if __name__ == "__main__":
    rospy.init_node("hw7_node")
    hw7()
    rospy.spin()
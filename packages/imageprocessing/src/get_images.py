#!/usr/bin/env python3
import rospy
import cv2
import os
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from cv_bridge import CvBridge

OUT_DIR = "/code/catkin_ws/images_output"


class ImageFilter:
    def __init__(self):
        # Instatiate the converter class once by using a class member
        self.N = 0
        self.start = False
        if not os.path.exists(OUT_DIR):
            os.mkdir(OUT_DIR)
        self.bridge = CvBridge()
        rospy.Subscriber("camera_node/image/compressed", CompressedImage, self.collect)
        rospy.Subscriber("cmd", String, self.flag)

    def flag(self, string):
        self.start = string.data == "start"
        rospy.loginfo(self.start)

    def collect(self, msg):
        rospy.loginfo("image received")
        if not self.start:
            return
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")  # convert to a cv2 image using the bridge
        img_name = f"{self.N}.jpg"
        out_path = os.path.join(OUT_DIR, img_name)
        cv2.imwrite(out_path, cv_img)
        rospy.loginfo("image saved")
        self.N += 1


if __name__ == "__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("get_images", anonymous=True)
    ImageFilter()
    rospy.spin()

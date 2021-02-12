#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32
from mystery_package.msg import UnitsLabelled
class Listener:
    def __init__(self):
        rospy.Subscriber("/mystery/output1", Float32, self.callback)
        rospy.Subscriber("/mystery/output2", UnitsLabelled, self.callback1)
    def callback(self, msg):
        rospy.loginfo("/mystery/output1")
    def callback1(self, msg):
        rospy.loginfo("/mystery/output2")
if __name__=='__main__':
    rospy.init_node('asimo1node', anonymous=True)
    Listener()

    rospy.spin()
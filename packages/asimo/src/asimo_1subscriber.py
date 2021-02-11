#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32
import mystery_package
class Listener:
    def __init__(self):
        rospy.Subscriber("/mystery/output1", Float32, self.callback)
        rospy.Subscriber("/mystery/output2", String, self.callback)
    def callback(self, msg):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data)

if __name__=='__main__':
    rospy.init_node('asimo1node', anonymous=True)
    Listener()

    rospy.spin()
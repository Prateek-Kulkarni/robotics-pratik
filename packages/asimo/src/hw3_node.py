#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from mystery_package.msg import UnitsLabelled
class hw3node:
    def __init__(self):
        rospy.Subscriber("/mystery/output2", UnitsLabelled, self.callback)
        self.pub_units = rospy.Publisher("hw3output3", UnitsLabelled, queue_size=10)
        self.total=0
        self.pub_msg = UnitsLabelled()
        self.pub_msg.units= "feet"

    def callback(self, msg):
         self.total=msg.value
         self.pub_msg.value = self.total*3.28084
         self.pub_units.publish(self.pub_msg)



if __name__ == '__main__':
    rospy.init_node('hw3_node')
    hw3node()

    rospy.spin()
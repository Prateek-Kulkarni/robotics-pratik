#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from mystery_package.msg import UnitsLabelled
class hw4:
    def __init__(self):
        rospy.Subscriber("/output2", UnitsLabelled, self.callback)
        self.pub_units = rospy.Publisher("/converted_total", UnitsLabelled, queue_size=10)
        self.pub_msg= UnitsLabelled()
        self.pub_msg.units= "meters"

    def callback(self, msg):
        if rospy.has_param("hw4units"):
                self.units = rospy.get_param("hw4units")
        else:
                self.units = "default"
                
        if self.units == "feet" :
                self.pub_msg.units="feet"
                self.total=msg.value*3.28084
                self.pub_msg.value=self.total
                self.pub_units.publish(self.pub_msg)
                rospy.loginfo("feet: %s", self.pub_msg)
                
        elif self.units == "smoots" :
                self.pub_msg.units="smoots"
                self.total=msg.value*1.7018
                self.pub_msg.value=self.total
                self.pub_units.publish(self.pub_msg)
                rospy.loginfo("smoots: %s", self.pub_msg)
                

        elif self.units == "meters" :
                self.pub_msg.units="meters"
                self.total=msg.value
                self.pub_msg.value=self.total
                self.pub_units.publish(self.pub_msg)
                rospy.loginfo("meters: %s", self.pub_msg)
                
        
if __name__=='__main__':
    rospy.init_node('Hw4')
    hw4()

    rospy.spin()

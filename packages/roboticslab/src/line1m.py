#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from duckietown_msgs.msg import Twist2DStamped

class line:
    def __init__(self):
        self.pub = rospy.Publisher("car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)

    def go(self,y,x):
        go_msg=Twist2DStamped()
        go_msg.v=y
        go_msg.omega=x

        self.pub.publish(go_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('line1m')
        pattern=line()
        rate=rospy.Rate(10)
        count=0
        for c in range(0,1):
            for count in range(0,10):
                pattern.go(0,0)#stop
                rate.sleep()
            for count in range(0,20):
                pattern.go(0.5,0)#go 1m 
                rate.sleep()
            for count in range(0,10):
                pattern.go(0,0)#stop
                rate.sleep()
            
        pattern.go(0,0)
    except rospy.ROSInterruptException:
        pass
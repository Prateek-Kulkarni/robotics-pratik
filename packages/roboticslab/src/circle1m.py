#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from duckietown_msgs.msg import Twist2DStamped

class Circle:
    def __init__(self):
        self.pub = rospy.Publisher("car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)

    def go(self,y,x):
        go_msg=Twist2DStamped()
        go_msg.v=y
        go_msg.omega=x

        self.pub.publish(go_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('circle1m')
        pattern=Circle()
        rate=rospy.Rate(2)
        count = 0
        #for count in range(0,100):#10 second timer
        while not rospy.is_shutdown():
            if count<29:
                pattern.go(0.2,1.4)
            elif count>=29:
                pattern.go(0,0)
                break
            count += 1
            rate.sleep()
        

    except rospy.ROSInterruptException:
        pass
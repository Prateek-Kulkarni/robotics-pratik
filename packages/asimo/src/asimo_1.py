#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32

def asimo1node():
    pub = rospy.Publisher('/input', Float32, queue_size=10)
    rospy.init_node('asimo_node', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    x=0
    y=1
    while not rospy.is_shutdown():
        fibonacci=x+y
        x=y
        y=fibonacci
        pub.publish(fibonacci)
        rate.sleep()

if __name__ == '__main__':
    try:
        asimo1node()
    except rospy.ROSInterruptException:
        pass

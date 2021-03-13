#!/usr/bin/env python3 
import numpy as np
import rospy
from duckietown_msgs.msg import Vector2D

class cordinatetform:
        def __init__(self):
                rospy.Subscriber('/input', Vector2D, self.callback)
                self.pub1 = rospy.Publisher('/result5', Vector2D, queue_size=10)
                self.pub2 = rospy.Publisher('/result6', Vector2D, queue_size=10)
                self.vector=Vector2D()

                
        def callback(self, msg):
                x_val=msg.x
                y_val=msg.y
                spa=np.matrix([[x_val],[y_val],[1]])
                rts=np.matrix([[-1,0,-1],[0,-1,0],[0,0,1]])
                new_v=rts*spa
                self.pub1.publish(new_v[0,0], new_v[1,0])

                wtr=np.matrix([[-0.7071,-0.7071,10],[0.7071,-0.7071,5],[0,0,1]])
                new_v1=wtr*new_v
                self.pub2.publish(new_v1[0,0], new_v1[1,0])

if __name__=='__main__':
        rospy.init_node('cordinatetransform', anonymous=True)
        cordinatetform()
        rospy.spin()
#!/usr/bin/env python3

import rospy
from duckietown_msgs.srv import SetCustomLEDPattern, ChangePattern
from std_msgs.msg import String, Float32

class Led_Lights:
    def __init__(self):
        self.obj = rospy.ServiceProxy("led_emitter_node/set_pattern", ChangePattern)
        self.pattern = [[0, 0, 0]] * 5
        self.frequency_mask = [0] * 5
        self.current_pattern_name = 'RED'
        self.changePattern(self.current_pattern_name)


if __name__ == '__main__':
    rospy.init_node("lednode")
    Led_Lights()
    # Keep it spinning to keep the node alive
    rospy.spin()
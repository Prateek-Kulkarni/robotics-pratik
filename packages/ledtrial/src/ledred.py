
#!/usr/bin/env python3

import sys
import rospy
from std_msgs.msg import String, Float32
from duckietown_msgs.srv import ChangePattern

def change_pattern_client(_color):
    rospy.wait_for_service('/fbi/led_emitter_node/set_pattern')
    led = String()
    led.data = _color
    try:
        _change_pattern_client = rospy.ServiceProxy('/fbi/led_emitter_node/set_pattern', ChangePattern)
        resp1 = _change_pattern_client(led)
        return _color
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    print("color: ", change_pattern_client("RED")) 
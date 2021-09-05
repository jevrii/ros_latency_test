#!/usr/bin/env python3

import rospy
from std_srvs.srv import *

def callback(req):
    return TriggerResponse()

if __name__ == "__main__":
    rospy.init_node('adder')
    rospy.Service('test', Trigger, callback)

    rospy.loginfo("Ready to add two ints.")
    rospy.spin()
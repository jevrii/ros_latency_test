#!/usr/bin/env python3

import rospy
from test_srv.srv import *

def callback(req):
    return TestResponse()

    # return ans

    # return AddTwoIntsResponse(sum = ans)

if __name__ == "__main__":
    rospy.init_node('adder')
    rospy.Service('test', Test, callback)

    rospy.loginfo("Ready to add two ints.")
    rospy.spin()
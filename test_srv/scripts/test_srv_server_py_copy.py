#!/usr/bin/env python3

import rospy
from test_srv.srv import *

def callback(req):
    return TestResponse()

    # return ans

    # return AddTwoIntsResponse(sum = ans)

if __name__ == "__main__":
    rospy.init_node('adder2')
    rospy.Service('test2', Test, callback)

    rospy.loginfo("Ready to add two ints.")
    rospy.spin()
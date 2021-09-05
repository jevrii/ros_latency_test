#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def callback(msg):
    sub.publish(msg.data*-1)

if __name__ == '__main__':
    rospy.init_node('pingpong_py', anonymous=True)
    sub_id = rospy.get_param('~sub_id')
    pub_id = rospy.get_param('~pub_id')
    rospy.Subscriber('ping'+str(sub_id), Int32, callback)
    sub = rospy.Publisher('ping'+str(pub_id), Int32, queue_size = 1)

    rospy.spin()

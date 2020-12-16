#!/usr/bin/env python3

import rospy
from driverless_msgs.msg import pt, ptArray

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.ptArray)

def signal():
    rospy.init_node('MPC', anonymous=True)
    return rospy.Subscriber('/global_cones', ptArray, callback)

if __name__ == '__main__':
    sub = signal()
    data = rospy.wait_for_message("/global_cones", ptArray)
    sub.unregister()
    print(data)
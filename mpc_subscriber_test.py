#!/usr/bin/env python3

import rospy
from driverless_msgs.msg import OptimizedTrackPoint, OptimizedTrackArray

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.optArray)

def listener():
    rospy.init_node('MPC', anonymous=True)
    rospy.Subscriber('/idealline_velocity', OptimizedTrackArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
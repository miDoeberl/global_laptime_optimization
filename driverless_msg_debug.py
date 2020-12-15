#!/usr/bin/env python3

import rospy
from driverless_msgs.msg import pt, ptArray

if __name__ == '__main__':
    rospy.init_node("random_pt_publisher")
    
    pub = rospy.Publisher("/my_robot/random_pts", pt, queue_size=10)
    
    rate = rospy.Rate(5)
    
    
    
    while not rospy.is_shutdown():
        msg = pt()
        msg.x = 32
        msg.y = 52
        pub.publish(msg)
        rate.sleep()
        

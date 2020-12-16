#!/usr/bin/env python3
"""
Multithreading does not seem to work. 
Runtime doubles even though two seperate threads are spawned.
Trying multiprocessing

import threading
import main_globaltraj


def sim(opt_type: str, plot: bool):
    main_globaltraj.run(opt_type, plot)


t1 = threading.Thread(target=sim, args=("mintime", False))
t2 = threading.Thread(target=sim, args=("mintime", False))

t1.start()
t2.start()

t1.join()
t2.join()
"""

from multiprocessing import Process, Queue
import main_globaltraj
import rospy
from driverless_msgs.msg import OptimizedTrackPoint, OptimizedTrackArray

def init_rosnode():
    "Returns publisher object"
    rospy.init_node("opt_track_publisher")
    pub = rospy.Publisher("/track_pts", OptimizedTrackArray, queue_size=10)
    return pub

def publish(pub, trackMatrix):
    "Publishes returnQueue Matrix to ROS publisher pub"
    trackPointArr = []
    for i in range(0, len(trackMatrix[:,0])):
        line = trackMatrix[i,:]
        point = OptimizedTrackPoint()
        point.s_m = line[0]
        point.x_m = line[1]
        point.y_m = line[2]
        point.psi_rad = line[3]
        point.kappa_radpm = line[4]
        point.vx_mps = line[5]
        point.ax_mps2 = line[6]
        trackPointArr.append(point)

    pubArr = OptimizedTrackArray(optArray=trackPointArr)


    while pub.get_num_connections() < 1:
        pass

    pointArr = OptimizedTrackArray(optArray=trackPointArr)
    pub.publish(pointArr)

# Values in queue:
# s_m: curvilinear distance along race line
# x_m: x coordinate
# y_m y coordinate
# psi_rad: Heading of raceline in current point, 0 is north
# kappa_radpm: curvature of raceline in current point
# vx_mps: target velocity
# ax_mps2: target acceleration from current point until next point
Q = Queue()

def run_sim(queue, track: str, opt_type: str, plot: bool, params_file: str):
    main_globaltraj.run(queue, track, opt_type, plot, params_file)

#myProcess = Process(target=run_sim, args=(returnQueue, "track_name/rosnode", "opt_type", plot?, "car init file"))
p1 = Process(target=run_sim, args=(Q, "rosnode", "mincurv", False, "racecar.ini"))
#p2 = Process(target=run_sim, args=(Q, "test_track", "mintime", False, "racecar.ini"))


p1.start()
#p2.start()

p1.join()

pub = init_rosnode()
trackMatrix = Q.get()
#publish(pub, trackMatrix)

#p2.join()
#trackMatrix = Q.get()
#publish(pub, trackMatrix)
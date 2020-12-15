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
p1 = Process(target=run_sim, args=(Q, "test_track", "mincurv", True, "racecar.ini"))
p2 = Process(target=run_sim, args=(Q, "test_track", "mintime", True, "racecar.ini"))


p1.start()
p2.start()

p1.join()
print(Q.get())
p2.join()
print(Q.get())
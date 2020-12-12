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

from multiprocessing import Process
import main_globaltraj


def run_sim(opt_type: str, plot: bool, params_file: str):
        main_globaltraj.run(opt_type, plot, params_file)

p1 = Process(target=run_sim, args=('mincurv', True, "racecar.ini"))
p2 = Process(target=run_sim, args=('mintime', True, "racecar.ini"))


p1.start()
p2.start()

p1.join()
p2.join()
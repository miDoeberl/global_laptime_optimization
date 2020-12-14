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

Q = Queue()

def run_sim(queue, opt_type: str, plot: bool, params_file: str):
        main_globaltraj.run(queue, opt_type, plot, params_file)

p1 = Process(target=run_sim, args=(Q, 'mincurv', False, "racecar.ini"))
p2 = Process(target=run_sim, args=(Q, 'mintime', False, "racecar.ini"))


p1.start()
p2.start()

p1.join()
print(Q.get())
p2.join()
print(Q.get())
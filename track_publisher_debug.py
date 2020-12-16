#!/usr/bin/env python3

import rospy
import numpy as np
from driverless_msgs.msg import pt, ptArray


def import_track(file_path, imp_opts, width_veh):
    # load data from csv file
    csv_data_temp = np.loadtxt(file_path, comments="#", delimiter=",")

    # get coords and track widths out of array
    if np.shape(csv_data_temp)[1] == 3:
        refline_ = csv_data_temp[:, 0:2]
        w_tr_r = csv_data_temp[:, 2] / 2
        w_tr_l = w_tr_r

    elif np.shape(csv_data_temp)[1] == 4:
        refline_ = csv_data_temp[:, 0:2]
        w_tr_r = csv_data_temp[:, 2]
        w_tr_l = csv_data_temp[:, 3]

    elif np.shape(csv_data_temp)[1] == 5:  # omit z coordinate in this case
        refline_ = csv_data_temp[:, 0:2]
        w_tr_r = csv_data_temp[:, 3]
        w_tr_l = csv_data_temp[:, 4]

    else:
        raise IOError("Track file cannot be read!")

    refline_ = np.tile(refline_, (imp_opts["num_laps"], 1))
    w_tr_r = np.tile(w_tr_r, imp_opts["num_laps"])
    w_tr_l = np.tile(w_tr_l, imp_opts["num_laps"])

    # assemble to a single array
    reftrack_imp = np.column_stack((refline_, w_tr_r, w_tr_l))

    # check if imported centerline should be flipped, i.e. reverse direction
    if imp_opts["flip_imp_track"]:
        reftrack_imp = np.flipud(reftrack_imp)

    # check if imported centerline should be reordered for a new starting point
    if imp_opts["set_new_start"]:
        ind_start = np.argmin(np.power(reftrack_imp[:, 0] - imp_opts["new_start"][0], 2) + np.power(reftrack_imp[:, 1] - imp_opts["new_start"][1], 2))
        reftrack_imp = np.roll(reftrack_imp, reftrack_imp.shape[0] - ind_start, axis=0)

    # check minimum track width for vehicle width plus a small safety margin
    w_tr_min = np.amin(reftrack_imp[:, 2] + reftrack_imp[:, 3])

    if w_tr_min < width_veh + 0.5:
        print("WARNING: Minimum track width %.2fm is close to or smaller than vehicle width!" % np.amin(w_tr_min))

    return reftrack_imp


def import_test_track():
    imp_opts = {"num_laps":1, "flip_imp_track":False, "set_new_start":False}
    return import_track("./inputs/tracks/test_track.csv", imp_opts, 2)

if __name__ == "__main__":
    
    rospy.init_node("Mapping")

    pub = rospy.Publisher("/global_cones", ptArray, queue_size=10)

    track = import_test_track()
    #Convert from m to mm
    track = np.multiply(track, 1000).astype(int)

    pubArr = []

    for i in range(len(track)):
        point = pt()
        point.x = track[i][0]
        point.y = track[i][1]
        point.w_r = track[i][2]
        point.w_l = track[i][3]
        pubArr.append(point)


    while pub.get_num_connections() < 1:
        pass
    pub.publish(pubArr)
    

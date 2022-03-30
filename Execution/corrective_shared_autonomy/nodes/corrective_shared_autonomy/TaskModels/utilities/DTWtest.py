#!/usr/bin/env python3

""" Performs FPCA using stacked, concatenated PCA
 Created: 01/21/2021
"""

__author__ = "Mike Hagenow"

import numpy as np
import matplotlib.pyplot as plt
import rospkg
import pickle
from core_robotics.filters import butter_lowpass_filter


def testDTW():
    rospack = rospkg.RosPack()
    root_dir = rospack.get_path(
        'corrective_shared_autonomy')
    path = pickle.load(open(root_dir + '/dtw.pkl', "rb"))

    diff1 = np.diff(path[0]).astype(float)
    diff2 = np.diff(path[1]).astype(float)
    diff1[0] = np.mean(diff1)
    diff2[0] = np.mean(diff2)
    diff1 = butter_lowpass_filter(diff1, 0.3, 110.0, order=1)
    diff2 = butter_lowpass_filter(diff2, 0.3, 110.0, order=1)
    plt.plot(diff1)
    plt.plot(diff2)
    plt.plot(diff2/diff1)
    plt.legend(["ref","demo","div"])
    plt.show()



if __name__ == "__main__":
    testDTW()





#!/usr/bin/env python

""" Performs FPCA using stacked, concatenated PCA
 Created: 01/21/2021
"""

__author__ = "Mike Hagenow"

import numpy as np
import matplotlib.pyplot as plt

def testSegmentation():
    y = np.zeros((400,))
    y[100:200] = np.sin(np.pi * np.array(range(0,100)) / 100.0)
    y[300:400] = np.sin(np.pi * np.array(range(0,100)) / 100.0)
    x = range(0,400)
    plt.plot(x,y)
    plt.show()


class Segmentation:
    def __init__(self):
        print("hi")

    def segment(self):
        print("hi")

if __name__ == "__main__":
    testSegmentation()





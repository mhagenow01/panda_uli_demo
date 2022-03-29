#!/usr/bin/env python3
# File name: openmp_testing.py
# Description: Used to confirm C++ Parallelism works in ROS
# Date: 10/5/2021

import time
import rospkg
import rospy
from std_msgs.msg import String

# Packages for C++ fitting
import ctypes


def openmpcallback(data):
    print("Message Received. Running test.")
    rospack = rospkg.RosPack()
    root_dir = rospack.get_path('affordance_corrections')+'/../../'

    # Ctypes testing (https://stackoverflow.com/questions/5081875/ctypes-beginner)
    # How I compiled the library: g++ -shared -o testOpenMP.so -fopenmp -fPIC testOpenMP.cpp
    testlib = ctypes.CDLL(root_dir+'src/affordance_corrections/nodes/affordance_corrections/scratch/testOpenMP.so')

    start = time.perf_counter()
    testlib.loopSerial()
    print("Serial Elapsed (s): ",time.perf_counter()-start)

    start = time.perf_counter()
    testlib.loopParallel()
    print("Parallel Elapsed (s): ",time.perf_counter()-start)

def main():

    rospy.init_node('testopenmp', anonymous=True)
    rospy.Subscriber("openmptest", String, openmpcallback)
    rospy.spin()
    
    
if __name__ == '__main__':
    main()

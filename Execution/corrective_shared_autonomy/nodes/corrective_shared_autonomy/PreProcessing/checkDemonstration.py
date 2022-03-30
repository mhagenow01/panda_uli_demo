#! /usr/bin/env python

""" Function to plot bagfile immediately after it has been captured
    for position verification

 Last Updated: 08/31/2021
"""

import rospkg
import sys
from core_robotics.bagfile_reader import bagfile_reader
from core_robotics.dataviz import highlightedthreedplot
import numpy as np
from core_robotics.ros_data_type_helpers import wrench_to_np_array, rotate_vec,pose_array_to_np_array, q_rotate_vec, float_to_np_array
from scipy import interpolate
from core_robotics.quaternion import quaternion_multiply, quaternion_flip, rotq, filter_quaternions, quaternion_inverse
from core_robotics.filters import butter_lowpass_filter


def preProcessPosition(bfr,cutoff = 10.0):

    # Relevant Topics for State Information
    mocap_topic = "/vrpn_client_node/sealant_gun/pose"

    # get data from bag file
    mocap, mocap_timestamps = bfr.get_topic_msgs(mocap_topic)

    if len(mocap)==0:
        mocap_topic = "/vrpn_client_node/roller/pose"
        mocap, mocap_timestamps = bfr.get_topic_msgs(mocap_topic)


    # Sample times constructed by diffing the timestamp datasets
    mocap_sample_time = np.mean(np.diff(mocap_timestamps))*1.0

    # Convert Data to Numpy Arrays
    p_array, q_array = pose_array_to_np_array(mocap)

    # Get lowest capture rate for interpolation
    slowest_rate_index = 0
    timesamples = mocap_timestamps

    # Interpolate all data to the slowest rate
    p_array_interp = interpolate.interp1d(mocap_timestamps, p_array, axis=0, fill_value="extrapolate")(timesamples)
    q_array_interp = interpolate.interp1d(mocap_timestamps, q_array, axis=0, fill_value="extrapolate")(timesamples)
    
    # filtering position and rotation data
    p_array_filt = butter_lowpass_filter(p_array_interp,cutoff,1.0/mocap_sample_time,axis = 0)
    q_array_filt = filter_quaternions(q_array_interp,cutOff=cutoff,fs=1.0/mocap_sample_time)

    #Return data to workspace
    data_out = np.zeros((7,len(timesamples)))
    data_out[0:3,:]=p_array_filt.T
    data_out[3:7,:]=q_array_filt.T

    return data_out

# TODO: slim down this example badly
def checkDemonstrations(names):
    xs = []
    ys = []
    zs = []
    for ii in range(0,len(names)):
        # TODO: change to just looking for position topics!
        bfr = bagfile_reader(names[ii])
        data_temp = preProcessPosition(bfr)
        xs.append(data_temp[0,:])
        ys.append(data_temp[1,:])
        zs.append(data_temp[2,:])

    highlightedthreedplot(xs[0],ys[0],zs[0],xs,ys,zs)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("No demo specified")
    else:
        names = []
        rospack = rospkg.RosPack()
        root_dir = rospack.get_path('corrective_shared_autonomy') + "/../../"
        for ii in range(1,len(sys.argv)):
            names.append(root_dir+sys.argv[ii])
        checkDemonstrations(names)
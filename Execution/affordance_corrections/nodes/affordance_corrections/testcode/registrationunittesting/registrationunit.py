#!/usr/bin/env python3
# File name: cpp_fitting_testing.py
# Description: Used to compare new C++ fitting with the existing python fitting algorithm
# Author: Mike Hagenow
# Date: 9/23/2021

import time
import os
import numpy as np
import open3d as o3d
import rospkg
from scipy.spatial import cKDTree as KDTree

from affordance_corrections.affordance_helpers.pcl_helpers import get_cloud_of_interest, get_cloud_of_interest_np
from affordance_corrections.affordance_helpers.Model import Model
from affordance_corrections.affordance_helpers.registration import get_fits_for_models, FitInfo, icp
from affordance_corrections.affordance_helpers.urdf_helpers import getPointsWithAngles

import ctypes
from affordance_corrections.affordance_helpers.cython_helpers import packageFittingInfo, convert_fits, FITHOLDER

def main():
    
    ############################
    # Set up data              #
    ############################
    rospack = rospkg.RosPack()
    root_dir = rospack.get_path('affordance_corrections')+'/../../'

    # Create two test models -- only one is the actual object
    # model1path = root_dir+'src/affordance_models/meshes/pvc_ball_valve_assem.STL'
    model1path = root_dir+'src/affordance_models/urdfs/pvc_ball_valve.urdf'
    model1 = Model(model1path)
    
    models = [model1]

    scene = o3d.io.read_point_cloud(root_dir+'pcd/valveonly2.pcd')
    scene = scene.voxel_down_sample(voxel_size=0.001)

    artic_svd_initial = True # if true, fit articulated models with SVD for random restarts, then fit once with articulations at end
    # if false, fit articulated models with NL optimization during random restarts

    ############################
    # Call the code     #
    ############################
    print("\n\nPython:")
    angles_temp = [0.1]
    R_temp = np.array([[0.9800666, -0.1986693,  0.0000000],[0.1986693,  0.9800666,  0.0000000], [0.0000000,  0.0000000,  1.0000000]])
    trans_temp = np.array([0.005, 0.005, 0.005])
    ref_point = trans_temp # this is where the user clicked
    scale = 1.0
    startt = time.time()
    cloud_trimmed = get_cloud_of_interest(scene,trans_temp,model1.diameter)
    R_new, t_new, angles_new, scale_new, error_new = icp(model1,cloud_trimmed,KDTree(cloud_trimmed),100, R_initial=R_temp,t_initial = trans_temp, angles_initial=angles_temp, scale_initial =1.0 , diameter = model1.diameter, artic_svd_initial = False)

    print(R_new,t_new,angles_new)
    print("Python time (s): ",time.time()-startt)

    aligned_source = getPointsWithAngles(model1,angles_new,R_new,t_new,scale_new)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(aligned_source)
    o3d.visualization.draw_geometries([scene,pcd])


    ############# C++ Version Using Ctypes ################
    print("\n\n\nCPP:")
    start = time.perf_counter()
    cppreglibpath = os.path.join(os.getcwd(),"build","libregistration_test.so")
    cppreglib = ctypes.CDLL(cppreglibpath)
    cppreglib.getFitsForModels.restype = ctypes.POINTER(FITHOLDER)
    
    startt = time.time()
    fitinputs = packageFittingInfo(np.asarray(scene.points),ref_point,models,artic_svd_initial)
    cppreglib.getFitsForModels(ctypes.byref(fitinputs))
    print("C++ time (s): ",time.time()-startt)
    
if __name__ == '__main__':
    main()

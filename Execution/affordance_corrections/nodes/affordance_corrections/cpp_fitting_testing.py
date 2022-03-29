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

from affordance_corrections.affordance_helpers.pcl_helpers import get_cloud_of_interest, get_cloud_of_interest_np
from affordance_corrections.affordance_helpers.Model import Model
from affordance_corrections.affordance_helpers.registration import get_fits_for_models, FitInfo
from affordance_corrections.affordance_helpers.urdf_helpers import getPointsWithAngles

import ctypes
from affordance_corrections.affordance_helpers.cython_helpers import packageFittingInfo, convert_fits, FITHOLDER

def main():
    
    ############################
    # Set up data              #
    ############################
    rospack = rospkg.RosPack()
    root_dir = rospack.get_path('affordance_corrections')+'/../../'

    ref_point = np.array([0.05, 0.04, 0.03]) # this is where the user clicked

    # Create two test models -- only one is the actual object
    model1path = root_dir+'src/affordance_models/meshes/pvc_ball_valve_assem.STL'
    model1 = Model(model1path)

    
    # Eventually try the URDF articulated version -- root_dir+'src/affordance_models/urdfs/pvc_ball_valve.urdf'
    model2path = root_dir+'src/affordance_models/meshes/e_stop_coarse.STL'
    model2 = Model(model2path)
    
    models = [model1, model2]

    scene = o3d.io.read_point_cloud(root_dir+'pcd/valveonly2.pcd')
    scene = scene.voxel_down_sample(voxel_size=0.001)

    artic_svd_initial = True # if true, fit articulated models with SVD for random restarts, then fit once with articulations at end
    # if false, fit articulated models with NL optimization during random restarts

    ############################
    # Call the fitting code    #
    ############################

    ############# Python Version #########################
    start = time.perf_counter()
    fits = get_fits_for_models(scene,ref_point,models,artic_svd_initial)
    print("Python Fitting Time (s): ",time.perf_counter()-start,"\n\n")

    # Correction model is the first one -- get fit from that one
    R_final = fits[0].rot
    t_final = fits[0].pos
    angles_final = fits[0].angles
    scale_final = fits[0].scale

    ############# C++ Version Using Ctypes ################
    print("begin with cpp")
    start = time.perf_counter()
    cppreglibpath = os.path.join(os.getcwd(), "affordance_helpers", "cppfitting", "build", "libcppregistration.so")
    cppreglib = ctypes.CDLL(cppreglibpath)
    cppreglib.getFitsForModels.restype = ctypes.POINTER(FITHOLDER)
    
    fitinputs = packageFittingInfo(np.asarray(scene.points),ref_point,models,artic_svd_initial)
   # print("fitinputs values: links: " +str (fitinputs.model_info[0]num_links))
    # Call to C++
    fitholder_ptr_unconverted = cppreglib.getFitsForModels(ctypes.byref(fitinputs))
    fits_cpp = convert_fits(fitholder_ptr_unconverted,cppreglib)
    print("C++ Fitting Time (s): ",time.perf_counter()-start,"\n\n\n")

    # Correction model is the first one -- get fit from that one
    R_final_cpp = fits_cpp[0].rot
    t_final_cpp = fits_cpp[0].pos
    angles_final_cpp = fits_cpp[0].angles
    scale_final_cpp = fits_cpp[0].scale

    ############################
    # Plot the results         #
    ############################

    print("Python: \n",np.round(R_final,3),"\n",np.round(t_final,3))
    print("Cpp: \n",R_final_cpp,"\n",t_final_cpp)

    aligned_source = getPointsWithAngles(model1,angles_final,R_final,t_final,scale_final)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(aligned_source)
    o3d.visualization.draw_geometries([scene,pcd])

    aligned_source_cpp= getPointsWithAngles(model1,angles_final_cpp,R_final_cpp,t_final_cpp,scale_final_cpp)
    pcd.points = o3d.utility.Vector3dVector(aligned_source_cpp)
    o3d.visualization.draw_geometries([scene,pcd])


    ############################
    # Other testing            #
    ############################

    # # Kevin code testing Shared Library
    # picoNNLib = ctypes.CDLL(root_dir+"src/affordance_corrections/nodes/affordance_corrections/testcode/build/libpicoNN.so")
    # picoNNLib.picoNN()
    

    
if __name__ == '__main__':
    main()

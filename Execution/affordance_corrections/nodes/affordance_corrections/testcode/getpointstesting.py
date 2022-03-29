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

# Packages for C++ fitting
import ctypes
#struct for LINK and JOINT
class LINK(ctypes.Structure):
    _fields_ = [('t',(ctypes.POINTER(ctypes.c_double))), ('R',(ctypes.POINTER(ctypes.POINTER(ctypes.c_double))))]
class JOINT(ctypes.Structure):
     _fields_ = [('axis',(ctypes.c_double * 3)), ('min_angle',ctypes.c_double), ('max_angle', ctypes.c_double), ('R',(ctypes.POINTER(ctypes.POINTER(ctypes.c_double)))), ('t',(ctypes.POINTER(ctypes.c_double)))]
class MODELINFO(ctypes.Structure):
        _fields_ = [('mesh_samples',ctypes.POINTER(ctypes.POINTER(ctypes.POINTER(ctypes.c_double)))),('links', ctypes.POINTER(LINK)), ('joints', ctypes.POINTER(JOINT)), ('num_links', ctypes.c_int), ('num_joints', ctypes.c_int),('num_mesh_points', ctypes.c_int),('diameter', ctypes.c_double)]
class FITINFO(ctypes.Structure):
    _fields_ = [('pos',(ctypes.c_double * 3)),('rot',(ctypes.c_double * 3) * 3),('num_angles',ctypes.c_int),('angles', ctypes.POINTER(ctypes.c_double)),('scale', ctypes.c_double),('residual', ctypes.c_double)]

class FITHOLDER(ctypes.Structure):
    _fields_ = [('num_fits',ctypes.c_int),('fits',ctypes.POINTER(FITINFO))]

class FITTINGINPUTS(ctypes.Structure):
        _fields_ = [('num_scene_pts',ctypes.c_int),('scene',ctypes.POINTER(ctypes.POINTER(ctypes.c_double))),('ref_point',(ctypes.c_double * 3)),('num_models',ctypes.c_int),('models', ctypes.POINTER(ctypes.c_char_p)),('model_info', ctypes.POINTER(ctypes.POINTER(MODELINFO))),('artic_svd_initial', ctypes.c_bool)]

def convert_fits(fitholder_ptr,cpp_reg_lib):
    # Convert ctype into python object for storage
    fitholder = fitholder_ptr[0] # get actual object

    fits = []

    # Load CPP result into FitInfo python object
    for kk in range(0,fitholder.num_fits):
        R_final = np.zeros((3,3))
        for ii in range(0,3):
            for jj in range(0,3):
                R_final[ii,jj] = fitholder.fits[kk].rot[ii][jj]
        
        t_final = np.array([fitholder.fits[kk].pos[0],fitholder.fits[kk].pos[1],fitholder.fits[kk].pos[2]])

        angles_final = []
        for ii in range(0,fitholder.fits[kk].num_angles):
            angles_final.append(fitholder.fits[kk].angles[ii])

        scale_final = fitholder.fits[kk].scale
        min_error = fitholder.fits[kk].residual
        
        fits.append(FitInfo(t_final,R_final,angles_final,scale_final,min_error))

    # Free the C++ memory TODO: this is not working yet
    cpp_reg_lib.freeFits(fitholder_ptr)
    return fits


def packageFittingInfo(scene,ref_point,models,artic_svd_initial):
    ###########################
    # Set up data for ctypes  #
    ###########################

    # Scene Conversion 2D dynamic array
    num_scene_pts = np.shape(scene)[0]
    SCENE_PTS = (num_scene_pts*(ctypes.POINTER(ctypes.c_double)))()
    for ii in range(0,num_scene_pts):
        SCENE_PTS[ii] = (ctypes.c_double * 3)()
        SCENE_PTS[ii][0] = scene[ii,0]
        SCENE_PTS[ii][1] = scene[ii,1]
        SCENE_PTS[ii][2] = scene[ii,2]

    #print("Py:",models[0].links[0].pts[399,2])

    # Model Files and Meshes
    # each model object has one input file stored in models array
    #MODEL_OBJ is an array of modelinfo structs
    MODEL_FILES = (len(models)*ctypes.c_char_p)()
    MODEL_OBJS = (len(models)*ctypes.POINTER(MODELINFO))()
    for ii in range(0,len(models)):
        MODEL_FILES[ii] = bytes(models[ii].name, 'utf-8')
        
        # Load the mesh (array of 2d arrays)
        model_obj = MODELINFO()
        model_obj.num_links = len(models[ii].links)
        model_obj.num_joints = len(models[ii].joints)
        model_obj.links = (model_obj.num_links*LINK)()
        for u in range (0 ,len(models[ii].links)):
            R = (3*(ctypes.POINTER(ctypes.c_double)))()
            for v in range(0,3):
                R[v] = (ctypes.c_double * 3)()
                R[v][0] = models[ii].links[u].R[v,0]
                R[v][1] = models[ii].links[u].R[v,1]
                R[v][2] = models[ii].links[u].R[v,2]
            (model_obj.links)[u].R = R
            (model_obj.links)[u].t = (ctypes.c_double * 3)()
            (model_obj.links)[u].t[0] = models[ii].links[u].t[0]
            (model_obj.links)[u].t[1] = models[ii].links[u].t[1]
            (model_obj.links)[u].t[2] = models[ii].links[u].t[2]

        model_obj.joints = (model_obj.num_joints*JOINT)()
        for y in range (0, len(models[ii].joints)):
            R = (3*(ctypes.POINTER(ctypes.c_double)))()
            for v in range(0,3):
                R[v] = (ctypes.c_double * 3)()
                R[v][0] = models[ii].joints[y].R[v,0]
                R[v][1] = models[ii].joints[y].R[v,1]
                R[v][2] = models[ii].joints[y].R[v,2]
            (model_obj.joints)[y].R = R
            (model_obj.joints)[y].t = (ctypes.c_double * 3)()
            (model_obj.joints)[y].t[0]  = models[ii].joints[y].t[0]
            (model_obj.joints)[y].t[1]  = models[ii].joints[y].t[1]
            (model_obj.joints)[y].t[2]  = models[ii].joints[y].t[2]
            (model_obj.joints)[y].axis[0] = models[ii].joints[y].axis[0]
            (model_obj.joints)[y].axis[1] = models[ii].joints[y].axis[1]
            (model_obj.joints)[y].axis[2] = models[ii].joints[y].axis[2]
            (model_obj.joints)[y].min_angle = models[ii].joints[y].min_angle
            (model_obj.joints)[y].max_angle = models[ii].joints[y].max_angle
            
    
        #MODEL_OBJ[ii].num_meshes
        model_obj.mesh_samples = (model_obj.num_links*ctypes.POINTER(ctypes.POINTER(ctypes.c_double)))()
        
        for jj in range(0,model_obj.num_links):
            num_pts = np.shape(models[ii].links[jj].pts)[0]
            model_obj.mesh_samples[jj] = (num_pts*ctypes.POINTER(ctypes.c_double))()
            for kk in range(0,num_pts):
                model_obj.mesh_samples[jj][kk] = (3*ctypes.c_double)()
                model_obj.mesh_samples[jj][kk][0] = models[ii].links[jj].pts[kk,0]
                model_obj.mesh_samples[jj][kk][1] = models[ii].links[jj].pts[kk,1]
                model_obj.mesh_samples[jj][kk][2] = models[ii].links[jj].pts[kk,2]

        model_obj.num_mesh_points = num_pts; 
        model_obj.diameter = models[ii].diameter
        MODEL_OBJS[ii] = ctypes.POINTER(MODELINFO)(model_obj) 


    # Load all inputs into container
    fitinputs = FITTINGINPUTS()
    fitinputs.num_scene_pts = num_scene_pts
    fitinputs.scene = SCENE_PTS
    # fitinputs.scene = bytes('hi', 'utf-8')
    fitinputs.ref_point = (ctypes.c_double*3)(*ref_point)
    fitinputs.num_models = len(models)
    fitinputs.models = MODEL_FILES
    fitinputs.model_info = MODEL_OBJS
    fitinputs.artic_svd_initial = artic_svd_initial   
    return fitinputs

def main():
    
    ############################
    # Set up data              #
    ############################
    rospack = rospkg.RosPack()
    root_dir = rospack.get_path('affordance_corrections')+'/../../'

    ref_point = np.array([0.05, 0.04, 0.03]) # this is where the user clicked

    # Create two test models -- only one is the actual object
    model1path = root_dir+'src/affordance_models/urdfs/pvc_ball_valve.urdf'
    # model1path = root_dir+'src/affordance_models/meshes/pvc_ball_valve_assem.STL'
    model1 = Model(model1path)
    
    models = [model1]

    scene = o3d.io.read_point_cloud(root_dir+'pcd/valveonly2.pcd')
    scene = scene.voxel_down_sample(voxel_size=0.001)

    artic_svd_initial = True # if true, fit articulated models with SVD for random restarts, then fit once with articulations at end
    # if false, fit articulated models with NL optimization during random restarts

    ############################
    # Call the code     #
    ############################
    print("Python:")
    angles = [0.5]
    R_temp = np.array([[0.10880512,, -0.99003329, 0.08941775],[-0.08941775, -0.09933467, -0.9910283], [0.99003329, 0.09983342, -0.09933467]])
    trans_temp = np.array([0.05, 0.03, 0.02])
    scale = 1.0
    start = time.time()
    aligned_source = getPointsWithAngles(model1,angles,R_temp,trans_temp, scale)
    print("time (micro):",(time.time()-start)*10**6)

    
    print(np.shape(aligned_source))
    print(aligned_source)

    ############# C++ Version Using Ctypes ################
    print("\n\n\nCPP:")
    start = time.perf_counter()
    cppreglibpath = os.path.join(os.getcwd(),"build","libcppregistration_test.so")
    cppreglib = ctypes.CDLL(cppreglibpath)
    cppreglib.getFitsForModels.restype = ctypes.POINTER(FITHOLDER)
    
    fitinputs = packageFittingInfo(np.asarray(scene.points),ref_point,models,artic_svd_initial)
    cppreglib.getFitsForModels(ctypes.byref(fitinputs))
    
if __name__ == '__main__':
    main()

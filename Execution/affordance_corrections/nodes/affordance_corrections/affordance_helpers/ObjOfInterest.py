#!/usr/bin/env python3
# File name: ObjOfInterest.py
# Description: Class for a selected point including all fits of models
# Author: Mike Hagenow
# Date: 6/17/2021

import numpy as np
import os
import time
import rospkg
import multiprocessing as mp
from multiprocessing import Pool
import trimesh
from scipy.spatial import cKDTree as KDTree
from affordance_corrections.affordance_helpers.pcl_helpers import get_cloud_of_interest, get_cloud_of_interest_np
from affordance_corrections.affordance_helpers.registration import icp_random_restarts, icp, get_fits_for_models, FitInfo
import ctypes
from affordance_corrections.affordance_helpers.cython_helpers import packageFittingInfo, packageRefittingInfo, convert_fits, FITHOLDER


class ObjOfInterest:
    ''' keeps track of a user-selected point of interest -- includes fits for a variety of models
        also calls the registration functions'''
    def __init__(self, models, ref_point, samples_per_mesh=150):
        self.models = models # list of Model Objects
        self.ref_point = ref_point # pt clicked by the user
        self.active_id = 0 # which model is active
        self.refitting = True # whether or not model should refit after corrections
        self.sorted = [] # used for cycling based on likelihood
        self.fits = [] # objects with all fitting info
        self.samples_per_mesh = samples_per_mesh # number of points for each mesh fit


    def default_fits(self):
        """ for each model, create a default fit (e.g., interactive markers)"""
        for ii in range(0,len(self.models)):
            angles = []
            for jj in range(0,len(self.models[ii].joints)):
                angles.append(self.models[ii].joints[jj].min_angle)
            info_temp = FitInfo(self.ref_point,np.eye(3),angles,1.0,np.inf)
            self.fits.append(info_temp) 

        # default to model 0
        self.active_id = 0

        # get sorted likelihoods for model cycling
        self.sorted = np.arange(0,len(self.models))          
    
    def fit_obj(self, scene, artic_svd_initial,cppfitting):
        """ for each model, sample points and then fit"""
        startt = time.time()

        # fits each of the models in the library
        if cppfitting: # c++ fitting
            rospack = rospkg.RosPack()
            root_dir = rospack.get_path('affordance_corrections')
            cppreglibpath = os.path.join(root_dir, "nodes", "affordance_corrections", "affordance_helpers", "cppfitting", "build", "libcppregistration.so")
            cppreglib = ctypes.CDLL(cppreglibpath)
            cppreglib.getFitsForModels.restype = ctypes.POINTER(FITHOLDER)
    
            fitinputs = packageFittingInfo(np.asarray(scene.points),self.ref_point,self.models,artic_svd_initial)
            
            # Call to C++
            fitholder_ptr_unconverted = cppreglib.getFitsForModels(ctypes.byref(fitinputs))
            self.fits = convert_fits(fitholder_ptr_unconverted,cppreglib)

        else: # python fitting
            self.fits = get_fits_for_models(scene,self.ref_point,self.models,artic_svd_initial)

        print("Fitting Time: ",time.time()-startt)
        
        # Set active to lowest error
        min_err = np.inf
        min_errors_temp = []
        for ii in range(0,len(self.models)):
            weighted_residual = self.fits[ii].residual/(self.models[ii].num_pts)
            min_errors_temp.append(weighted_residual)
            # print("Model: ",self.models[ii].name,weighted_residual, self.models[ii].area)
            if weighted_residual<min_err:
                min_err = weighted_residual
                self.active_id = ii

        # get sorted likelihoods for model cycling
        self.sorted = np.argsort(np.array(min_errors_temp))

    def refit_obj(self,scene,cpp_fitting):
        ''' After the user finishes a correction, perform a more basic ICP to further refine the pose '''
        print("SF REFIT: ",self.refitting)
        if self.refitting:
            R_temp = self.fits[self.active_id].rot
            t_temp = self.fits[self.active_id].pos
            angles_temp = self.fits[self.active_id].angles
            scale_temp = self.fits[self.active_id].scale

            startt = time.time()

            if cpp_fitting:
                rospack = rospkg.RosPack()
                root_dir = rospack.get_path('affordance_corrections')
                cppreglibpath = os.path.join(root_dir, "nodes", "affordance_corrections", "affordance_helpers", "cppfitting", "build", "libcppregistration.so")
                cppreglib = ctypes.CDLL(cppreglibpath)
                cppreglib.getRefitsForModel.restype = ctypes.POINTER(FITHOLDER)
        
                refitinputs = packageRefittingInfo(np.asarray(scene.points),self.ref_point,self.models[self.active_id],t_temp,R_temp,angles_temp,scale_temp)
                
                # Call to C++
                fitholder_ptr_unconverted = cppreglib.getRefitsForModel(ctypes.byref(refitinputs))
                fits_temp = convert_fits(fitholder_ptr_unconverted,cppreglib)
                self.fits[self.active_id] = fits_temp[0] # only one fit in pos 0


            else: # Python
                cloud_trimmed = get_cloud_of_interest(scene,self.ref_point,self.models[self.active_id].diameter)
                R_new, t_new, angles_new, scale_new, error_new = icp(self.models[self.active_id],cloud_trimmed,KDTree(cloud_trimmed),100, R_initial=R_temp,
                t_initial=t_temp,angles_initial=angles_temp,scale_initial = scale_temp, diameter=self.models[self.active_id].diameter, artic_svd_initial=False)
                
                self.fits[self.active_id].rot = R_new
                self.fits[self.active_id].pos = t_new
                self.fits[self.active_id].angles = angles_new
                self.fits[self.active_id].scale = scale_new
                self.fits[self.active_id].residual = error_new

            print("Refitting Time: ",time.time()-startt)
        
    def toggleActive(self):
        ''' Using a level of indirection to choose the next model based on the initial fitting likelihoods'''
        curr_ind = np.where(self.sorted==self.active_id)[0][0]
        new_ind = curr_ind+1
        if new_ind>=len(self.sorted):
            new_ind = 0
        new_active = self.sorted[new_ind]
        self.active_id=new_active
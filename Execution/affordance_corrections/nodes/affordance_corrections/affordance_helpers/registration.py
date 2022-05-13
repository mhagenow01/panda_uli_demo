#!/usr/bin/env python3
# File name: registration.py
# Description: register a model using ICP
# Author: Mike Hagenow
# Date: 6/7/2021

import argparse
import numpy as np
from scipy.spatial import cKDTree as KDTree
from scipy.stats import special_ortho_group
from affordance_corrections.affordance_helpers.urdf_helpers import getPointsWithAngles
from affordance_corrections.affordance_helpers.pcl_helpers import get_cloud_of_interest, get_cloud_of_interest_np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as ScipyR
import time
import matplotlib.pyplot as plt

class FitInfo:
    ''' used to keep track of the details of a particular model's fit
        also used when pose, etc get corrections to store updates '''
    def __init__(self,pos,rot,angles,scale,residual):
        self.pos = pos
        self.rot = rot
        self.angles = angles
        self.scale = scale
        self.residual = residual

def get_fits_for_models(scene,ref_point,models,artic_svd_initial):
    """ Runs ICP random restarts for each of the provided models and returns a list of the fits for each model """
    fits = []
    for ii in range(0,len(models)):
        cloud_trimmed = get_cloud_of_interest(scene,ref_point,models[ii].diameter)
        R_final,t_final,angles_final,scale_final,min_error = icp_random_restarts(models[ii],cloud_trimmed,25,100,diameter=models[ii].diameter,ref_pt = ref_point, artic_svd_initial=artic_svd_initial)
        info_temp = FitInfo(t_final,R_final,angles_final,scale_final,min_error)
        fits.append(info_temp)
    return fits

def icp_random_restarts(model,target,num_restarts,n, diameter,ref_pt, artic_svd_initial):
    """Run ICP multiple times to avoid local minima (random rotations, translations, angles, scale)."""
    min_error = np.inf
    # Store target in KDTree for fast lookup of closest points
    tree_target = KDTree(target)
    
    # Default the return values in case nothing is found (e.g., clicked off point cloud)
    R_final = np.eye(3)
    t_final = ref_pt
    angles_final = [0 for joint in model.joints]

    for ii in range(0,num_restarts):
        # Get random start for orientation, translation, and all relevant ariculations
        R_init = special_ortho_group.rvs(3)
        # t_init = ((diameter/2.0)*np.random.random_sample()*special_ortho_group.rvs(3)[0,:]+ref_pt).reshape((3,))
        t_init = ((diameter/2.0)*np.random.normal(0.0, 0.33, 1)*special_ortho_group.rvs(3)[0,:]+ref_pt).reshape((3,))
        
        angles = []
        for ii in range(0,len(model.joints)): # model articulations
            angles.append(model.joints[ii].min_angle + (model.joints[ii].max_angle-model.joints[ii].min_angle)*np.random.rand())

        scale_init = 1.0
        if model.scaling_allowed:
            scale_init = model.min_scale + (model.max_scale-model.min_scale)*np.random.rand()

        R_temp,t_temp,angles_temp,scale_temp,error_temp = icp(model, target, tree_target, n, R_initial=R_init,t_initial=t_init, angles_initial = angles, scale_initial = scale_init, diameter = diameter, artic_svd_initial=artic_svd_initial)
        
        # Choose whether to normalize mesh size based on the approx size of the particular model
        error_temp = error_temp
        # error_temp = error_temp/model.volume # normalize based on mesh size

        if error_temp<min_error:
            R_final = R_temp
            t_final = t_temp
            angles_final = angles_temp
            scale_final = scale_temp
            min_error = error_temp

    # if skipping articulations on first, refit best fit
    if artic_svd_initial and len(model.joints)>0:
        R_final,t_final,angles_final,scale_final,min_error = icp(model, target, tree_target, n, R_initial=R_final,t_initial=t_final, angles_initial = angles_final, scale_initial = scale_final, diameter = diameter, artic_svd_initial=False)

    return R_final,t_final,angles_final,scale_final,min_error

def get_closest_alignment(aligned_source,target,target_kdtree,max_distance_threshold):
    ''' Function for aligning closest points between source and target to be used for SVD'''
    
    if len(target)==0: # no points
        return np.zeros((0,3)), np.zeros((0,3)), np.zeros((0,3)), []
    dists, indices = target_kdtree.query(aligned_source,k=1)
    closest_target = target[indices]

    # Use mask filter to get the final sets of points that should be compared
    close_enough_inds = dists < max_distance_threshold # remove outliers
    s_vals = aligned_source[close_enough_inds]
    t_vals = closest_target[close_enough_inds]

    return s_vals, t_vals, dists, close_enough_inds

def registration_residual(x,model,t_vals,close_enough_inds, weights, R_final, trans_final):
    ''' Calculates a residual for nonlinear fitting of R, t, angles, and scale if applicable '''
    
    # optimization variables are: rotation (4- quat), translation (3), angles (if app.), scale (if app.)
    R = ScipyR.from_quat(x[0:4]).as_matrix()
    t = x[4:7]
    angles_final = list(x[7:7+len(model.joints)])

    if model.scaling_allowed:
        scale = x[-1]
    else:
        scale = 1.0

    R_temp = R @ R_final
    trans_temp = R @ trans_final + t

    aligned_source = getPointsWithAngles(model,angles_final,R_temp,trans_temp, scale)

    s_vals = aligned_source[close_enough_inds] # using previous alignment (point indices)

    non_weighted_residual = np.linalg.norm(s_vals - t_vals,axis=1).flatten()
    return np.dot(weights,non_weighted_residual)/scale # normalize by scale of model to avoid scale heading towards zero

def quat_con(x):
    ''' quaternion must have magnitude 1 -- used in NL optimization '''
    return 1.0-np.linalg.norm(x[0:4])

def icp(model,target,target_kdtree,n, R_initial=np.eye(3),t_initial = np.zeros((3,)), angles_initial=[], scale_initial =1.0 , diameter = 1.0, artic_svd_initial = False, refitting = False):
    """Take two arrays of points in 3d space and computes the registration."""
    """ source refers to mesh/model to be fit, target refers to scene/point cloud"""
    """ source and target dimensions are ax3 and bx3 """
    """ Note: target should be a region of interest (~ centered wrt the selected point) """

    converged = False
    max_distance_threshold = diameter
    num_iters = 0

    R_final = R_initial
    trans_final = t_initial
    angles_final = angles_initial
    prev_error = np.inf
    scale_final = scale_initial

    # Get initial sampling with angles

    aligned_source = getPointsWithAngles(model,angles_final,R_final,trans_final,scale_final)

    

    while not converged and num_iters<n:
        if(num_iters==0):
            

            s_vals, t_vals, dists, close_enough_inds = get_closest_alignment(aligned_source,target,target_kdtree,max_distance_threshold)
            if refitting:
                print("PYTHON REFITTING")
                plt.hist(dists, density=True, cumulative=True, label='CDF', histtype='step', alpha=0.8, color='k')
                plt.show()
                print("New distance threshold: ")
                max_distance_threshold = float(input())
                s_vals, t_vals, dists, close_enough_inds = get_closest_alignment(aligned_source,target,target_kdtree,max_distance_threshold)
                aligned_source = s_vals
                max_distance_threshold = 999.9

            error = np.linalg.norm((R_initial @ s_vals.T).T+t_initial - t_vals)
            # print("Initial error: ",error)
            if len(s_vals)<3: # need 3 points
                # insufficient points for registration, return initial pose
                return R_final,trans_final,angles_final, scale_final, np.inf # return as failure (infinite error)

        

        # Weighting is based on the inverse of distances (with some normalization)
        weights = 1/(1+np.power(dists[close_enough_inds],10.0))
        weights_matrix = np.diag(weights)

        if (not model.requires_NL_fitting() or artic_svd_initial): # use SVD if not fitting articulations/scale
            # Following this formulation: https://igl.ethz.ch/projects/ARAP/svd_rot.pdf
            w_cent_s = np.divide(weights @ s_vals ,np.sum(weights))
            w_cent_t = np.divide(weights @ t_vals ,np.sum(weights))

            s_vals -= w_cent_s
            t_vals -= w_cent_t

            S = s_vals.T @ weights_matrix @ t_vals
            U, sigma, Vh = np.linalg.svd(S)
            V = np.transpose(Vh)
            R = V @ U.T
            trans = (w_cent_t - R @ w_cent_s)
            
            # print("WC: ",w_cent_t,w_cent_s)
            # print("trs: ",trans)

            R_final = R @ R_final
            trans_final = R @ trans_final + trans
        
        else: # articulations, need to use NL optimization
            cons = {'type':'eq', 'fun': quat_con}
            
            weights_new = weights

            # structure of minimization: R as quaternion, t as (3x), angles as list
            # initialize parameters
            x0 = np.zeros((7+len(angles_final)+int(model.scaling_allowed)))
            quat_temp = np.array([0.0, 0.0, 0.0, 1.0])
            x0[0:4]= quat_temp
            x0[4:7] = np.zeros((3,))
            x0[7:(7+len(angles_final))] = np.array(angles_final)
            
            if model.scaling_allowed:
                x0[-1] = scale_final

            # bound for pose (quaterions and translation)
            bounds = [(-1.0, 1.0),(-1.0, 1.0),(-1.0, 1.0),(-1.0, 1.0),(-model.diameter/2,model.diameter/2),(-model.diameter/2,model.diameter/2),(-model.diameter/2,model.diameter/2)]
    
            # Add bounds for all articulations
            for ii in range(0,len(angles_final)):
                bounds.append((model.joints[ii].min_angle,model.joints[ii].max_angle))

            # Add bounds for scaling if enabled
            if model.scaling_allowed:
                bounds.append((model.min_scale,model.max_scale))
            res = minimize(registration_residual, x0, constraints=cons, bounds=bounds,args=(model,t_vals,close_enough_inds, weights_new, R_final,trans_final), method='SLSQP', options={'disp': False,'maxiter': 50})

            #########
            # Capture output from minimization
            #########
            R = ScipyR.from_quat(res.x[0:4] ).as_matrix()
            trans = res.x[4:7]
            angles_final = list(res.x[7:7+len(angles_final)])
            # print("x0: ",x0," r:",res.x)

            if model.scaling_allowed:
                scale_final = res.x[-1]
            
            R_final = R @ R_final
            trans_final = R @ trans_final + trans
            
        # Transform source points
        aligned_source = getPointsWithAngles(model,angles_final,R_final,trans_final,scale_final)

        #########################################################################
        # Check if done - calculate residual based on next set of pt alignments #
        #########################################################################

        s_vals, t_vals, dists, close_enough_inds = get_closest_alignment(aligned_source,target,target_kdtree,max_distance_threshold)
        if len(s_vals)<3: # need 3 points
            print("Exiting early")
            # insufficient points for registration, return initial pose
            return R_final,trans_final,angles_final, np.inf # return as failure (infinite error)

        # Calculate residual
        error = np.linalg.norm((R @ s_vals.T).T+trans - t_vals)

        # Check if converged (changing by small amount -- empiricially determined)
        error_per_pt = 0.000000025
        if np.abs(prev_error-error)<(error_per_pt*len(s_vals)):
            converged = True
        prev_error = error
        num_iters+=1

        # print("end error: ",error)

    return R_final,trans_final, angles_final, scale_final, error

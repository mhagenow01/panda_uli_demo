#!/usr/bin/env python3
# File name: cython_helpers.py
# Description: Convert python to Ctypes datatypes for sending to C++
# Author: Mike Hagenow, Nicole Gathman
# Date: 12/21/2021

from affordance_corrections.affordance_helpers.registration import get_fits_for_models, FitInfo

# Packages for C++ fitting
import ctypes
import numpy as np


#struct for LINK and JOINT
class LINK(ctypes.Structure):
    _fields_ = [('t',(ctypes.POINTER(ctypes.c_double))), ('R',(ctypes.POINTER(ctypes.POINTER(ctypes.c_double)))),('num_mesh_points', ctypes.c_int)]
class JOINT(ctypes.Structure):
     _fields_ = [('axis',(ctypes.c_double * 3)), ('min_angle',ctypes.c_double), ('max_angle', ctypes.c_double), ('R',(ctypes.POINTER(ctypes.POINTER(ctypes.c_double)))), ('t',(ctypes.POINTER(ctypes.c_double)))]
class MODELINFO(ctypes.Structure):
        _fields_ = [('mesh_samples',ctypes.POINTER(ctypes.POINTER(ctypes.POINTER(ctypes.c_double)))),('links', ctypes.POINTER(LINK)), ('joints', ctypes.POINTER(JOINT)), ('num_links', ctypes.c_int), ('num_joints', ctypes.c_int),('diameter', ctypes.c_double)]
class FITINFO(ctypes.Structure):
    _fields_ = [('pos',(ctypes.c_double * 3)),('rot',(ctypes.c_double * 3) * 3),('num_angles',ctypes.c_int),('angles', ctypes.POINTER(ctypes.c_double)),('scale', ctypes.c_double),('residual', ctypes.c_double)]

class FITHOLDER(ctypes.Structure):
    _fields_ = [('num_fits',ctypes.c_int),('fits',ctypes.POINTER(FITINFO))]

class FITTINGINPUTS(ctypes.Structure):
        _fields_ = [('num_scene_pts',ctypes.c_int),('scene',ctypes.POINTER(ctypes.POINTER(ctypes.c_double))),('ref_point',(ctypes.c_double * 3)),('num_models',ctypes.c_int),('models', ctypes.POINTER(ctypes.c_char_p)),('model_info', ctypes.POINTER(ctypes.POINTER(MODELINFO))),('artic_svd_initial', ctypes.c_bool)]

class REFITTINGINPUTS(ctypes.Structure):
        _fields_ = [('num_scene_pts',ctypes.c_int),('scene',ctypes.POINTER(ctypes.POINTER(ctypes.c_double))),('ref_point',(ctypes.c_double * 3)),('model', ctypes.c_char_p),('model_info', ctypes.POINTER(MODELINFO)),('initial_fit',FITINFO)]

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

    # Free the C++ memory
    cpp_reg_lib.freeFits(fitholder_ptr)
    return fits


def packageRefittingInfo(scene,ref_point,model,t_init,R_init,angles_init,scale_init):
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


    # Model Files and Meshes
    # each model object has one input file stored in models array
    #MODEL_OBJ is an array of modelinfo structs
    MODEL_FILE = bytes(model.name, 'utf-8')
    
    # Load the mesh (array of 2d arrays)
    model_obj = MODELINFO()
    model_obj.num_links = len(model.links)
    model_obj.num_joints = len(model.joints)
    model_obj.links = (model_obj.num_links*LINK)()
    for u in range (0 ,len(model.links)):
        R = (3*(ctypes.POINTER(ctypes.c_double)))()
        for v in range(0,3):
            R[v] = (ctypes.c_double * 3)()
            R[v][0] = model.links[u].R[v,0]
            R[v][1] = model.links[u].R[v,1]
            R[v][2] = model.links[u].R[v,2]
        (model_obj.links)[u].R = R
        (model_obj.links)[u].t = (ctypes.c_double * 3)()
        (model_obj.links)[u].t[0] = model.links[u].t[0]
        (model_obj.links)[u].t[1] = model.links[u].t[1]
        (model_obj.links)[u].t[2] = model.links[u].t[2]
        (model_obj.links)[u].num_mesh_points = len(model.links[u].pts)

    model_obj.joints = (model_obj.num_joints*JOINT)()
    for y in range (0, len(model.joints)):
        R = (3*(ctypes.POINTER(ctypes.c_double)))()
        for v in range(0,3):
            R[v] = (ctypes.c_double * 3)()
            R[v][0] = model.joints[y].R[v,0]
            R[v][1] = model.joints[y].R[v,1]
            R[v][2] = model.joints[y].R[v,2]
        (model_obj.joints)[y].R = R
        (model_obj.joints)[y].t = (ctypes.c_double * 3)()
        (model_obj.joints)[y].t[0]  = model.joints[y].t[0]
        (model_obj.joints)[y].t[1]  = model.joints[y].t[1]
        (model_obj.joints)[y].t[2]  = model.joints[y].t[2]
        (model_obj.joints)[y].axis[0] = model.joints[y].axis[0]
        (model_obj.joints)[y].axis[1] = model.joints[y].axis[1]
        (model_obj.joints)[y].axis[2] = model.joints[y].axis[2]
        (model_obj.joints)[y].min_angle = model.joints[y].min_angle
        (model_obj.joints)[y].max_angle = model.joints[y].max_angle
            
    
    model_obj.mesh_samples = (model_obj.num_links*ctypes.POINTER(ctypes.POINTER(ctypes.c_double)))()
        
    for jj in range(0,model_obj.num_links):
        num_pts = np.shape(model.links[jj].pts)[0]
        model_obj.mesh_samples[jj] = (num_pts*ctypes.POINTER(ctypes.c_double))()
        for kk in range(0,num_pts):
            model_obj.mesh_samples[jj][kk] = (3*ctypes.c_double)()
            model_obj.mesh_samples[jj][kk][0] = model.links[jj].pts[kk,0]
            model_obj.mesh_samples[jj][kk][1] = model.links[jj].pts[kk,1]
            model_obj.mesh_samples[jj][kk][2] = model.links[jj].pts[kk,2]

    model_obj.num_mesh_points = num_pts; 
    model_obj.diameter = model.diameter
    MODEL_OBJ = ctypes.POINTER(MODELINFO)(model_obj) 


    # Load initial fit (pos, rot, num_angles, angles)
    INIT_FIT = FITINFO()
    
    INIT_FIT.pos[0] = t_init[0]
    INIT_FIT.pos[1] = t_init[1]
    INIT_FIT.pos[2] = t_init[2]

    for v in range(0,3):
        # R_i[v] = (ctypes.c_double * 3)()
        INIT_FIT.rot[v][0] = R_init[v,0]
        INIT_FIT.rot[v][1] = R_init[v,1]
        INIT_FIT.rot[v][2] = R_init[v,2]

    INIT_FIT.num_angles = len(angles_init)

    angles_i = (ctypes.c_double * len(angles_init))()
    for v in range(0,len(angles_init)):
        angles_i[v] = angles_init[v]
    INIT_FIT.angles = angles_i

    INIT_FIT.scale = scale_init

    # Load all inputs into container
    refitinputs = REFITTINGINPUTS()
    refitinputs.num_scene_pts = num_scene_pts
    refitinputs.scene = SCENE_PTS
    refitinputs.ref_point = (ctypes.c_double*3)(*ref_point)
    refitinputs.model = MODEL_FILE
    refitinputs.model_info = MODEL_OBJ
    refitinputs.initial_fit = INIT_FIT
    return refitinputs


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
            (model_obj.links)[u].num_mesh_points = len(models[ii].links[u].pts)

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

        model_obj.diameter = models[ii].diameter
        MODEL_OBJS[ii] = ctypes.POINTER(MODELINFO)(model_obj) 


    # Load all inputs into container
    fitinputs = FITTINGINPUTS()
    fitinputs.num_scene_pts = num_scene_pts
    fitinputs.scene = SCENE_PTS
    fitinputs.ref_point = (ctypes.c_double*3)(*ref_point)
    fitinputs.num_models = len(models)
    fitinputs.models = MODEL_FILES
    fitinputs.model_info = MODEL_OBJS
    fitinputs.artic_svd_initial = artic_svd_initial   
    return fitinputs
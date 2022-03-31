#!/usr/bin/env python3
# File name: affordance_engine.py
# Description: Interface for fitting objects/primitives
# This interface provides the abstraction and data structures and
# interacts with a "visual" and "input" interface
# Author: Mike Hagenow
# Date: 6/17/2021

import argparse
import time
import open3d as o3d
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation as ScipyR
from affordance_corrections.affordance_helpers.ObjOfInterest import ObjOfInterest
from affordance_corrections.affordance_helpers.Model import Model
from affordance_corrections.affordance_helpers.urdf_helpers import getPointsWithAngles
from affordance_corrections.affordance_helpers.pcl_helpers import pt_near_cloud

class AffordanceEngine:
    """ Keeps track of objects of interest, fits, etc."""
    def __init__(self):
        self.objectsOfInterest = []
        self.models = []
        self.active_obj = 0
        self.scene = None
        self.artic_svd_initial = False # whether to fit the initial articulation models closed form (performance)
        self.fitting = True
        self.cppfitting = False
        
    
    def setScene(self,file, scene_rot = np.array([0.0, 0.0, 0.0, 1.0]),scene_trans = np.zeros((3,))):
        ''' Set the pcd point cloud and voxelize for performance -- can also take quat xyzw and trans for scene'''
        self.scene = o3d.io.read_point_cloud(file)
        self.scene = self.scene.voxel_down_sample(voxel_size=0.005) #0.005" -> .127 mm
        R_scene = ScipyR.from_quat(scene_rot)
        nppoints = np.asarray(self.scene.points)
        nppoints = R_scene.apply(nppoints)
        nppoints = nppoints + scene_trans
        self.scene.points = o3d.utility.Vector3dVector(nppoints)
        

    def setSceneXYZRGB(self,xyz,rgb):
        ''' Set the pcd point cloud and voxelize for performance'''
        self.scene = o3d.geometry.PointCloud()
        self.scene.points = o3d.utility.Vector3dVector(xyz)
        self.scene.colors = o3d.utility.Vector3dVector(rgb)
        self.scene = self.scene.voxel_down_sample(voxel_size=0.005) #0.005" -> .127 mm

    def ptNearScene(self,pt,dist):
        ''' check if a candidate point is actually near the scene point cloud '''
        return pt_near_cloud(self.scene,pt,dist)

    def deleteActivePoint(self):
        if(len(self.objectsOfInterest)>0):
            self.objectsOfInterest.pop(self.active_obj)
            self.active_obj -= 1

    def nearbyPointOfInterest(self,pt,camera_pos):
        ''' Checks whether a candidate point is near existing selected points
            based on the mesh diameter '''
        closest_pt = -1 # negative value in case nothing is found
        closest_dist = np.inf
        for ii in range(0,len(self.objectsOfInterest)):
            diameter_temp = self.objectsOfInterest[ii].models[self.objectsOfInterest[ii].active_id].diameter
            
            # get model and clicked point location
            model_loc_temp = self.objectsOfInterest[ii].fits[self.objectsOfInterest[ii].active_id].pos
            ref_pt_temp = self.objectsOfInterest[ii].ref_point

            scenenp = np.asarray(self.scene)

            # get distances from model and point
            model_loc_dist = np.linalg.norm(np.cross(pt-camera_pos, camera_pos-model_loc_temp))/np.linalg.norm(pt-camera_pos)
            ref_pt_dist = np.linalg.norm(np.cross(pt-camera_pos, camera_pos-ref_pt_temp))/np.linalg.norm(pt-camera_pos)

            print("model dist:",model_loc_dist, " ref dist: ",ref_pt_dist)

            if model_loc_dist<diameter_temp/2 or ref_pt_dist<diameter_temp/2: # closer than radius
                if np.minimum(model_loc_dist,ref_pt_dist)<closest_dist:
                    closest_pt = ii
                    closest_dist = np.minimum(model_loc_dist,ref_pt_dist)
        return closest_pt # negative value if nothing was found

    def setModels(self,model_files):
        ''' List of files (stl or urdf) for the model objects that should be fit '''
        for file in model_files:
            self.models.append(Model(file))

    def toggleSVDforInitialArticulation(self,toggle):
        ''' Toggles whether the initial fit of objects with articulations uses
            NL optimization (Toggle = False) or uses SVD (no articulation in fitting)
            and then does NL optimization as a post-processing step (Toggle = TRUE) '''
        self.artic_svd_initial = toggle

    def setFitting(self,fitting):
        ''' Toggles whether to do fitting at all or just allow the user to specify/augment pose'''
        self.fitting = fitting

    def setCppFitting(self,cppfitting):
        ''' Toggles whether the cpp fitting is used instead of python
            requires that the library has been built -- see repo readme '''
        self.cppfitting = cppfitting

    def setActiveObject(self,obj_id):
        ''' Used to set which object the corrections are currently being applied to'''
        self.active_obj = obj_id

    def setActiveObjectRefitting(self,refitting):
        ''' set flag for a particular object of whether refitting is enabled'''
        if len(self.objectsOfInterest)>0:
            self.objectsOfInterest[self.active_obj].refitting = refitting

    def getActiveObjectRefitting(self):
        ''' get flag for a particular object of whether refitting is enabled'''
        if len(self.objectsOfInterest)>0:
            return self.objectsOfInterest[self.active_obj].refitting
        else:
            return True # default is enabled
        
    def getActiveObjectPose(self):
        if len(self.objectsOfInterest) > 0:
            active_id = self.objectsOfInterest[self.active_obj].active_id
            return self.objectsOfInterest[self.active_obj].fits[active_id].rot, self.objectsOfInterest[self.active_obj].fits[active_id].pos
        # if nothing, return identity
        return np.array([0, 0, 0, 1]), np.zeros((3,))

    def addObjOfInterest(self,pt):
        ''' Create a new object and fit the model for a given point in space'''
        obj = ObjOfInterest(self.models,pt)
        if self.fitting:
            obj.fit_obj(self.scene,self.artic_svd_initial,self.cppfitting)
        else:
            obj.default_fits()
        self.active_obj = len(self.objectsOfInterest) # set new object as active
        self.objectsOfInterest.append(obj)

    def update_object_pose(self,pos,rot,obj_id):
        ''' If the pose is changed externally, update the engine'''
        active_id = self.objectsOfInterest[obj_id].active_id
        self.objectsOfInterest[obj_id].fits[active_id].rot = rot
        self.objectsOfInterest[obj_id].fits[active_id].pos = pos

    def twist_active_object(self,twist_array):
        ''' Applies a twist to the active object'''
        R_temp = ScipyR.from_euler('xyz',twist_array[3:6], degrees=True).as_matrix()
        if(len(self.objectsOfInterest)>0):
            active_id = self.objectsOfInterest[self.active_obj].active_id
            R_old = self.objectsOfInterest[self.active_obj].fits[active_id].rot
            self.objectsOfInterest[self.active_obj].fits[active_id].rot = R_temp @ R_old
            #need to compensate for any translation induced by the rotation
            centroid_shift = self.centroid_shift(R_old,R_temp @ R_old,self.objectsOfInterest[self.active_obj])
            self.objectsOfInterest[self.active_obj].fits[active_id].pos += (twist_array[0:3]-centroid_shift)

    def cycle_active_articulation(self):
        ''' Used to choose what articulation/scaling the user can provide corrections to (in addition to pose)
            Note: this is an additional 1DOF beyond the 6DOF required for the pose'''
        if(len(self.objectsOfInterest)>0):
            active_id = self.objectsOfInterest[self.active_obj].active_id
            max_artics = len(self.objectsOfInterest[self.active_obj].models[active_id].joints)
            
            # if enabled, the last articulation is scaling
            if self.objectsOfInterest[self.active_obj].models[active_id].scaling_allowed:
                max_artics+=1

            new_artic = self.objectsOfInterest[self.active_obj].models[active_id].artic_id + 1
            
            if new_artic>=max_artics: # if end of list, wrap back to number 0
                new_artic = 0
            self.objectsOfInterest[self.active_obj].models[active_id].artic_id = new_artic

    def articulate_active_object(self,angle):
        ''' Rotates one of the axes of the active object '''
        if(len(self.objectsOfInterest)>0):
            active_id = self.objectsOfInterest[self.active_obj].active_id
            artic_id = self.objectsOfInterest[self.active_obj].models[active_id].artic_id
            if artic_id<len(self.objectsOfInterest[self.active_obj].fits[active_id].angles):
                # angle articulation
                curr_angle = self.objectsOfInterest[self.active_obj].fits[active_id].angles[artic_id]
                desired_angle = curr_angle+angle
                if(desired_angle>self.objectsOfInterest[self.active_obj].models[active_id].joints[artic_id].max_angle):
                    desired_angle = self.objectsOfInterest[self.active_obj].models[active_id].joints[artic_id].max_angle
                if(desired_angle<self.objectsOfInterest[self.active_obj].models[active_id].joints[artic_id].min_angle):
                    desired_angle = self.objectsOfInterest[self.active_obj].models[active_id].joints[artic_id].min_angle
                self.objectsOfInterest[self.active_obj].fits[active_id].angles[artic_id]=desired_angle
            
            elif(artic_id==len(self.objectsOfInterest[self.active_obj].fits[active_id].angles) and self.objectsOfInterest[self.active_obj].models[active_id].scaling_allowed):
                # scale change
                curr_scale = self.objectsOfInterest[self.active_obj].fits[active_id].scale
                desired_scale = curr_scale + angle

                if(desired_scale>self.objectsOfInterest[self.active_obj].models[active_id].max_scale):
                    desired_scale = self.objectsOfInterest[self.active_obj].models[active_id].max_scale
                if(desired_scale<self.objectsOfInterest[self.active_obj].models[active_id].min_scale):
                    desired_scale = self.objectsOfInterest[self.active_obj].models[active_id].min_scale
                self.objectsOfInterest[self.active_obj].fits[active_id].scale=desired_scale

    def angle_active_object(self,angle):
        ''' Sets the angle of the active object (only works on angles -- not scale) '''
        if(len(self.objectsOfInterest)>0):
            active_id = self.objectsOfInterest[self.active_obj].active_id
            artic_id = self.objectsOfInterest[self.active_obj].models[active_id].artic_id

            # Make sure angle is within range
            if angle<0:
                angle=0.0
            if angle>1.0:
                angle = 1.0

            if artic_id<len(self.objectsOfInterest[self.active_obj].fits[active_id].angles):
                # angle articulation (set to passed in angle -- percentage)
                min_angle = self.objectsOfInterest[self.active_obj].models[active_id].joints[artic_id].min_angle
                max_angle = self.objectsOfInterest[self.active_obj].models[active_id].joints[artic_id].max_angle
                desired_angle = angle*(max_angle-min_angle)+min_angle
                if(desired_angle>self.objectsOfInterest[self.active_obj].models[active_id].joints[artic_id].max_angle):
                    desired_angle = self.objectsOfInterest[self.active_obj].models[active_id].joints[artic_id].max_angle
                if(desired_angle<self.objectsOfInterest[self.active_obj].models[active_id].joints[artic_id].min_angle):
                    desired_angle = self.objectsOfInterest[self.active_obj].models[active_id].joints[artic_id].min_angle
                self.objectsOfInterest[self.active_obj].fits[active_id].angles[artic_id]=desired_angle

    def flip_active_object(self):
        ''' Flips the object arround one of the top principal axes'''
        if(len(self.objectsOfInterest)>0):
            active_id = self.objectsOfInterest[self.active_obj].active_id
            R_old = self.objectsOfInterest[self.active_obj].fits[active_id].rot

            R_flip = self.objectsOfInterest[self.active_obj].models[active_id].getFlip()

            self.objectsOfInterest[self.active_obj].fits[active_id].rot = R_old @ R_flip
            #compensate for any translation induced by the rotation
            centroid_shift = self.centroid_shift(R_old,R_old @ R_flip,self.objectsOfInterest[self.active_obj])
            self.objectsOfInterest[self.active_obj].fits[active_id].pos -= centroid_shift
        

    def centroid_shift(self,R_old, R_new,obj):
        ''' Determine shift of centroid induced by rotation for compensation (rotating around centroid) '''
        fit = obj.fits[obj.active_id]
        # get points with no rotation/translation applied
        pts = getPointsWithAngles(obj.models[obj.active_id],fit.angles,np.eye(3),np.zeros((3,)))
        old_pts = ScipyR.from_matrix(R_old).apply(pts)
        new_pts = ScipyR.from_matrix(R_new).apply(pts)
        old_centroid = np.mean(old_pts,axis=0)
        new_centroid = np.mean(new_pts,axis=0)
        return new_centroid-old_centroid

    def refit_active_object(self):
        ''' Once corrections are applied, this routine refits using the slimmed ICP'''
        if(len(self.objectsOfInterest)>0 and self.fitting):
            self.objectsOfInterest[self.active_obj].refit_obj(self.scene, self.cppfitting)
    
    def refit_all_objects(self):
        ''' Fits all objects in scene, for example, when scene is updated '''
        for ii in range(0,len(self.objectsOfInterest) and self.fitting):
            self.objectsOfInterest[self.active_obj].refit_obj(self.scene, self.cppfitting)

    def getUpdatedWorld(self):
        ''' Creates a dictionary with all properties of the scene, including the fit objects'''
        worldState = dict()
        if self.scene is not None:
            worldState['scene'] = self.scene
        worldState['objectsOfInterest'] = self.objectsOfInterest
        worldState['active_obj']=self.active_obj
        return worldState


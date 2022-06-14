#!/usr/bin/env python3
# File name: Model.py
# Description: Class which keeps track of all model information (e.g., file, pts, articulations, etc.)
# Author: Mike Hagenow
# Date: 6/29/2021

import re
import time
import scipy.spatial as sspatial
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation as ScipyR
from affordance_corrections.affordance_helpers.urdf_helpers import getLinksAndJoints, getLinksAndJointsFromSTL, getPointsWithAngles

class Model:
    ''' A model contains information such as meshes, links, joints, and transforms '''
    
    def __init__(self,inputfile, num_pts=150, pkg_dir=None):
        self.name = inputfile.split('/')[-1].split(".")[0]
        self.filename = inputfile

        filetype = inputfile.split(".")[-1]

        ''' Currently loads either a single STL or a URDF model'''
        if filetype=="STL" or filetype=="stl":
            self.links, self.joints = getLinksAndJointsFromSTL(inputfile, pkg_dir)
        else:
            self.links, self.joints = getLinksAndJoints(inputfile, pkg_dir)
        
        self.calculate_diameter()
        self.scaling_allowed = False

        self.num_pts = 0
        for ii in range(0,len(self.links)):
            self.num_pts += len(self.links[ii].pts)

        ''' prinicipal axes for flipping -- not used currently'''
        self.principal_axes = []
        self.calculate_principal_axes()
        self.flip_number = 0
        self.artic_id = 0 # which articulation (e.g., joints, scaling) is active


    def calculate_diameter(self):
        ''' determine the diameter to use for approximate calculations of size, distance, etc.'''
        # concatenate all the links together to calculate the diameter
        angles = [0 for joint in self.joints]
        points = getPointsWithAngles(self,angles,np.eye(3),np.zeros((3,)))

        # Use convex hull to get the model diameter and volume
        start = time.perf_counter()
        hull = sspatial.ConvexHull(points)
        new_points = points[hull.vertices]
        diameter_new_hull = 0.0
        for point in new_points:
            for point2 in new_points:
                if np.linalg.norm(point-point2)>diameter_new_hull:
                    diameter_new_hull = np.linalg.norm(point-point2)

        self.diameter = diameter_new_hull
        self.volume = hull.volume
        self.area = hull.area

    def allowMeshScaling(self,min_scale,max_scale):
        ''' if set, a model can also update its scale during NL fitting (e.g., different size valves)'''
        self.scaling_allowed = True
        self.min_scale = min_scale
        self.max_scale = max_scale

    def requires_NL_fitting(self):
        ''' Need NL fitting if articulations or scaling allowed'''
        return len(self.joints)>0 or self.scaling_allowed

    ##############################################################
    # Prinicpal component flipping (below) is being deprecated   #
    # interesting idea, but didn't work well in practice         #
    ##############################################################

    def calculate_principal_axes(self):
        ''' use SVD to get the principal axes to use for quick flipping '''
        angles = [0 for joint in self.joints]
        points = getPointsWithAngles(self,angles,np.eye(3),np.zeros((3,)))

        u,s,vh = np.linalg.svd(points)
        for ii in range(0,2):
            self.principal_axes.append(vh[ii,:])

    def getFlip(self):
        ''' returns a rotation by a principal axis '''
        if self.flip_number<4:
            axis = self.principal_axes[0]
        else:
            axis = self.principal_axes[1]

        # Move to next behavior
        max_flip_number = 7
        if self.flip_number==max_flip_number:
            self.flip_number=0
        else:
            self.flip_number+=1

        # Rotation is 180 degrees wrt the given axis
        return ScipyR.from_rotvec(np.pi/2*np.array(axis)).as_matrix()
    
    def resetFlip(self):
        ''' for flipping around principal axes'''
        self.flip_number = 0

    

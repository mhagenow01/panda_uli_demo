#!/usr/bin/env python3
# File name: urdf_helpers.py
# Description: URDF to class objects
# Author: Mike Hagenow
# Date: 7/07/2021

import numpy as np
import trimesh
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation as ScipyR
import rospkg

class Joint:
    ''' used in model'''
    def __init__(self,axis,min_angle,max_angle,R,t):
        # store and sample meshes
        self.axis = axis
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.R = R
        self.t = t

class Link:
    ''' used in model'''
    def __init__(self,meshfile,R,t, num_pts = 400):
        self.meshfile = meshfile
        self.R = R
        self.t = t

        self.mesh = trimesh.load_mesh(meshfile)
        self.pts = trimesh.sample.sample_surface_even(self.mesh,num_pts,radius=None)[0]

def getLinksAndJoints(urdffile):
    ''' From the URDF, load link and joint objects for fitting, plotting, etc.
    Note: an object without articulations would be expected to have one link'''
    
    xmltree = ET.parse(urdffile)
    root = xmltree.getroot()
    
    joints = []
    links = []

    # Determine number of links a priori to determine number of points per link
    # (same total number of points)
    num_links = 0
    for child in root:
        if child.tag == "link":
            num_links += 1
    if num_links == 0:
        num_links = 1  
    
    # The following is mainly traversing the XML structure
    # to pull out key information needed to define the full mesh
    for child in root:
        # TODO: only supports revolute currently
        if child.tag=="joint" and child.attrib['type']=='revolute':
            for attr in child:
                if attr.tag == "limit":
                    min_angle = float(attr.attrib['lower'])
                    max_angle = float(attr.attrib['upper'])
                if attr.tag == "axis":
                    axis = np.fromstring(attr.attrib['xyz'],dtype=float, sep=' ')
                    axis = axis/np.linalg.norm(axis) # normalize if necessary
                if attr.tag == "origin":
                    rpy = np.fromstring(attr.attrib['rpy'],dtype=float, sep=' ')
                    R = ScipyR.from_euler('zyx',rpy,degrees=False).as_matrix()
                    t = np.fromstring(attr.attrib['xyz'],dtype=float, sep=' ')
            joint_temp = Joint(axis,min_angle,max_angle,R,t)
            joints.append(joint_temp)
        if child.tag=="link":
            visual = child[0]
            for attr in visual:
                if attr.tag== "origin":
                    rpy = np.fromstring(attr.attrib['rpy'],dtype=float, sep=' ')
                    R = ScipyR.from_euler('zyx',rpy,degrees=False).as_matrix()
                    t = np.fromstring(attr.attrib['xyz'],dtype=float, sep=' ')
                if attr.tag == "geometry":
                    for geo_attr in attr:
                        if geo_attr.tag == "mesh":
                            mesh_file = geo_attr.attrib['filename']
                            # resolve full filename
                            pkg_temp = mesh_file.split("//")[1].split("/")[0]

                            rospack = rospkg.RosPack()
                            path_within_package = ""
                            for ii in range(1,len(mesh_file.split("//")[1].split("/"))):
                                path_within_package+=("/"+mesh_file.split("//")[1].split("/")[ii])
                            mesh_file_full = rospack.get_path(pkg_temp)+path_within_package
            link_temp = Link(mesh_file_full,R,t, num_pts =int(400.0/num_links))
            links.append(link_temp)

    return links, joints

def getLinksAndJointsFromSTL(stlfile):
    ''' An STL file can be interpreted as a single link'''
    joints = []
    links = []

    link_temp = Link(stlfile,np.eye(3),np.zeros((3,)))
    links.append(link_temp)

    return links, joints

def getPointsWithAngles(model,angles,R_base,t_base,scale=1):
    ''' gets the full point sampled model for a model with given joint angles'''
    pts_arr = np.zeros((0,3))

    for ii in range(0,len(model.links)):
        pts_temp = np.copy(model.links[ii].pts)
        # scale part
        centroid_before = np.mean(pts_temp,axis=0)
        pts_temp*=scale
        centroid_after = np.mean(pts_temp,axis = 0)
        pts_temp-=(centroid_after-centroid_before)

        for jj in range(ii,-1,-1): # count from ii down to 0

            # link offset (needs to be subtracted)
            pts_temp = (model.links[jj].R @ pts_temp.T).T - scale*model.links[jj].t
            
            # joint transform
            if (jj<len(model.joints) and ii!=0): # need to ignore final link (has no joint)
                # Transformation of joint itself
                
                pts_temp = (model.joints[jj].R @ pts_temp.T).T + scale*model.joints[jj].t

                # Rotation from the angle (with respect to the model's origin)
                R_angle = ScipyR.from_rotvec(angles[jj]*model.joints[jj].axis).as_matrix()
                # centroid_before = np.mean(pts_temp,axis=0)
                origin_before = -scale*model.links[jj].t
                origin_after = (R_angle @ origin_before.T).T
                pts_temp = (R_angle @ pts_temp.T).T
                pts_temp -= (origin_after-origin_before)
                
        # Rotation of entire structure (i.e., initial)
        pts_temp = (R_base @ pts_temp.T).T + t_base
        pts_arr = np.concatenate((pts_arr,pts_temp),axis=0)
    
    return pts_arr


#!/usr/bin/env python3
# File name: pcl_helpers.py
# Description: Misc utilites for point clouds
# Author: Mike Hagenow
# Date: 6/17/2021

import numpy as np
from scipy.spatial import KDTree as KDTree


def pt_near_cloud(pcd,pt,dist):
    ''' Check if a selected point is near a point cloud '''
    cloud_np = np.asarray(pcd.points)
    if len(get_cloud_of_interest_np(cloud_np,pt,dist))>0:
        return True
    return False

def get_cloud_of_interest(pcd,pt,radius):
    """ Get NP array from PCD and trim"""
    cloud_np = np.asarray(pcd.points)
    return get_cloud_of_interest_np(cloud_np,pt,radius)

def get_cloud_of_interest_np(cloud_np,pt,radius):
    """ Given a point of interest, find all points in the cloud within a given radius"""
    cloud_kdtree = KDTree(cloud_np)
    closeby_inds = cloud_kdtree.query_ball_point(pt,radius)
    return cloud_np[closeby_inds]
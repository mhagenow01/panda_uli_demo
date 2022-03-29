#!/usr/bin/env python3
# File name: pc_conversions.py
# Description: Convert pcd to pointcloud2 msg for ros publisher
# Author: Mike Hagenow
# Date: 6/16/2021

import argparse
import time
import copy
import open3d as o3d
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation as ScipyR

def downsample_pcd(pcd_file,voxelsize):
    ''' voxelize pcd to make the file a reasonable size'''
    scene = o3d.io.read_point_cloud(pcd_file)
    scene = scene.voxel_down_sample(voxel_size=voxelsize)
    o3d.io.write_point_cloud(pcd_file,scene)

def rotate_pcd(pcd_file):
    ''' rotate pcd so that it works well with rviz controls'''
    scene = o3d.io.read_point_cloud(pcd_file)
    
    # Apply any transformation required
    T = np.eye(4)
    T[:3, :3]=scene.get_rotation_matrix_from_xyz((-np.pi/180.0*30,0,0))
    T[0,3]=0
    T[1,3]=0
    T[2,3]=0
    scene = scene.transform(T)
    
    # # save once happy with rotation
    o3d.io.write_point_cloud(pcd_file,scene)


def cut_scene(pcd_file):
    ''' cut out irrelevant part of scene (one axis at a time)'''
    scene = o3d.io.read_point_cloud(pcd_file)
    pts = np.asarray(scene.points)
    colors = np.asarray(scene.colors)
    print(np.shape(pts))
    pts_culled = np.zeros((0,3))
    colors_culled = np.zeros((0,3))
    for ii in range(0,len(pts)):
        if pts[ii,2]>-0.1:
            pts_culled = np.append(pts_culled,pts[ii].reshape((1,3)),axis=0)
            colors_culled = np.append(colors_culled,colors[ii].reshape((1,3)),axis=0)
    print(np.shape(pts_culled))
    scene.points = o3d.utility.Vector3dVector(pts_culled)
    scene.colors = o3d.utility.Vector3dVector(colors_culled)
    o3d.visualization.draw_geometries([scene])
    o3d.io.write_point_cloud(pcd_file,scene)


def center_stl(stl_file):
    ''' moves stl to be centered around centroid (sometimes something else random from solidworks/blender)'''
    mesh_trimesh = trimesh.load_mesh(stl_file)
    pts = trimesh.sample.volume_mesh(mesh_trimesh,3000)
    center = np.mean(pts,axis=0)
    mesh = o3d.io.read_triangle_mesh(stl_file)
    mesh.translate((-center[0], -center[1], -center[2]))
    mesh.compute_vertex_normals()
    print("Center (before): ",center)
    o3d.io.write_triangle_mesh(stl_file,mesh)


if __name__ == '__main__':
    # args = parse_arguments()
    # downsample_pcd("../../../../../pcd/taskboard3.pcd",0.005)
    # rotate_pcd("../../../../../pcd/taskboard3.pcd")
    # cut_scene("../../../../../pcd/taskboard3.pcd")
    # center_stl("../../../../../src/affordance_models/meshes/ball_valve_handle.STL")

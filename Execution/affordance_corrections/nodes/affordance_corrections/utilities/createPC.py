#!/usr/bin/env python3
# File name: createPC.py
# Description: Place objects and create pcd files for testing
# Author: Mike Hagenow
# Date: 6/15/2021

import argparse
import time
import open3d as o3d
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation as ScipyR

def sampleMesh(mesh,n):
    """Take STL and return numpy array of points."""
    return trimesh.sample.sample_surface_even(mesh,n,radius=None)[0]


def testminimalscene():
    ''' A few points for ros msg testing '''
    pts = np.array([[0.0, 0.0, 0.0],
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0]])
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd.colors = o3d.utility.Vector3dVector(pts)
    
    return pcd, pts

def scene1():
    ''' Scene 1 is one ball valve and one E-stop on top of a table'''
    valve_file = '../models/ball_valve_one_inch.STL'
    estop_file = '../models/e_stop_coarse.STL'
    table_file = '../models/table.stl'

    points = np.zeros((0,3))
    colors = np.zeros((0,3))

    # Place the table
    table = trimesh.load_mesh(table_file)
    table_pts = sampleMesh(table,8000)

    points = np.concatenate((points,table_pts),axis=0)
    table_colors = np.tile(np.array([0.541, 0.321, 0.058]),[len(table_pts), 1])
    colors = np.concatenate((colors,table_colors),axis=0)

    # Place the valve on the table
    valve = trimesh.load_mesh(valve_file)
    valve_pts = sampleMesh(valve,1000)

    R_valve = ScipyR.from_euler('xyz',[0, 105, 0], degrees=True).as_matrix()
    t_valve = np.array([-0.45, 0.0, 0.03])
    valve_pts = (R_valve @ valve_pts.T).T + t_valve # (rotated and translated)

    points = np.concatenate((points,valve_pts),axis=0)
    valve_colors = np.tile(np.array([0.2, 0.2, 0.8]),[len(valve_pts), 1])
    colors = np.concatenate((colors,valve_colors),axis=0)

    # Place the estop on the table
    estop = trimesh.load_mesh(estop_file)
    estop_pts = sampleMesh(estop,400)

    R_estop = ScipyR.from_euler('xyz',[0, 0, 0], degrees=True).as_matrix()
    t_estop = np.array([-0.1, -0.1, 0.0])
    estop_pts = (R_estop @ estop_pts.T).T + t_estop # (rotated and translated)
    
    points = np.concatenate((points,estop_pts),axis=0)
    estop_colors = np.tile(np.array([0.8, 0.1, 0.2]),[len(estop_pts), 1])
    colors = np.concatenate((colors,estop_colors),axis=0)

    # Create the final point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    return pcd, points

def scene2():
    ''' Scene 1 is one ball valve, one E-stop, and one handle on top of a table'''
    valve_file = '../../../../../models/ball_valve_one_inch.STL'
    estop_file = '../../../../../models/e_stop_coarse.STL'
    handle_file = '../../../../../models/handle.STL'
    table_file = '../../../../../models/table.stl'

    points = np.zeros((0,3))
    colors = np.zeros((0,3))

    # Place the table
    table = trimesh.load_mesh(table_file)
    table_pts = sampleMesh(table,8000)

    points = np.concatenate((points,table_pts),axis=0)
    table_colors = np.tile(np.array([0.541, 0.321, 0.058]),[len(table_pts), 1])
    colors = np.concatenate((colors,table_colors),axis=0)

    # Place the valve on the table
    valve = trimesh.load_mesh(valve_file)
    valve_pts = sampleMesh(valve,1000)

    R_valve = ScipyR.from_euler('xyz',[0, 105, 0], degrees=True).as_matrix()
    t_valve = np.array([-0.20, 0.0, 0.03])
    valve_pts = (R_valve @ valve_pts.T).T + t_valve # (rotated and translated)

    points = np.concatenate((points,valve_pts),axis=0)
    valve_colors = np.tile(np.array([0.2, 0.2, 0.8]),[len(valve_pts), 1])
    colors = np.concatenate((colors,valve_colors),axis=0)

    # Place the estop on the table
    estop = trimesh.load_mesh(estop_file)
    estop_pts = sampleMesh(estop,400)

    R_estop = ScipyR.from_euler('xyz',[0, 0, 0], degrees=True).as_matrix()
    t_estop = np.array([-0.55, -0.1, 0.0])
    estop_pts = (R_estop @ estop_pts.T).T + t_estop # (rotated and translated)
    
    points = np.concatenate((points,estop_pts),axis=0)
    estop_colors = np.tile(np.array([0.8, 0.1, 0.2]),[len(estop_pts), 1])
    colors = np.concatenate((colors,estop_colors),axis=0)

    # Place the handle on the table
    handle = trimesh.load_mesh(handle_file)
    handle_pts = sampleMesh(handle,400)

    R_handle = ScipyR.from_euler('xyz',[0, 0, 0], degrees=True).as_matrix()
    t_handle = np.array([-0.37, 0.1, 0.06])
    handle_pts = (R_handle @ handle_pts.T).T + t_handle # (rotated and translated)
    
    points = np.concatenate((points,handle_pts),axis=0)
    handle_colors = np.tile(np.array([1.0, 0.647, 0.0]),[len(handle_pts), 1])
    colors = np.concatenate((colors,handle_colors),axis=0)

    # Create the final point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    return pcd, points


def scene3():
    ''' Scene 3 is all five meshes on a table'''
    valve_file = '../../../../../models/ball_valve_one_inch.STL'
    estop_file = '../../../../../models/e_stop_coarse.STL'
    handle_file = '../../../../../models/handle.STL'
    stopvalve_file = '../../../../../models/stop_valve_assem.STL'
    pvcvalve_file = '../../../../../src/affordance_models/meshes/pvc_ball_valve_assem.STL'
    table_file = '../../../../../models/table.stl'

    points = np.zeros((0,3))
    colors = np.zeros((0,3))

    # Place the table
    table = trimesh.load_mesh(table_file)
    table_pts = sampleMesh(table,8000)

    points = np.concatenate((points,table_pts),axis=0)
    table_colors = np.tile(np.array([0.541, 0.321, 0.058]),[len(table_pts), 1])
    colors = np.concatenate((colors,table_colors),axis=0)

    # Place the valve on the table
    valve = trimesh.load_mesh(valve_file)
    valve_pts = sampleMesh(valve,1000)

    R_valve = ScipyR.from_euler('xyz',[0, 105, 0], degrees=True).as_matrix()
    t_valve = np.array([-0.20, 0.0, 0.03])
    valve_pts = (R_valve @ valve_pts.T).T + t_valve # (rotated and translated)

    points = np.concatenate((points,valve_pts),axis=0)
    valve_colors = np.tile(np.array([0.2, 0.2, 0.8]),[len(valve_pts), 1])
    colors = np.concatenate((colors,valve_colors),axis=0)

    # Place the estop on the table
    estop = trimesh.load_mesh(estop_file)
    estop_pts = sampleMesh(estop,400)

    R_estop = ScipyR.from_euler('xyz',[0, 0, 0], degrees=True).as_matrix()
    t_estop = np.array([-0.55, -0.1, 0.06])
    estop_pts = (R_estop @ estop_pts.T).T + t_estop # (rotated and translated)
    
    points = np.concatenate((points,estop_pts),axis=0)
    estop_colors = np.tile(np.array([0.8, 0.1, 0.2]),[len(estop_pts), 1])
    colors = np.concatenate((colors,estop_colors),axis=0)

    # Place the handle on the table
    handle = trimesh.load_mesh(handle_file)
    handle_pts = sampleMesh(handle,400)

    R_handle = ScipyR.from_euler('xyz',[0, 0, 0], degrees=True).as_matrix()
    t_handle = np.array([-0.37, 0.1, 0.015])
    handle_pts = (R_handle @ handle_pts.T).T + t_handle # (rotated and translated)
    
    points = np.concatenate((points,handle_pts),axis=0)
    handle_colors = np.tile(np.array([1.0, 0.647, 0.0]),[len(handle_pts), 1])
    colors = np.concatenate((colors,handle_colors),axis=0)


    # Place the stop valve on the table
    stopvalve = trimesh.load_mesh(stopvalve_file)
    stopvalve_pts = sampleMesh(stopvalve,400)

    R_stopvalve = ScipyR.from_euler('xyz',[0, 0, 0], degrees=True).as_matrix()
    t_stopvalve = np.array([0.05, 0.0, 0.04])
    stopvalve_pts = (R_stopvalve @ stopvalve_pts.T).T + t_stopvalve # (rotated and translated)
    
    points = np.concatenate((points,stopvalve_pts),axis=0)
    stopvalve_colors = np.tile(np.array([0.7, 0.2, 0.4]),[len(stopvalve_pts), 1])
    colors = np.concatenate((colors,stopvalve_colors),axis=0)

    # Place the pvc valve on the table
    pvc = trimesh.load_mesh(pvcvalve_file)
    pvc_pts = sampleMesh(pvc,400)

    R_pvc = ScipyR.from_euler('xyz',[90, -90, 0], degrees=True).as_matrix()
    t_pvc = np.array([0.0, 0.18, 0.00])
    pvc_pts = (R_pvc @ pvc_pts.T).T + t_pvc # (rotated and translated)
    
    points = np.concatenate((points,pvc_pts),axis=0)
    pvc_colors = np.tile(np.array([0.4, 0.7, 0.3]),[len(pvc_pts), 1])
    colors = np.concatenate((colors,pvc_colors),axis=0)

    # Create the final point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    
    return pcd, points


def mounted_valve(): # degrees
    ''' Scene 1 is one ball valve, one E-stop, and one handle on top of a table'''
    valve_base_file = '../../../../affordance_models/meshes/ball_valve_base.STL'
    valve_top_file = '../../../../affordance_models/meshes/ball_valve_handle.STL'
    table_file = '../../../../affordance_models/meshes/teaser_robot.stl'
    pipe_file = '../../../../affordance_models/meshes/1_in_pipe_straight.STL'
    wall_file = '../../../../affordance_models/meshes/wall.stl'

    points = np.zeros((0,3))
    colors = np.zeros((0,3))

    # Place the table
    table = trimesh.load_mesh(table_file)
    table_pts = sampleMesh(table,4000)

    # points = np.concatenate((points,table_pts),axis=0)
    # table_colors = np.tile(np.array([0.541, 0.321, 0.058]),[len(table_pts), 1])
    # colors = np.concatenate((colors,table_colors),axis=0)


    # Place the valve base
    valve_base = trimesh.load_mesh(valve_base_file)
    valve_base_pts = sampleMesh(valve_base,100)

    R_valve_base = ScipyR.from_euler('xyz',[-90, 0, 0], degrees=True).as_matrix()
    t_valve_base = np.array([0.0, 0.0475, 0.0])
    valve_base_pts = (R_valve_base @ valve_base_pts.T).T + t_valve_base # (rotated and translated)

    points = np.concatenate((points,valve_base_pts),axis=0)
    table_colors = np.tile(np.array([0.8, 0.8, 0.8]),[len(valve_base_pts), 1])
    colors = np.concatenate((colors,table_colors),axis=0)

    # Place the valve top
    valve_top = trimesh.load_mesh(valve_top_file)
    valve_top_pts = sampleMesh(valve_top,50)

    R_valve_top = ScipyR.from_euler('xyz',[-90, 45, 0], degrees=True).as_matrix()
    orig_before  = np.array([0, 0.0, 0.0])
    orig_after =  (ScipyR.from_euler('xyz',[0, 45, 0], degrees=True)).apply(orig_before)
    t_valve_top = np.array([0, 0.0475, 0])-(orig_after-orig_before)
    valve_top_pts = (R_valve_top @ valve_top_pts.T).T + t_valve_top # (rotated and translated)

    points = np.concatenate((points,valve_top_pts),axis=0)
    table_colors = np.tile(np.array([0.0, 0.2, 0.8]),[len(valve_top_pts), 1])
    colors = np.concatenate((colors,table_colors),axis=0)

    # Pipe 1
    pipe1 = trimesh.load_mesh(pipe_file)
    pipe1_pts = sampleMesh(pipe1,100)

    R_pipe1 = ScipyR.from_euler('xyz',[0, 90, 0], degrees=True).as_matrix()
    t_pipe1 = np.array([-0.005, -0.005, -0.033])
    pipe1_pts = (R_pipe1 @ pipe1_pts.T).T + t_pipe1 # (rotated and translated)

    points = np.concatenate((points,pipe1_pts),axis=0)
    pipe_colors = np.tile(np.array([0.220, 0.095, 0.010]),[len(pipe1_pts), 1])
    colors = np.concatenate((colors,pipe_colors),axis=0)

    # Pipe 2
    pipe2 = trimesh.load_mesh(pipe_file)
    pipe2_pts = sampleMesh(pipe1,100)

    R_pipe2 = ScipyR.from_euler('xyz',[0, 90, 0], degrees=True).as_matrix()
    t_pipe2 = np.array([-0.005, -0.005, 0.490])
    pipe2_pts = (R_pipe2 @ pipe2_pts.T).T + t_pipe2 # (rotated and translated)

    points = np.concatenate((points,pipe2_pts),axis=0)
    pipe_colors = np.tile(np.array([0.220, 0.095, 0.010]),[len(pipe2_pts), 1])
    colors = np.concatenate((colors,pipe_colors),axis=0)

    # Wall
    wall = trimesh.load_mesh(wall_file)
    wall_pts = sampleMesh(wall,40000)

    R_wall = ScipyR.from_euler('xyz',[0, 90, 90], degrees=True).as_matrix()
    t_wall = np.array([0.0, -0.08, 0.0])
    wall_pts = (R_wall @ wall_pts.T).T + t_wall # (rotated and translated)

    points = np.concatenate((points,wall_pts),axis=0)
    wall_colors = np.tile(np.array([0.941, 0.820, 0.545]),[len(wall_pts), 1])
    colors = np.concatenate((colors,wall_colors),axis=0)

    # Create the final point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    return pcd, points


def scene4():
    ''' Scene 4 is all five meshes on a table -- two have joint angles'''
    valve_file = '../../../../../models/ball_valve_angled.STL'
    estop_file = '../../../../../models/e_stop_coarse.STL'
    handle_file = '../../../../../models/handle.STL'
    stopvalve_file = '../../../../../models/stop_valve_assem.STL'
    pvcvalve_file = '../../../../../models/pvc_ball_valve_angled.STL'
    table_file = '../../../../../models/table.stl'

    points = np.zeros((0,3))
    colors = np.zeros((0,3))

    # Place the table
    table = trimesh.load_mesh(table_file)
    table_pts = sampleMesh(table,8000)

    points = np.concatenate((points,table_pts),axis=0)
    table_colors = np.tile(np.array([0.541, 0.321, 0.058]),[len(table_pts), 1])
    colors = np.concatenate((colors,table_colors),axis=0)

    # Place the valve on the table
    valve = trimesh.load_mesh(valve_file)
    valve_pts = sampleMesh(valve,1000)

    R_valve = ScipyR.from_euler('xyz',[0, 105, 0], degrees=True).as_matrix()
    t_valve = np.array([-0.55, -0.1, 0.10])
    valve_pts = (R_valve @ valve_pts.T).T + t_valve # (rotated and translated)

    points = np.concatenate((points,valve_pts),axis=0)
    valve_colors = np.tile(np.array([0.2, 0.2, 0.8]),[len(valve_pts), 1])
    colors = np.concatenate((colors,valve_colors),axis=0)

    # Place the estop on the table
    estop = trimesh.load_mesh(estop_file)
    estop_pts = sampleMesh(estop,400)

    
    R_estop = ScipyR.from_euler('xyz',[0, 0, 0], degrees=True).as_matrix()
    t_estop = np.array([-0.20, 0.0, 0.06])
    estop_pts = (R_estop @ estop_pts.T).T + t_estop # (rotated and translated)
    
    points = np.concatenate((points,estop_pts),axis=0)
    estop_colors = np.tile(np.array([0.8, 0.1, 0.2]),[len(estop_pts), 1])
    colors = np.concatenate((colors,estop_colors),axis=0)

    # Place the handle on the table
    handle = trimesh.load_mesh(handle_file)
    handle_pts = sampleMesh(handle,400)

    R_handle = ScipyR.from_euler('xyz',[0, 0, 0], degrees=True).as_matrix()
    t_handle = np.array([-0.37, 0.13, 0.015])
    handle_pts = (R_handle @ handle_pts.T).T + t_handle # (rotated and translated)
    
    points = np.concatenate((points,handle_pts),axis=0)
    handle_colors = np.tile(np.array([1.0, 0.647, 0.0]),[len(handle_pts), 1])
    colors = np.concatenate((colors,handle_colors),axis=0)


    # Place the stop valve on the table
    stopvalve = trimesh.load_mesh(stopvalve_file)
    stopvalve_pts = sampleMesh(stopvalve,400)

    R_stopvalve = ScipyR.from_euler('xyz',[0, 0, 0], degrees=True).as_matrix()
    t_stopvalve = np.array([0.05, 0.0, 0.04])
    stopvalve_pts = (R_stopvalve @ stopvalve_pts.T).T + t_stopvalve # (rotated and translated)
    
    points = np.concatenate((points,stopvalve_pts),axis=0)
    stopvalve_colors = np.tile(np.array([0.7, 0.2, 0.4]),[len(stopvalve_pts), 1])
    colors = np.concatenate((colors,stopvalve_colors),axis=0)

    # Place the pvc valve on the table
    pvc = trimesh.load_mesh(pvcvalve_file)
    pvc_pts = sampleMesh(pvc,400)

    R_pvc = ScipyR.from_euler('xyz',[90, -90, 0], degrees=True).as_matrix()
    t_pvc = np.array([0.0, 0.18, 0.00])
    pvc_pts = (R_pvc @ pvc_pts.T).T + t_pvc # (rotated and translated)
    
    points = np.concatenate((points,pvc_pts),axis=0)
    pvc_colors = np.tile(np.array([0.4, 0.7, 0.3]),[len(pvc_pts), 1])
    colors = np.concatenate((colors,pvc_colors),axis=0)

    # Create the final point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    
    return pcd, points


def valveOnlyScene(angle=0): # degrees
    ''' Scene 1 is one ball valve, one E-stop, and one handle on top of a table'''
    valve_base_file = '../../../../affordance_models/meshes/pvc_ball_valve_base.STL'
    valve_top_file = '../../../../affordance_models/meshes/pvc_ball_valve_valve.STL'

    points = np.zeros((0,3))
    colors = np.zeros((0,3))

    # Place the valve base
    valve_base = trimesh.load_mesh(valve_base_file)
    valve_base_pts = sampleMesh(valve_base,3000)

    R_valve_base = ScipyR.from_euler('xyz',[0, 0, 0], degrees=True).as_matrix()
    t_valve_base = np.array([0.0, 0.0, 0.0])
    valve_base_pts = (R_valve_base @ valve_base_pts.T).T + t_valve_base # (rotated and translated)

    points = np.concatenate((points,valve_base_pts),axis=0)
    table_colors = np.tile(np.array([0.8, 0.8, 0.8]),[len(valve_base_pts), 1])
    colors = np.concatenate((colors,table_colors),axis=0)

    # Place the valve top
    valve_top = trimesh.load_mesh(valve_top_file)
    valve_top_pts = sampleMesh(valve_top,2000)

    R_valve_top = ScipyR.from_euler('xyz',[0, 0, 0], degrees=True).as_matrix()
    t_valve_top = np.array([0.0, 0.0, 0.034736])
    valve_top_pts = (R_valve_top @ valve_top_pts.T).T + t_valve_top # (rotated and translated)

    points = np.concatenate((points,valve_top_pts),axis=0)
    table_colors = np.tile(np.array([0.0, 0.2, 0.8]),[len(valve_top_pts), 1])
    colors = np.concatenate((colors,table_colors),axis=0)

    # Create the final point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    return pcd, points

def ballvalveOnlyScene(angle=0): # degrees
    ''' Scene 1 is one ball valve, one E-stop, and one handle on top of a table'''
    valve_base_file = '../../../../affordance_models/meshes/ball_valve_base.STL'
    valve_top_file = '../../../../affordance_models/meshes/ball_valve_handle.STL'

    points = np.zeros((0,3))
    colors = np.zeros((0,3))

    # Place the valve base
    valve_base = trimesh.load_mesh(valve_base_file)
    valve_base_pts = sampleMesh(valve_base,3000)

    R_valve_base = ScipyR.from_euler('xyz',[0, 0, 0], degrees=True).as_matrix()
    t_valve_base = np.array([0.0, 0.0, 0.0])
    valve_base_pts = (R_valve_base @ valve_base_pts.T).T + t_valve_base # (rotated and translated)

    points = np.concatenate((points,valve_base_pts),axis=0)
    table_colors = np.tile(np.array([0.8, 0.8, 0.8]),[len(valve_base_pts), 1])
    colors = np.concatenate((colors,table_colors),axis=0)

    # Place the valve top
    valve_top = trimesh.load_mesh(valve_top_file)
    valve_top_pts = sampleMesh(valve_top,2000)

    R_valve_top = ScipyR.from_euler('xyz',[0, 0, 90], degrees=True).as_matrix()
    t_valve_top = np.array([0.0, 0.0, -0.009])
    valve_top_pts = (R_valve_top @ valve_top_pts.T).T + t_valve_top # (rotated and translated)

    points = np.concatenate((points,valve_top_pts),axis=0)
    table_colors = np.tile(np.array([0.0, 0.2, 0.8]),[len(valve_top_pts), 1])
    colors = np.concatenate((colors,table_colors),axis=0)

    # Create the final point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    return pcd, points

def valveOnlyScaledScene(angle=0): # degrees
    ''' Scene 1 is one ball valve, one E-stop, and one handle on top of a table'''
    valve_base_file = '../../../../affordance_models/meshes/pvc_ball_valve_base.STL'
    valve_top_file = '../../../../affordance_models/meshes/pvc_ball_valve_valve.STL'

    points = np.zeros((0,3))
    colors = np.zeros((0,3))

    scale = 1.35

    # Place the valve base
    valve_base = trimesh.load_mesh(valve_base_file)
    valve_base_pts = sampleMesh(valve_base,3000)

    R_valve_base = ScipyR.from_euler('xyz',[0, 0, 0], degrees=True).as_matrix()
    t_valve_base = np.array([0.0, 0.0, 0.0])
    valve_base_pts = (R_valve_base @ valve_base_pts.T).T + t_valve_base # (rotated and translated)

    points = np.concatenate((points,valve_base_pts),axis=0)
    table_colors = np.tile(np.array([0.8, 0.8, 0.8]),[len(valve_base_pts), 1])
    colors = np.concatenate((colors,table_colors),axis=0)

    # Place the valve top
    valve_top = trimesh.load_mesh(valve_top_file)
    valve_top_pts = sampleMesh(valve_top,2000)

    R_valve_top = ScipyR.from_euler('xyz',[0, 0, 45], degrees=True).as_matrix()
    t_valve_top = np.array([0.0, 0.0, 0.034736])
    valve_top_pts = (R_valve_top @ valve_top_pts.T).T + t_valve_top # (rotated and translated)

    points = np.concatenate((points,valve_top_pts),axis=0)
    table_colors = np.tile(np.array([0.0, 0.2, 0.8]),[len(valve_top_pts), 1])
    colors = np.concatenate((colors,table_colors),axis=0)

    points = scale*points

    # Create the final point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    return pcd, points


def main():
    ''' Choose the file to be saved'''
    # pcd, points = testminimalscene()
    # pcd, points = scene1()
    # pcd, points = scene2()
    # o3d.visualization.draw_geometries([pcd])
    # pcd, points = valveOnlyScene()
    pcd, points = mounted_valve()
    # pcd, points = ballvalveOnlyScene()

    o3d.visualization.draw_geometries([pcd])
    o3d.io.write_point_cloud("teaser_robot_scene_only.pcd",pcd)

if __name__ == '__main__':
    # args = parse_arguments()
    main()
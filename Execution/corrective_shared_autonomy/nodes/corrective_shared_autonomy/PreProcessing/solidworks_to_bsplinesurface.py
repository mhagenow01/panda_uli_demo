#!/usr/bin/env python

""" Convert STL of desired surface to a BSplineSurface
 using the BSplinesurface calls from the other library
 Created: 07/10/2020
"""

__author__ = "Mike Hagenow"

import trimesh
import trimesh.sample as sample
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import minimize, Bounds
from scipy.spatial import ConvexHull
from core_robotics.PyBSpline import BSplineSurface
from corrective_shared_autonomy.PreProcessing.rigid_registration_panda import calculateRR
import time

def exp_map_np(w):
    w = np.array(w)
    w = w.reshape(3,)
    theta = (w[0] ** 2 + w[1] ** 2 + w[2] ** 2) ** 0.5 + 1e-30
    w = w / theta
    w_hat = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    return np.eye(3) + w_hat * np.sin(theta) + np.dot(w_hat, w_hat) * (1 - np.cos(theta))

# x are the parameters, stored as wx,wy,d
# y are the points in the point cloud
def obj_plane_dist(x, pts):
    sum = 0.0
    for pt in pts:
        # equation for the plane - sum of residual is used as the LSQ error
        vec_to_plane_center = np.transpose(np.matmul(exp_map_np([x[0], x[1], 0.0]),np.array([0.0, 0.0, x[2]]).reshape((3,1)))-pt.reshape((3,1)))
        sum+=np.abs(np.matmul(vec_to_plane_center,np.matmul(exp_map_np([x[0], x[1], 0.0]),np.array([0.0, 0.0, 1.0]).reshape((3,1))))[0][0])
    # print x[0], x[1], x[2], sum
    return sum

def findPlane(pts):
    best_opt = np.inf
    for ii in range(0,5):
        wx = np.random.rand()*np.pi/2
        wy = np.random.rand()*np.pi/2
        d = 1.0*(np.random.rand()-0.5)

        x0 = np.array([wx, wy, d])
        bounds = Bounds([0.0, 0.0, -np.inf], [np.pi, np.pi, np.inf])
        res = minimize(obj_plane_dist, x0, method='Nelder-Mead',
                       options={'disp': False}, bounds=None, args=(pts))

        if res.fun < best_opt:
            best_opt=res.fun
            wx_best = res.x[0]
            wy_best = res.x[1]
            d_best = res.x[2]

    return wx_best, wy_best, d_best

def surfaceFromSTL(surface_file,stl_file,rigid_file,breadboard_offset=[0,0]):
    mesh = trimesh.load(stl_file)

    # Turn STL into a point cloud of the desired surface using even sampling
    points, something = sample.sample_surface_even(mesh,200)

    # convert properly for mm to m issue (from solidworks models)
    points = points/1000

    A_mocap_panda, A_breadboard_mocap, A_breadboard_panda = calculateRR(rigid_file)
    # Apply a rotation and a translation to all of the points
    R = A_breadboard_panda[0:3,0:3]
    t = np.array(A_breadboard_panda[0:3,3]).reshape((3,))

    # t_addtl is the offset in the breadboard (measured in number of holes)
    t_addtl = np.matmul(R,np.array([0.025*breadboard_offset[0], 0.025*breadboard_offset[1], 0.0]).reshape((3,1))).reshape((3,))
    t = t+t_addtl

    for ii in range(0, len(points)):
        points[ii] = (np.matmul(R, points[ii].reshape((3, 1))) + t.reshape((3, 1))).reshape(3, )

    ###########################################################################################
    ##  Algorithm for fitting the B Spline Surface to the Point Cloud                        ##
    ##  partially adopted from https://www.inf.usi.ch/hormann/papers/Greiner.1997.IAA.pdf    ##
    ###########################################################################################

    # Note: may also need segmentation, but not in this case

    # Fit a plane to the points using NLSQ
    wx,wy,d = findPlane(points)
    print("OPT:", wx, wy, d)

    # Project all of the points onto the plane
    projected_pts = []
    plane_origin = np.matmul(exp_map_np([wx, wy, 0.0]),np.array([0.0, 0.0, d]))
    plane_normal = np.matmul(exp_map_np([wx, wy, 0.0]),np.array([0.0, 0.0, 1.0]))
    for point in points:
        v = point - plane_origin
        dist = np.dot(v,plane_normal)
        proj_point = point - dist*plane_normal

        # convert back to plane parameterized representation -> aka divide by exponential map?
        param_pt = np.matmul(np.transpose(exp_map_np([wx, wy, 0.0])),proj_point.reshape((3,)))
        u_temp = param_pt[0]
        v_temp = param_pt[1]
        projected_pts.append([u_temp, v_temp])
    projected_pts = np.array(projected_pts)


    #######################################################
    # Fit the best rectangle around the points            #
    #######################################################
    # use something like this... https://gis.stackexchange.com/questions/22895/finding-minimum-area-rectangle-for-given-points
    # key idea: orientation of minimized rectangle is the same as one of the edges of the point cloud convex hull
    # get the direction and the normal direction
    # using all points, find the min and max of these
    hull = ConvexHull(projected_pts)
    # for simplex in hull.simplices:
    #     print "simplex:",simplex
    #     plt.plot(projected_pts[simplex, 0], projected_pts[simplex, 1], 'k-', color='red')

    min_area_temp = np.inf

    # each simplex is a set of indices representing an edge in the convex hull
    for simplex_id in hull.simplices:
        plt.plot([projected_pts[simplex_id[0]][0],projected_pts[simplex_id[1]][0]], [projected_pts[simplex_id[0]][1],projected_pts[simplex_id[1]][1]], 'k-', color='red')

        min_edge = np.inf
        max_edge = -np.inf
        min_orth_edge = np.inf
        max_orth_edge = -np.inf

        # projection stuff for the rectangle
        # get vector from point 1 to point 2
        edge_vec = projected_pts[simplex_id[1]]-projected_pts[simplex_id[0]]
        edge_vec = edge_vec/np.linalg.norm(edge_vec)

        # 90 degree rotation CW
        orth_edge_vec = np.array([edge_vec[1], -edge_vec[0]])

        for vertex in hull.vertices:
            # get vector from point 1 to point n
            vec_to_n = projected_pts[vertex]-projected_pts[simplex_id[0]]

            # along axis_proj
            dist_temp_axis = np.dot(vec_to_n,edge_vec)
            if dist_temp_axis<min_edge:
                min_edge = dist_temp_axis
            if dist_temp_axis>max_edge:
                max_edge = dist_temp_axis

            # orthogonal axis projection
            dist_temp_orth = np.dot(vec_to_n, orth_edge_vec)
            if dist_temp_orth < min_orth_edge:
                min_orth_edge = dist_temp_orth
            if dist_temp_orth > max_orth_edge:
                max_orth_edge = dist_temp_orth

        # Calculate area
        area_temp = (max_edge-min_edge)*(max_orth_edge-min_orth_edge)

        if area_temp<min_area_temp:
            min_area_temp=area_temp
            pt_1 = projected_pts[simplex_id[0]]+min_edge*edge_vec+min_orth_edge*orth_edge_vec
            pt_2 = projected_pts[simplex_id[0]]+max_edge*edge_vec+min_orth_edge*orth_edge_vec
            pt_3 = projected_pts[simplex_id[0]] + max_edge * edge_vec + max_orth_edge * orth_edge_vec
            pt_4 = projected_pts[simplex_id[0]] + min_edge * edge_vec + max_orth_edge * orth_edge_vec
            max_edge_b = max_edge
            min_edge_b = min_edge
            max_orth_edge_b = max_orth_edge
            min_orth_edge_b = min_orth_edge
            edge_vec_b = edge_vec
            orth_edge_vec_b = orth_edge_vec

    fig1 = plt.figure(1)
    plt.scatter(projected_pts[:, 0], projected_pts[:, 1])

    # PLOT THE RECTANGLE!!!!
    plt.plot([pt_1[0], pt_2[0]],[pt_1[1], pt_2[1]], color='green')
    plt.plot([pt_2[0], pt_3[0]], [pt_2[1], pt_3[1]], color='green')
    plt.plot([pt_3[0], pt_4[0]], [pt_3[1], pt_4[1]], color='green')
    plt.plot([pt_4[0], pt_1[0]], [pt_4[1], pt_1[1]], color='green')

    plt.show(block=False)


    ##############################################################
    # Create a starting point for the control points             #
    ##############################################################
    # create evenly spaced candidates along this new grid

    # number of control points
    num_ctrl_pts = 20

    ctrl_pts = np.zeros((num_ctrl_pts,num_ctrl_pts,3))
    ctrl_pts_plotting = []
    for ii in range(0,num_ctrl_pts):
        for jj in range(0,num_ctrl_pts):
            pt_on_plane = pt_1+(max_edge_b-min_edge_b)*(ii*1.0/(num_ctrl_pts-1.0))*edge_vec_b+(max_orth_edge_b-min_orth_edge_b)*(jj*1.0/(num_ctrl_pts-1.0))*orth_edge_vec_b
            pt_3d = np.matmul(exp_map_np([wx,wy,0.0]),np.array([pt_on_plane[0], pt_on_plane[1], d]).reshape((3,1)))
            ctrl_pts[ii,jj,:]=pt_3d.reshape((3,))

    # Get the U and V vectors in 3D space to be used later on for input mapping
    u_dir_plane = np.matmul(exp_map_np([wx,wy,0.0]),np.array([edge_vec_b[0], edge_vec_b[1], 0]).reshape((3,1))).reshape((3,))
    v_dir_plane = np.matmul(exp_map_np([wx, wy, 0.0]), np.array([orth_edge_vec_b[0], orth_edge_vec_b[1], 0]).reshape((3,1))).reshape((3,))


    # conform back to the surface as control points (Do I need a spring - start w/o it!)
    points_rich, something = sample.sample_surface_even(mesh, 10000)
    points_rich = points_rich/1000 # mm to m issue

    for ii in range(0, len(points_rich)):
        points_rich[ii] = (np.matmul(R, points_rich[ii].reshape((3, 1))) + t.reshape((3, 1))).reshape(3, )

    for ii in range(0, num_ctrl_pts):
        for jj in range(0, num_ctrl_pts):
            best_match = np.inf
            temp_pt = ctrl_pts[ii,jj,:]
            for point in points_rich:
                if np.linalg.norm(point-temp_pt)<best_match:
                    best_match = np.linalg.norm(point-temp_pt)
                    best_pt = point
            ctrl_pts[ii, jj, :] = best_pt
            ctrl_pts_plotting.append(best_pt.reshape((3,)))
    ctrl_pts_plotting = np.array(ctrl_pts_plotting)

    fig2 = plt.figure(2)
    ax = fig2.gca(projection='3d')

    # Plot the plane!!!
    plane_pts = []
    plane_center = np.average(projected_pts,axis=0)
    for ii in np.linspace(-0.1, 0.1, 8):
        for jj in np.linspace(-0.1, 0.1, 8):
            plane_pts.append(np.matmul(exp_map_np([wx, wy, 0.0]),np.array([[ii+plane_center[0]], [jj+plane_center[1]], [d]])))
    plane_pts = np.array(plane_pts)

    ax.scatter(ctrl_pts_plotting[:,0], ctrl_pts_plotting[:,1], ctrl_pts_plotting[:,2], color='green')
    ax.scatter(plane_pts[:,0],plane_pts[:,1],plane_pts[:,2],color='orange')
    ax.scatter(points[:,0],points[:,1],points[:,2])

    # Check that the normals are facing the correct direction
    u_dir = ctrl_pts[5,0,:]-ctrl_pts[0,0,:]
    v_dir = ctrl_pts[0,5,:]-ctrl_pts[0,0,:]

    normal_dir = np.cross(u_dir,v_dir)
    normal_dir = normal_dir*10.0
    ax.quiver(ctrl_pts[0,0,0],ctrl_pts[0,0,1],ctrl_pts[0,0,2],normal_dir[0],normal_dir[1],normal_dir[2])


    ax.set_xlim3d(0.0, 0.4)
    ax.set_ylim3d(0.0, 0.4)
    ax.set_zlim3d(0.0, 0.4)
    print("----------------")
    print("point = {" + str(t[0]) + ", " + str(t[1]) + ", " + str(t[2]) + "}")
    print("color2 = {0.0, 0.0, 1.0}")
    print("pt=sim.addDrawingObject(dr,0.003,0.0,-1,30000,color2)")
    print("point[1]=point[1]+panda_frame[1]")
    print("point[2]=point[2]+panda_frame[2]")
    print("point[3]=point[3]+panda_frame[3]")
    print("sim.addDrawingObjectItem(pt,point)")


    for ii in range(0,num_ctrl_pts):
        for jj in range(0,num_ctrl_pts):
            print("point = {"+str(ctrl_pts[ii,jj,0])+", "+str(ctrl_pts[ii,jj,1])+", "+str(ctrl_pts[ii,jj,2])+"}")
            print("pt=sim.addDrawingObject(dr,0.001,0.0,-1,30000,color)")
            print("point[1]=point[1]+panda_frame[1]")
            print("point[2]=point[2]+panda_frame[2]")
            print("point[3]=point[3]+panda_frame[3]")
            print("sim.addDrawingObjectItem(pt,point)")

    plt.show(block=False)
    time.sleep(2)

    print(" ")
    print(" ")
    print(" ")
    flipNormals=input("Should the normal be flipped? (Y/N)\n")

    if flipNormals=="Y" or flipNormals=="y":
        # flip u and v in order to flip the normals
        ctrl_pts_flipped = np.zeros((num_ctrl_pts,num_ctrl_pts,3))
        for ii in range(0,num_ctrl_pts):
            for jj in range(0,num_ctrl_pts):
                ctrl_pts_flipped[jj][ii][:]=ctrl_pts[ii][jj][:]

        ctrl_pts = ctrl_pts_flipped
        temp_dir = u_dir_plane
        u_dir_plane = v_dir_plane
        v_dir_plane = temp_dir

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.scatter(ctrl_pts_plotting[:, 0], ctrl_pts_plotting[:, 1], ctrl_pts_plotting[:, 2], color='green')
    # Check that the normals are facing the correct direction
    u_dir = ctrl_pts[5, 0, :] - ctrl_pts[0, 0, :]
    v_dir = ctrl_pts[0, 5, :] - ctrl_pts[0, 0, :]

    normal_dir = np.cross(u_dir, v_dir)
    normal_dir = normal_dir * 10.0
    ax.quiver(ctrl_pts[0, 0, 0], ctrl_pts[0, 0, 1], ctrl_pts[0, 0, 2], normal_dir[0], normal_dir[1], normal_dir[2])

    plt.show()

    # Create a B-Spline instance
    curve = BSplineSurface()
    curve.initialize(k=3, control_pts=ctrl_pts, u_dir=u_dir_plane, v_dir=v_dir_plane)
    curve.writeSurface(surface_file)


if __name__ == "__main__":
    stl_file = '/home/mike/Desktop/cowling_4_surface.STL'
    rigid_file = '/home/mike/Documents/LearningCorrections/data/10-18-21/rigid1_10-18-21.bag'
    surface_file = '/home/mike/Documents/LearningCorrections/data/10-18-21/layup.csv'
    surfaceFromSTL(surface_file,stl_file,rigid_file,breadboard_offset=[0,0])





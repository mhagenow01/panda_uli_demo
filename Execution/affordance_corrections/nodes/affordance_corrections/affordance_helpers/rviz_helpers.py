#!/usr/bin/env python3
# File name: rviz_helpers.py
# Description: Converts representations to proper rviz topics
# Author: Mike Hagenow
# Date: 6/17/2021

import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation as ScipyR
import rospy
import copy
from sensor_msgs.msg import PointCloud2, PointField
import struct
from visualization_msgs.msg import Marker, MarkerArray

def pcd_to_pc2(pcd,active=True):
    ''' Takes pcd and massages into ROS Point Cloud 2 --
    just the points and colors'''
    num_pts = len(pcd.points)
    
    # Create message type
    cloud_msg = PointCloud2()
    
    # Header
    cloud_msg.header.seq = 0
    cloud_msg.header.stamp = rospy.Time.now()
    cloud_msg.header.frame_id = "map"
    
    # Set up the point fields (for now just x,y,z,color [rgb])
    names = ["x","y","z","rgb"]
    for ii in range(0,4):
        temp_pf = PointField()
        temp_pf.name = names[ii]
        temp_pf.offset = 4*ii # each field is 4 bytes
        temp_pf.datatype = 7 #float32
        temp_pf.count= 1
        cloud_msg.fields.append(temp_pf)
    
    # Cloud data
    cloud_msg.is_bigendian = False
    cloud_msg.height = 1 # 1D unordered data
    cloud_msg.width = len(pcd.points)
    cloud_msg.point_step = 16 # number of bytes for each value -- each is 4 bytes (x,y,z,rgb)
    
    data = []
    for ii in range(0,num_pts):
        # x,y,z
        for jj in range(0,3):
            temp = list(struct.pack("!f",float(pcd.points[ii][jj])))
            temp.reverse()
            data.extend(temp)
        
        # color (r,g,b)
        # open3d stores colors from o-1, convert to 0-255 (also it is reversed with one empty byte at the end)
        if active:
            temp = [int(255*pcd.colors[ii][2]), int(255*pcd.colors[ii][1]), int(255*pcd.colors[ii][0]), 0]
        else:
            temp = [150, 150, 150, 0]
        data.extend(temp)
    
    cloud_msg.data = data
    return cloud_msg


def mesh_to_marker(stl_file, index, pos, quat, color=[0.0, 1.0, 0.0], frame="map", scale=1.0): #xyzw
    """ Converts an STL into the marker representation """
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = rospy.Time.now()
    marker.ns = "meshes"
    marker.id = index
    marker.type = 10 # mesh type
    marker.action = 0 # Add/Modify
    marker.pose.position.x = pos[0]
    marker.pose.position.y = pos[1]
    marker.pose.position.z = pos[2]
    marker.pose.orientation.x = quat[0]
    marker.pose.orientation.y = quat[1]
    marker.pose.orientation.z = quat[2]
    marker.pose.orientation.w = quat[3]
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.mesh_resource = "file://"+stl_file
    return marker

def empty_marker(count,prev_count,meshes):
    ''' rviz has some issues with publishing less markers in an array than previously
        this function is used to publish invisible markers when the number of markers
        decreases (e.g., switching from a 2-mesh to 1-mesh model)'''
    for ii in range(count,prev_count):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "meshes"
        marker.id = ii
        marker.type = 1 # cube type (not showing anyways)
        marker.action = 0 # Add/Modify
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.color.a = 0.0
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0
        meshes.markers.append(marker)
        count+=1


def mesh_to_interactive_marker2(stl_file,pos, quat,color=[0.0, 1.0, 0.0]):
    """ Converts an STL into the marker representation for interactive marker """
    """ only requires a small subset of the normal marker fields! """
    marker = Marker()
    marker.type = 10 # mesh type
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1
    marker.color.a = 1.0
    marker.pose.position.x = pos[0]
    marker.pose.position.y = pos[1]
    marker.pose.position.z = pos[2]
    marker.pose.orientation.x = quat[0]
    marker.pose.orientation.y = quat[1]
    marker.pose.orientation.z = quat[2]
    marker.pose.orientation.w = quat[3]
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.mesh_resource = "file://"+stl_file
    return marker

def mesh_to_interactive_marker(stl_file,color=[0.0, 1.0, 0.0]):
    """ Converts an STL into the marker representation for interactive marker """
    """ only requires a small subset of the normal marker fields! """
    marker = Marker()
    marker.type = 10 # mesh type
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.mesh_resource = "file://"+stl_file
    return marker

def modeltoInteractiveMarker(model,fit,curr_iter,color=[0.0, 1.0, 0.0]):
    """ Converts a Model into the marker representation for interactive marker """
    """ similar to normal, but don't apply base rotation/translation! """
    markers = []
    for ii in range(0,len(model.links)):
        R = np.eye(3)
        t = np.zeros((3,))
        pts_temp = model.links[ii].pts
        scale = fit.scale

        ''' Need to calculate mesh transforms based on angles of rotation'''
        for jj in range(ii,-1,-1): # count from ii down to 0
            # link transform
            R = model.links[jj].R @ R
            t = R @ t - scale*model.links[jj].t
            pts_temp = (model.links[jj].R @ pts_temp.T).T - scale*model.links[jj].t

            # joint transform
            if (jj<len(model.joints) and ii!=0): # need to ignore final link (has no joint)c
                # Transformation of joint itself
                R = model.joints[jj].R @ R
                t = R @ t + scale*model.joints[jj].t
                pts_temp = (model.joints[jj].R @ pts_temp.T).T + scale*model.joints[jj].t
                
                # Rotation from the angle (with respect to the model's origin)
                R_angle = ScipyR.from_rotvec(fit.angles[jj]*model.joints[jj].axis).as_matrix()
                origin_before = scale*model.links[jj].t
                origin_after = (R_angle @ origin_before.T).T
                pts_temp = (R_angle @ pts_temp.T).T
                pts_temp -= (origin_after-origin_before)

                R = R_angle @ R
                t -= (origin_after-origin_before)
            
        

        # Don't apply Base Rotation/Translation for Interactive Markers
        R_base = np.eye(3)
        t_base = np.zeros((3,))
        pts_temp = (R_base @ pts_temp.T).T + t_base
        R = R_base @ R
        t = R @ t + t_base

        # mesh needs quaterion instead of rotation matrix
        quat = ScipyR.from_matrix(R).as_quat()
        mesh_stl = model.links[ii].meshfile
        
        marker_temp = mesh_to_interactive_marker2(mesh_stl,t,quat,color)
        markers.append(copy.deepcopy(marker_temp))
        curr_iter+=1
    return markers

def modelUpdateInteractiveMarker(model,fit,markers,curr_iter,color=[0.0, 1.0, 0.0]):
    """ Converts a Model into the marker representation for interactive marker """
    """ similar to normal, but don't apply base rotation/translation! """
    for ii in range(0,len(model.links)):
        R = np.eye(3)
        t = np.zeros((3,))
        pts_temp = model.links[ii].pts
        scale = fit.scale

        ''' Need to calculate mesh transforms based on angles of rotation'''
        for jj in range(ii,-1,-1): # count from ii down to 0
            # link transform
            R = model.links[jj].R @ R
            t = R @ t - scale*model.links[jj].t
            pts_temp = (model.links[jj].R @ pts_temp.T).T - scale*model.links[jj].t

            # joint transform
            if (jj<len(model.joints) and ii!=0): # need to ignore final link (has no joint)c
                # Transformation of joint itself
                R = model.joints[jj].R @ R
                t = R @ t + scale*model.joints[jj].t
                pts_temp = (model.joints[jj].R @ pts_temp.T).T + scale*model.joints[jj].t
                
                # Rotation from the angle (with respect to the model's origin)
                R_angle = ScipyR.from_rotvec(fit.angles[jj]*model.joints[jj].axis).as_matrix()
                origin_before = scale*model.links[jj].t
                origin_after = (R_angle @ origin_before.T).T
                pts_temp = (R_angle @ pts_temp.T).T
                pts_temp -= (origin_after-origin_before)

                R = R_angle @ R
                t -= (origin_after-origin_before)
            
        
        # Don't apply Base Rotation/Translation for Interactive Markers
        R_base = np.eye(3)
        t_base = np.zeros((3,))
        pts_temp = (R_base @ pts_temp.T).T + t_base
        R = R_base @ R
        t = R @ t + t_base

        # mesh needs quaterion instead of rotation matrix
        quat = ScipyR.from_matrix(R).as_quat()
        mesh_stl = model.links[ii].meshfile
        
        # update position and rotation of links
        markers[ii].pose.position.x = t[0]
        markers[ii].pose.position.y = t[1]
        markers[ii].pose.position.z = t[2]
        markers[ii].pose.orientation.x = quat[0]
        markers[ii].pose.orientation.y = quat[1]
        markers[ii].pose.orientation.z = quat[2]
        markers[ii].pose.orientation.w = quat[3]
        markers[ii].color.r = color[0]
        markers[ii].color.g = color[1]
        markers[ii].color.b = color[2]

        curr_iter+=1

def circ_marker(index, pos, size, color=[0.0, 1.0, 0.0], frame="map"):
    """ Creates circular markers on points of interest """
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = rospy.Time.now()
    marker.ns = "pts"
    marker.id = index
    marker.type = 2 # sphere
    marker.action = 0 # Add/Modify
    marker.pose.position.x = pos[0]
    marker.pose.position.y = pos[1]
    marker.pose.position.z = pos[2]
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1
    marker.scale.x = size
    marker.scale.y = size
    marker.scale.z = size
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    return marker

def modeltoMarkers(model,fit,curr_iter,color):
    ''' takes a model class instance (likely multiple meshes) and creates
        the appropriate markers for visualization '''
    markers = []
    for ii in range(0,len(model.links)):
        R = np.eye(3)
        t = np.zeros((3,))
        pts_temp = model.links[ii].pts
        scale = fit.scale

        ''' Need to calculate mesh transforms based on angles of rotation'''
        for jj in range(ii,-1,-1): # count from ii down to 0
            # link transform
            R = model.links[jj].R @ R
            t = R @ t - scale*model.links[jj].t
            pts_temp = (model.links[jj].R @ pts_temp.T).T - scale*model.links[jj].t

            # joint transform
            if (jj<len(model.joints) and ii!=0): # need to ignore final link (has no joint)
                # Transformation of joint itself
                R = model.joints[jj].R @ R
                t = R @ t + scale*model.joints[jj].t
                pts_temp = (model.joints[jj].R @ pts_temp.T).T + scale*model.joints[jj].t
                
                # Rotation from the angle (with respect to the model's origin)
                R_angle = ScipyR.from_rotvec(fit.angles[jj]*model.joints[jj].axis).as_matrix()
                origin_before = -scale*model.links[jj].t
                origin_after = (R_angle @ origin_before.T).T
                pts_temp = (R_angle @ pts_temp.T).T
                pts_temp -= (origin_after-origin_before)

                R = R_angle @ R
                t -= (origin_after-origin_before)
            
        R_base = fit.rot
        t_base = fit.pos

        pts_temp = (R_base @ pts_temp.T).T + t_base
        R = R_base @ R
        t = R @ t + t_base

        # mesh needs quaterion instead of rotation matrix
        quat = ScipyR.from_matrix(R).as_quat()
        mesh_stl = model.links[ii].meshfile
        
        marker_temp = mesh_to_marker(mesh_stl,curr_iter,t,quat,color,scale=scale)
        markers.append(copy.deepcopy(marker_temp))
        curr_iter+=1
    return markers, curr_iter


#!/usr/bin/env python3
# File name: demonstration_ros_wrapper.py
# Description: Creates affordance engine and deals with
# passing of information between the input and engine interfaces
# Author: Mike Hagenow
# Date: 6/17/2021

import argparse
import time
import open3d as o3d
import numpy as np
import trimesh
import rospy
import rospkg
from scipy.spatial.transform import Rotation as ScipyR

from affordance_corrections.affordance_engine import AffordanceEngine
from affordance_corrections.affordance_helpers.rviz_helpers import pcd_to_pc2, mesh_to_marker, circ_marker, modeltoMarkers, empty_marker
from affordance_corrections.affordance_helpers.InputHandler import inputLaunch
from affordance_corrections.ros_affordance_wrapper import ROSAffordances

from geometry_msgs.msg import PointStamped, Quaternion, Pose, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64

import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct

rosaff = None
angle = None

class ROSDemoAffordances(ROSAffordances):
    """ Keeps track of objects of interest, fits, etc. Specificially, provides a level on top of the
        affordance engine which interfaces with many ROS topics"""
    def __init__(self):
        ''' Set up additionalROS topics'''
        rospy.Subscriber("/panda/executeAT",String,self.executeAT)
        self.atpub = rospy.Publisher('panda/valve_at_pose', PoseStamped, queue_size=1)
        super().__init__('spacemouseNL')

    def executeAT(self,data):
        print(len(self.engine.objectsOfInterest))

        if len(self.engine.objectsOfInterest)>0:
            print("Desired angle: ", data.data)
            curr_angle = self.engine.objectsOfInterest[self.engine.active_obj].fits[self.engine.objectsOfInterest[self.engine.active_obj].active_id].angles[0]
            print("Current angle: ",curr_angle)
            print("Active Pos: ",self.engine.objectsOfInterest[self.engine.active_obj].fits[self.engine.objectsOfInterest[self.engine.active_obj].active_id].pos)
            print("Active Rot: ",ScipyR.from_matrix(self.engine.objectsOfInterest[self.engine.active_obj].fits[self.engine.objectsOfInterest[self.engine.active_obj].active_id].rot).as_quat())
            posetemp = PoseStamped()
            posetemp.header.frame_id = str(curr_angle)+":"+str(data.data)
            posetemp.pose.position.x = self.engine.objectsOfInterest[self.engine.active_obj].fits[self.engine.objectsOfInterest[self.engine.active_obj].active_id].pos[0]
            posetemp.pose.position.y = self.engine.objectsOfInterest[self.engine.active_obj].fits[self.engine.objectsOfInterest[self.engine.active_obj].active_id].pos[1]
            posetemp.pose.position.z = self.engine.objectsOfInterest[self.engine.active_obj].fits[self.engine.objectsOfInterest[self.engine.active_obj].active_id].pos[2]
            posetemp.pose.orientation.x = ScipyR.from_matrix(self.engine.objectsOfInterest[self.engine.active_obj].fits[self.engine.objectsOfInterest[self.engine.active_obj].active_id].rot).as_quat()[0]
            posetemp.pose.orientation.y = ScipyR.from_matrix(self.engine.objectsOfInterest[self.engine.active_obj].fits[self.engine.objectsOfInterest[self.engine.active_obj].active_id].rot).as_quat()[1]
            posetemp.pose.orientation.z = ScipyR.from_matrix(self.engine.objectsOfInterest[self.engine.active_obj].fits[self.engine.objectsOfInterest[self.engine.active_obj].active_id].rot).as_quat()[2]
            posetemp.pose.orientation.w = ScipyR.from_matrix(self.engine.objectsOfInterest[self.engine.active_obj].fits[self.engine.objectsOfInterest[self.engine.active_obj].active_id].rot).as_quat()[3]
            self.atpub.publish(posetemp)
    

def receivedScene(data):
    global rosaff, angle

    ## https://answers.ros.org/question/255351/how-o-save-a-pointcloud2-data-in-python/
    xyz = np.zeros((0,3))
    rgb = np.zeros((0,3))
    #self.lock.acquire()
    gen = pc2.read_points(data, skip_nans=True)
    int_data = list(gen)

    for x in int_data:
        test = x[3] 
        # cast float32 to int so that bitwise operations are possible
        s = struct.pack('>f' ,test)
        i = struct.unpack('>l',s)[0]
        # you can get back the float value by the inverse operations
        pack = ctypes.c_uint32(i).value
        r = (pack & 0x00FF0000)>> 16
        g = (pack & 0x0000FF00)>> 8
        b = (pack & 0x000000FF)
        # prints r,g,b values in the 0-255 range
                    # x,y,z can be retrieved from the x[0],x[1],x[2]
        xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)
        rgb = np.append(rgb,[[r,g,b]], axis = 0)

    # print("SHAPE: ",np.shape(xyz)," ",np.shape(rgb))
    rgb = rgb.astype(float)/255.0
    # print(xyz)
    # print(rgb)
    print("received a new scene", len(xyz))
    
    # save point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    pcd.colors = o3d.utility.Vector3dVector(rgb)
    o3d.io.write_point_cloud("/home/mike/Documents/CorrectionsIVA/kinect"+str(int(time.time()))+".pcd",pcd)

    rosaff.setSceneXYZRGB(xyz,rgb)
    time.sleep(1.0)
    if angle is not None and len(xyz)>10:
        rosaff.setFitting(True)
        rosaff.applyAngle(angle/1.57)


def setAngle(data):
    global angle
    angle = float(data.header.frame_id.split(":")[1])
    rosaff.setFitting(False)
    rosaff.applyAngle(angle/1.57)


def main():
    global rosaff
    rospy.init_node('affordance_wait_for_scan', anonymous=True)
    rospy.Subscriber("/pcscene", PointCloud2, receivedScene, queue_size=1)
    rospy.Subscriber("/panda/valve_at_pose", PoseStamped, setAngle, queue_size=1)
    rospack = rospkg.RosPack()
    root_dir = rospack.get_path('affordance_corrections')+'/../../'
    input_method = 'spacemouseNL'
    
    rosaff = ROSDemoAffordances()
    rosaff.toggleSVDforInitialArticulation(True)
    rosaff.setCppFitting(True)
    rosaff.setFitting(True)
    # rosaff.setModels([root_dir+'src/affordance_models/urdfs/ball_valve.urdf', root_dir+'src/affordance_models/urdfs/pvc_ball_valve.urdf', root_dir+'src/affordance_models/meshes/e_stop_coarse.STL', root_dir+'src/affordance_models/meshes/handle.STL', root_dir+'src/affordance_models/meshes/stop_valve_assem.STL'])
    rosaff.setModels([root_dir+'src/affordance_models/urdfs/ball_valve.urdf'])
    rosaff.runLoop()
    # rosaff.setModels([root_dir+'src/affordance_models/meshes/e_stop_coarse.STL', root_dir+'src/affordance_models/meshes/handle.STL', root_dir+'src/affordance_models/meshes/stop_valve_assem.STL', root_dir+'src/affordance_models/meshes/pvc_ball_valve_assem.STL', root_dir+'src/affordance_models/meshes/ball_valve_one_inch.STL'])
    
    
    # rosaff.runLoop()
    
if __name__ == '__main__':
    main()

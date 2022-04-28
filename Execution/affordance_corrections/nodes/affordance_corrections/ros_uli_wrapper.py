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
        self.regobjposepub = rospy.Publisher('/registeredObject',PoseStamped,queue_size=1)
        rospy.Subscriber("/getObjPose", String, self.getObjPose, queue_size=1)
        super().__init__('spacemouseNL')
    
    def getObjPose(self,data):
        # gets pose and model from affordance engine and publish out registration info
        global rosaff
        rot, pos, model_name = rosaff.engine.getActiveObjectPose()

        pose_temp = PoseStamped()
        pose_temp.header.frame_id = model_name
        pose_temp.pose.position.x = pos[0]
        pose_temp.pose.position.y = pos[1]
        pose_temp.pose.position.z = pos[2]

        R_temp = ScipyR.from_matrix(rot)
        quat_temp = R_temp.as_quat()

        pose_temp.pose.orientation.x = quat_temp[0]
        pose_temp.pose.orientation.y = quat_temp[1] 
        pose_temp.pose.orientation.z = quat_temp[2] 
        pose_temp.pose.orientation.w = quat_temp[3] 

        self.regobjposepub.publish(pose_temp)

        # print("OBJECT: ",model_name, R_temp.as_quat(), pos)

# TODO: get pose of object
# only one object?
# refit when desired?
# set the models in a different way

def receivedScene(data):
    
    startt = time.time()
    global rosaff, angle

    ## https://answers.ros.org/question/255351/how-o-save-a-pointcloud2-data-in-python/
    xyz = []
    rgb = []
    #self.lock.acquire()
    # gen = pc2.read_points(data, skip_nans=True)
    gen2 = pc2.read_points(data)
    int_data = list(gen2)

    if len(int_data)==0:
        return # no data received (blank point cloud)

    print("Received a new PC2: ",len(int_data))

    for ii in range(0,len(int_data)):
        x = int_data[ii]
        if ii%10000 == 0:
            print("  ",ii)
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
        xyz.append([x[0],x[1],x[2]])
        rgb.append([r,g,b])
        # rgb = np.append(rgb,[[r,g,b]], axis = 0)

    # print("SHAPE: ",np.shape(xyz)," ",np.shape(rgb))
    xyz = np.array(xyz)
    rgb = np.array(rgb)
    print("shape: ",np.shape(xyz))
    rgb = rgb.astype(float)/255.0
    # print(xyz)
    # print(rgb)
    print("received a new scene: ", len(xyz), " in ",time.time()-startt," seconds")

    rosaff.setSceneXYZRGB(xyz,rgb)
    time.sleep(1.0)

def main():
    global rosaff
    rospy.init_node('affordance_registration', anonymous=True)
    rospack = rospkg.RosPack()
    package_dir = rospack.get_path('affordance_corrections')+'/../../'
    
    rosaff = ROSDemoAffordances()
    rosaff.toggleSVDforInitialArticulation(True)
    rosaff.setCppFitting(True)
    rosaff.setFitting(True)
    models_str = rospy.get_param("/ros_uli_wrapper/models")
    model_dir = rospy.get_param("/ros_uli_wrapper/model_dir")
    models = []
    for model in models_str.split(";"):
        models.append(package_dir+'ULIConfig/registration_models/'+model)
    rosaff.setModels(models)
    rospy.Subscriber("/filtered_cloud", PointCloud2, receivedScene, queue_size=1)
    

    rosaff.runLoop()
    
if __name__ == '__main__':
    main()

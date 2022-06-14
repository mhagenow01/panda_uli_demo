#!/usr/bin/env python
# File name: ros_affordance_wrapper.py
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

from affordance_corrections.affordance_helpers.urdf_helpers import getPointsWithAngles

from geometry_msgs.msg import PointStamped, Quaternion, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64, Bool

import threading
from threading import Lock

class ROSAffordances:
    """ Keeps track of objects of interest, fits, etc. Specificially, provides a wrapper on top of the
        affordance engine which interfaces with many ROS topics"""
    def __init__(self,input_method):
        ''' Initalize other engines and set up ROS topics'''
        
        
        self.engine = AffordanceEngine()

        self.active = True # whether new points can be added, etc
        
        self.pts_of_interest = []
        self.num_meshes = 0
        self.camera_quat = np.array([0.5, 0.5, 0.5, 0.5]) #x,y,z,w
        self.camera_pos = np.array([0.0, 0.0, 0.0]) #x,y,z
        
        # Used for timing the loop, when refitting is needed, etc.
        self.timer = 0
        self.updateNeeded=False
        self.timeBeforeUpdate = 1.25 # time before ICP refits after changes
        self.timestep = 0.01 # main loop rate

        self.usevisualmodels = False

        # Rostopics: publishers and subscribers
        self.objspub = rospy.Publisher('meshes', MarkerArray, queue_size=1)
        self.pointsclickedpub = rospy.Publisher('points_clicked', MarkerArray, queue_size=10)
        self.clickedptlogger = rospy.Publisher('rvizclick', String, queue_size=1)
        self.scenepub = rospy.Publisher('scene', PointCloud2, queue_size=1)
        self.cameraqchangedpub = rospy.Publisher('rviz_camera_q_changed', Quaternion, queue_size=1)
        self.camerapchangedpub = rospy.Publisher('rviz_camera_p_changed', Point, queue_size=1)
        self.refittingupdatepub = rospy.Publisher('objRefittingUpdate',Bool, queue_size=1)
        self.fittingtimepub = rospy.Publisher('fittingTime',Float64,queue_size=1)
        self.refittingtimepub = rospy.Publisher('refittingTime',Float64,queue_size=1)
        self.articrecordingpub = rospy.Publisher('refittingTime',Float64,queue_size=1)
        rospy.Subscriber("clicked_point", PointStamped, self.pointClick)
        rospy.Subscriber("delete_active", String,self.deleteActive)
        rospy.Subscriber("rviz_camera_q", Quaternion, self.setCameraQ)
        rospy.Subscriber("rviz_camera_p", Point, self.setCameraP)
        rospy.Subscriber("rviz_triggers", String, self.triggers)
        rospy.Subscriber("pc_flip", String, self.applyFlip)
        rospy.Subscriber("update_scene",String,self.updateScene)
        rospy.Subscriber("correction", Twist, self.applyCorrection)
        rospy.Subscriber("articulation",Float64,self.applyArticulation)
        rospy.Subscriber("nextarticulation",String,self.nextArticulation)

        rospy.sleep(1) # sleep one second to allow publishers to register

        # Separate thread for the input device handler
        t = threading.Thread(name='input_thread', target=inputLaunch, args=[input_method])
        t.start()

        self.lock = Lock() # avoid duplicate calls to visualization


    def deleteActive(self):
        ''' If there is an active object, delete it! '''
        if len(self.pts_of_interest)>0:
            self.pts_of_interest.pop(self.engine.active_obj)
        
        self.engine.deleteActivePoint()
        str_temp = String()
        str_temp.data = 'delete_active'
        self.clickedptlogger.publish(str_temp)
        worldState = self.engine.getUpdatedWorld()
        self.updateVisualization(worldState)

    def clearMarkerArray(self,num_markers,mArr_pub):
        ''' Used to get rid of both temporary markers (e.g., before fit finishes) and meshes (e.g. clear button).
         This will take a markerarray topic and individually remove each entry '''
        markers = MarkerArray()
        for ii in range(0,num_markers):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.action = marker.DELETEALL # for some reason, DELETE doesn't work
            markers.markers.append(marker)
        mArr_pub.publish(markers)

    def getPtAlongLine(self,pt):
        # https://stackoverflow.com/questions/61582739/how-to-find-the-nearest-point-to-the-line
        # modified to multiply by the distance from the camera to the scene to prioritize closer points
        startt = time.time()
        acc_dist_to_click = 0.005 # 0.5 cm
        dist_from_line = np.linalg.norm(np.cross(pt-self.camera_pos, self.camera_pos-self.scenenp, axisb=1), axis=1)/np.linalg.norm(pt-self.camera_pos)
        close_enough_inds = np.argwhere(dist_from_line<acc_dist_to_click).flatten()
        if len(close_enough_inds)==0: # no points found, return flag
            return -1
        closest_point = self.scenenp[close_enough_inds[np.argmin(np.linalg.norm(self.camera_pos-self.scenenp[close_enough_inds],axis=1))]]
        # closest_point = self.scenenp[np.argmin(np.multiply(np.power(np.linalg.norm(self.camera_pos-self.scenenp,axis=1), 10),np.linalg.norm(np.cross(pt-self.camera_pos, self.camera_pos-self.scenenp, axisb=1), axis=1)/np.linalg.norm(pt-self.camera_pos)))]
        return closest_point

    def setScene(self,file):
        ''' Load a given point cloud scene (expects pcd file) '''
        worldState = self.engine.getUpdatedWorld()
        self.updateVisualization(worldState)
        
        self.engine.setScene(file)
        # Store open3d for linecasting (finding closest point on point cloud to clicked direction)
        scene_temp = o3d.io.read_point_cloud(file)
        scene_temp = scene_temp.voxel_down_sample(voxel_size=0.005)
        self.scenenp = np.asarray(scene_temp.points)

        self.clearAllPts()
        worldState = self.engine.getUpdatedWorld()
        self.updateVisualization(worldState)

    def setSceneXYZRGB(self,xyz,rgb):
        ''' Load a given point cloud from xyz and rgb data (nx3 for each) '''
        worldState = self.engine.getUpdatedWorld()
        self.updateVisualization(worldState)

        self.engine.setSceneXYZRGB(xyz,rgb)
        # Store open3d for linecasting (finding closest point on point cloud to clicked direction)
        scene_temp = o3d.geometry.PointCloud()
        scene_temp.points = o3d.utility.Vector3dVector(xyz)
        scene_temp.colors = o3d.utility.Vector3dVector(rgb)
        scene_temp = scene_temp.voxel_down_sample(voxel_size=0.005)
        self.scenenp = np.asarray(scene_temp.points)

        # self.clearAllPts()
        worldState = self.engine.getUpdatedWorld()
        self.updateVisualization(worldState)
    
    def updateScene(self,data):
        ''' Update the view of the 3d scene (assumes it is localized wrt previous scenes)
            also triggers the refitting of all current objects '''
        
        pcd_file = data.data
        scene_rot = np.array([0.0, 0.0, 0.0, 1.0])
        scene_trans = np.array([0.0, 0.0, 0.0])

        # can either be a pcd file or a pcd file with a rotation and translation for the pcd
        if "scenepose" in data.data:
            pcd_file = data.data.split("scenepose")[0]
            pose = data.data.split("scenepose:")[1]
            scene_rot = np.fromstring(pose.split(",")[0].split("[")[1].split("]")[0],dtype=float,sep=" ")
            scene_trans = np.fromstring(pose.split(",")[1].split("[")[1].split("]")[0],dtype=float,sep=" ")

        # Store open3d for raycasting
        scene_temp = o3d.io.read_point_cloud(pcd_file)
        scene_temp = scene_temp.voxel_down_sample(voxel_size=0.005)
        self.scenenp = np.asarray(scene_temp.points)
        print(np.shape(self.scenenp))
        R_scene = ScipyR.from_quat(scene_rot)
        self.scenenp = R_scene.apply(self.scenenp)
        self.scenenp = self.scenenp + scene_trans

        self.engine.setScene(pcd_file,scene_rot = scene_rot,scene_trans = scene_trans)
        if "scenepose" in data.data:
            # clear if there is a scenepose
            self.clearAllPts()
        else:
            self.engine.refit_all_objects()
        worldState = self.engine.getUpdatedWorld()
        self.updateVisualization(worldState)

    def clearAllPts(self):
        ''' Removes all tracked objects of interest and clears marker array topic'''
        self.pts_of_interest.clear()
        self.engine.objectsOfInterest.clear()
        self.engine.active_obj = 0
        self.clearMarkerArray(self.num_meshes,self.objspub)

    def triggers(self,data):
        ''' Captures triggering events (strings) from the rviz front end (buttons, etc)'''
        if data.data=="clear":
            self.clearAllPts()
        if data.data=="deleteactive":
            self.deleteActive()
        if data.data=="toggleactive":
            if len(self.engine.objectsOfInterest)>0:
                activeid = self.engine.active_obj
                self.engine.objectsOfInterest[activeid].toggleActive()
        if data.data=="refittingon":
            self.engine.setActiveObjectRefitting(True)
        if data.data=="refittingoff":
            self.engine.setActiveObjectRefitting(False)

    def setActive(self,active):
        ''' determine whether the system is active -- should be able to create fits, corrections, etc. Scene is gray when inactive'''
        self.active = active
        worldState = self.engine.getUpdatedWorld()
        self.updateVisualization(worldState)

    def setModels(self,models, pkg_dir = None):
        ''' Takes a list of filenames for the objects to be fit'''
        self.engine.setModels(models, pkg_dir)

    def toggleSVDforInitialArticulation(self,toggle):
        ''' Tells affordance engine whether or not to use SVD for articulation
            fitting (i.e., avoid NL fitting for random restarts)'''
        self.engine.toggleSVDforInitialArticulation(toggle)

    def setCppFitting(self,cppfitting):
        ''' Tells affordance engine whether or not to use CPP fitting'''
        self.engine.setCppFitting(cppfitting)

    def setCppRefitting(self,cpprefitting):
        ''' Tells affordance engine whether or not to use CPP fitting'''
        self.engine.setCppRefitting(cpprefitting)

    def setFitting(self,fitting):
        ''' Tells engine whether or not to do fitting or just set/update poses'''
        self.engine.setFitting(fitting)

    def useVisualModels(self,usevisual):
        ''' Tells engine whether or not visual models are used '''
        self.usevisualmodels = usevisual

    def setCameraQ(self, data):
        ''' Takes ros Quaternion msg and updates known camera orientation for camera-centric controls'''
        if not np.array_equal(self.camera_quat,np.array([data.x, data.y, data.z, data.w])):
            self.cameraqchangedpub.publish(data)
            self.camera_quat = np.array([data.x, data.y, data.z, data.w])

    def setCameraP(self, data):
        ''' Takes ros Point msg and updates known camera position for camera-centric controls'''
        if not np.array_equal(self.camera_pos,np.array([data.x, data.y, data.z])):
            self.camerapchangedpub.publish(data)
            self.camera_pos = np.array([data.x, data.y, data.z])

    def pointClick(self, data):
        ''' Determine action to take when a point is selected in a point cloud '''
        if self.active:
            pt = np.array([data.point.x, data.point.y, data.point.z])
            pt = self.getPtAlongLine(pt) # project onto point cloud
            
            if isinstance(pt,int) and pt ==-1: # failed to get close point
                return # don't do anything
            
            str_temp = String()

            # Options are either: (1) set object to active, (2) cycle object model, or (3) fit new model to pt
            nearby = self.engine.nearbyPointOfInterest(pt,self.camera_pos)
            if nearby != -1 and nearby!=self.engine.active_obj: # there is an object there already
                print("Change Active!")
                str_temp.data = 'change_active'
                self.clickedptlogger.publish(str_temp)
                self.engine.setActiveObject(nearby)

                # Tell rviz whether active object has refitting
                updateRefitting = Bool()
                updateRefitting.data = self.engine.getActiveObjectRefitting()
                self.refittingupdatepub.publish(updateRefitting)
            elif nearby !=-1: # already active, change to next best model fit
                print("Change Model!")
                str_temp.data = 'change_model'
                self.clickedptlogger.publish(str_temp)
                self.engine.objectsOfInterest[nearby].toggleActive()
            else: # fit a new object
                if self.engine.ptNearScene(pt,0.005):
                    print("Identifying Object!")
                    str_temp.data = 'new_point'
                    self.clickedptlogger.publish(str_temp)
                    # adds a temporary marker while fitting
                    self.pts_of_interest.append(pt)
                    worldState = self.engine.getUpdatedWorld()
                    self.ptsInterestVis(worldState)
                    startt = time.time()
                    self.engine.addObjOfInterest(pt)
                    if self.engine.fitting: # if fitting, record fitting time
                        fit_temp = Float64()
                        fit_temp.data = time.time()-startt
                        self.fittingtimepub.publish(fit_temp)

                    # Tell rviz whether active object has refitting
                    updateRefitting = Bool()
                    updateRefitting.data = self.engine.getActiveObjectRefitting()
                    self.refittingupdatepub.publish(updateRefitting)
                else:
                    print("Invalid selected point")
            
            # reset the scene
            worldState = self.engine.getUpdatedWorld()
            self.updateVisualization(worldState)

    def rotateCorrectionByCamera(self,twist):
        """ This performs the conversion from raw input based on the rviz camera
            Camera in RVIZ has an offset from the original x,y,z of scene objects
            There is also a 180 degree rotation (z-axis) between the input and scene"""
        R_offset = ScipyR.from_quat(np.array([0.5, 0.5, 0.5, 0.5]))
        R_offset_2 = ScipyR.from_quat(np.array([0, 0, 1, 0]))
        R_camera = ScipyR.from_quat(self.camera_quat)

        # Apply and create new output (as numpy array)
        R_adjusted =  R_camera * R_offset.inv() * R_offset_2 
        new_linear = R_adjusted.apply(twist[0:3])
        new_ang = R_adjusted.apply(twist[3:6])
        return np.append(new_linear,new_ang)
    
    def applyCorrection(self,twist):
        ''' Take a twist and apply it to the current pose of the active object'''
        if self.active:
            self.updateNeeded= True
            self.timer = 0.0
        
            # update the pose of the active object based on the correction
            twist_array = np.array([twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z])
            twist_array = self.rotateCorrectionByCamera(twist_array)
            self.engine.twist_active_object(twist_array)
            
            # update visualization
            worldState = self.engine.getUpdatedWorld()
            self.updateVisualization(worldState, sceneChange=False, changing=True)

    def nextArticulation(self,data):
        ''' ROS message tells engine corrections are wanted to a different articulation'''
        self.engine.cycle_active_articulation()

    def applyArticulation(self,delta_theta):
        ''' Take a single angle articulation and apply it to the current pose of the active object'''

        self.updateNeeded= True
        self.timer = 0.0

        self.engine.articulate_active_object(delta_theta.data)
        
        # update visualization
        worldState = self.engine.getUpdatedWorld()
        self.updateVisualization(worldState, sceneChange=False, changing=True)

    def applyAngle(self,theta):
        ''' Take a single angle articulation and apply it to the current pose of the active object'''

        self.updateNeeded= True
        self.timer = 0.0
       
        self.engine.angle_active_object(theta)
        
        # update visualization
        worldState = self.engine.getUpdatedWorld()
        self.updateVisualization(worldState, sceneChange=False, changing=True)

    def applyFlip(self,data):
        ''' Tell the active object to flip with respect to the "next" principal axis
            The object itself keeps track of the flipping '''

        self.updateNeeded= True
        self.timer = 0.0
        
        # update the pose of the active object based on the correction
        self.engine.flip_active_object()
        
        # update visualization
        worldState = self.engine.getUpdatedWorld()
        self.updateVisualization(worldState, sceneChange=False, changing=True)

    def runLoop(self):
        ''' This is the main loop that checks for updates/corrections to the object '''
        while not rospy.is_shutdown():
            rospy.sleep(self.timestep)
            self.timer+=self.timestep
            # print("timer: ",self.timer)

            # If it has been enough time since a correction, refit the object
            if self.engine.fitting and len(self.engine.objectsOfInterest)>0 and self.timer>self.timeBeforeUpdate and self.updateNeeded:
                # Run ICP on updated object and update scene
                startt = time.time()
                self.engine.refit_active_object()
                refit_temp = Float64()
                refit_temp.data = time.time()-startt
                self.refittingtimepub.publish(refit_temp)

                # Publish that system is refitting and which model
                active_obj = self.engine.objectsOfInterest[self.engine.active_obj]
                active_model_name = active_obj.models[active_obj.active_id].name
                str_temp = String()
                str_temp.data = 'refit_model:'+active_model_name
                self.clickedptlogger.publish(str_temp)

                worldState = self.engine.getUpdatedWorld()
                self.updateVisualization(worldState, sceneChange=False, changing=False)
                self.updateNeeded = False

    def updateVisualization(self, worldState, sceneChange=True, changing=False):
        ''' Use Engine world to update visualization
        Note: worldState is a dictionary '''

        self.lock.acquire()

        self.clearMarkerArray(len(self.pts_of_interest),self.pointsclickedpub)

        # Query relevant information from the affordance engine
        scene = worldState.get('scene')
        objectsOfInterest = worldState.get('objectsOfInterest')
        active_obj = worldState.get('active_obj')

        # see whether any temporary markers need to be removed
        self.ptsInterestVis(worldState)

        # Scene is not always published (since it can be large)
        if sceneChange and scene is not None:
            pc2 = pcd_to_pc2(scene, self.active)
            self.scenepub.publish(pc2)

        # publish each one of the meshes in the affordanceEngine
        meshes = MarkerArray()
        count = 0
        for ii in range(0,len(objectsOfInterest)):
            active_id = objectsOfInterest[ii].active_id
            model = objectsOfInterest[ii].models[active_id]
            fit = objectsOfInterest[ii].fits[active_id]
            
            color = np.array([0.5, 0.5, 0.5]) # gray for inactive
            if ii==active_obj:
                if changing:
                    color = np.array([0.5, 0.8, 0.5]) # mod. green for active, but changing
                else:
                    color = np.array([0.2, 0.8, 0.2]) # green for active

            markers_temp, count = modeltoMarkers(model,fit,count,color,self.usevisualmodels)
            for marker in markers_temp:
                meshes.markers.append(marker)

            # pts = getPointsWithAngles(model,fit.angles,fit.rot,fit.pos)
            # pcd = o3d.geometry.PointCloud()
            # pcd.points=o3d.utility.Vector3dVector(pts)
            # pcd.colors = o3d.utility.Vector3dVector(np.zeros(np.shape(pts)))
            # pc2 = pcd_to_pc2(pcd)
            # self.scenepub.publish(pc2)
        
        empty_marker(count,self.num_meshes,meshes) # used to add blank markers if the count has decreased since last call
        self.num_meshes = count
        self.objspub.publish(meshes)

        self.lock.release()

    def ptsInterestVis(self,worldState):
        ''' Used to publish points in the scene where the user has selected the point, but
            the fitting has not completed '''

        # separate marker array in rviz
        pts = MarkerArray()
        objectsOfInterest = worldState['objectsOfInterest']

        # Look through the points and publish when a point has been selected, but the fit mesh
        # doesn't yet exist
        for ii in range(0,len(self.pts_of_interest)):
            pt = self.pts_of_interest[ii]
            # check whether a corresponding mesh already exists in world state
            plot_marker=True
            for jj in range(0,len(objectsOfInterest)):
                if np.linalg.norm(objectsOfInterest[jj].ref_point-pt)==0.0: # if the points are the same
                    plot_marker=False
            if plot_marker:
                color = np.array([0.2, 0.8, 0.2]) # green for active
                marker_temp = circ_marker(ii,pt,0.02,color)
                pts.markers.append(marker_temp)
        
        # Requires a blank publishing if you want to get rid of objects
        self.clearMarkerArray(len(self.pts_of_interest),self.pointsclickedpub)
        self.pointsclickedpub.publish(pts) # publish markers that should be visible


def main():
    rospy.init_node('affordance_specifications', anonymous=True)
    rospack = rospkg.RosPack()
    root_dir = rospack.get_path('affordance_corrections')+'/../../'
    input_method = rospy.get_param("ros_affordance_wrapper/input","teleop")
    
    rosaff = ROSAffordances(input_method)
    rosaff.setFitting(True)
    rosaff.toggleSVDforInitialArticulation(True)
    rosaff.setCppFitting(True)
    rosaff.setScene(root_dir+'pcd/fakebaneliva.pcd')

    rosaff.setModels([root_dir+'src/affordance_models/meshes/space_drill.STL',root_dir+'src/affordance_models/meshes/stringer.STL'])
    # rosaff.setModels([root_dir+'src/affordance_models/urdfs/ball_valve.urdf', root_dir+'src/affordance_models/urdfs/pvc_ball_valve.urdf', root_dir+'src/affordance_models/meshes/e_stop_coarse.STL', root_dir+'src/affordance_models/meshes/handle.STL',root_dir+'src/affordance_models/urdfs/gas_valve.urdf'])
    # rosaff.setModels([root_dir+'src/affordance_models/meshes/e_stop_coarse.STL', root_dir+'src/affordance_models/meshes/handle.STL', root_dir+'src/affordance_models/meshes/stop_valve_assem.STL', root_dir+'src/affordance_models/meshes/pvc_ball_valve_assem.STL', root_dir+'src/affordance_models/meshes/ball_valve_one_inch.STL'])
    # rosaff.setModels([root_dir+'src/affordance_models/urdfs/gas_valve.urdf'])
    
    rosaff.runLoop()
    
if __name__ == '__main__':
    main()

#!/usr/bin/env python

"""
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

#!/usr/bin/env python3
# File name: InteractiveMesh.py
# Description: Interactive Marker Class for Visualization (based on rviz tutorial - http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Getting%20Started)
# Author: Mike Hagenow
# Date: 6/29/2021

import rospy
import copy
from scipy.spatial.transform import Rotation as ScipyR

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point, Quaternion

from tf.broadcaster import TransformBroadcaster

# Added importants on top of tutorial
import rospkg
import numpy as np
from affordance_corrections.affordance_helpers.rviz_helpers import mesh_to_interactive_marker, modeltoInteractiveMarker, modelUpdateInteractiveMarker
from affordance_corrections.affordance_helpers.Model import Model
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String

class InteractiveMesh:
  ''' Class defines an interactive marker with 6D pose control and a mesh'''
  def __init__(self,lock,models,server, pose_update_pub, model_update_pub, rviztrig_pub, int_correctionpub, fits,sorted_ids,mark_id=0,active_id=0,color=np.array([0.2, 0.8, 0.2])):
    self.id = mark_id

    self.lock = lock
    self.lock.acquire()

    self.models = models # models are a Model object

    self.server = server
    self.int_marker = InteractiveMarker()
    self.int_marker.name = "intmesh"+str(mark_id)
    
    self.menu_handler = MenuHandler()

    self.fits = fits
    self.active_id = active_id
    self.sorted = sorted_ids
    pos = fits[active_id].pos
    quat = ScipyR.from_matrix(fits[active_id].rot).as_quat()
    
    self.int_marker.header.frame_id = "map"
    self.int_marker.pose.position = Point(pos[0],pos[1],pos[2]) # ref_pt is a numpy array
    self.int_marker.pose.orientation = Quaternion(quat[0],quat[1],quat[2],quat[3])

    self.pose_update_pub = pose_update_pub
    self.model_update_pub = model_update_pub
    self.rviztrig_pub = rviztrig_pub
    self.int_correctionpub = int_correctionpub
    # initial pose
    self.pose = Pose()

    self.initializeMesh(active_id,color) # initialize to first mesh
    self.setupModelMenu()
    self.add_6D_controls()

    self.server.insert(self.int_marker, self.processFeedback)
    self.menu_handler.apply(self.server,self.int_marker.name)
    self.server.applyChanges()
    self.lock.release()

  def updateIntMarkerPose(self,pos,quat):
     # after pose is propogated to ros_marker_wrapper, send back updated postion
     self.int_marker.pose.position = Point(pos[0],pos[1],pos[2]) # ref_pt is a numpy array
     self.int_marker.pose.orientation = Quaternion(quat[0],quat[1],quat[2],quat[3])
     self.int_correctionpub.publish(self.int_marker.pose)
     self.server.applyChanges()

  def updateIntMarkerAngle(self,angle):
     # propogate angle change from ros_marker_wrapper to interactive marker
     active_id = self.active_id
     if len(self.fits[active_id].angles)==1:
        self.fits[active_id].angles = [angle]
        modelUpdateInteractiveMarker(self.models[active_id],self.fits[active_id],self.int_marker.controls[0].markers,self.id,np.array([0.5, 0.8, 0.5]))
        # self.int_marker.controls[0].markers = markers_updated
        self.server.insert(self.int_marker, self.processFeedback)
        self.server.applyChanges()

  def setupModelMenu(self):
    ''' menu is used to display available model names'''
    for ii in range(0,len(self.models)):
      self.menu_handler.insert(self.models[self.sorted[ii]].name, callback=self.processFeedback)
    
    menu_control = InteractiveMarkerControl()
    menu_control.interaction_mode = InteractiveMarkerControl.MENU
    menu_control.name = "mesh_menu"
    menu_control.description = "Fit: "+str(self.id)
    self.int_marker.controls.append(copy.deepcopy(menu_control))

  def processFeedback(self,feedback):
    ''' when the interactive mesh state changes, process: 1 menu change, 2. pose update, 3. pose update complete (possibly trigger refit)'''
    self.lock.acquire()

    # update base pose internally
    pos = [feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z]
    quat = [feedback.pose.orientation.x, feedback.pose.orientation.y, feedback.pose.orientation.z, feedback.pose.orientation.w]
    self.updateIntMarkerPose(pos,quat)
    
    if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        # change to the newly selected mesh
        # self.setMesh(feedback.menu_entry_id-1)
        str_temp = String()
        str_temp.data = str(self.id)+","+str(self.sorted[feedback.menu_entry_id-1])
        self.model_update_pub.publish(str_temp)
    elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        # keep track of pose update
        active_id = self.active_id
        modelUpdateInteractiveMarker(self.models[active_id],self.fits[active_id],self.int_marker.controls[0].markers,self.id,np.array([0.5, 0.8, 0.5]))
        self.server.insert(self.int_marker, self.processFeedback)
        str_temp = String()
        str_temp.data = "int_marker_updates"
        self.rviztrig_pub.publish(str_temp) # tell system to not refit yet
        
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        # publish once the mouse is released
        self.publishPose(feedback.pose)

    self.server.applyChanges()
    self.lock.release()

  def initializeMesh(self,active_id,color):
    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.NONE
    control.always_visible = True
    self.int_marker.scale = self.models[active_id].diameter*1.25 # add margin

    markers = modeltoInteractiveMarker(self.models[active_id],self.fits[active_id],self.id,color)
    for marker in markers:
      control.markers.append(marker)

    self.int_marker.controls.append(copy.deepcopy(control))
    self.server.applyChanges()
    

  def updatecolor(self):
    ''' change color to green when updating'''
    light_green = np.array([0.5, 0.8, 0.5])
    self.int_marker.controls[0].markers[0].color.r = light_green[0]
    self.int_marker.controls[0].markers[0].color.g = light_green[1]
    self.int_marker.controls[0].markers[0].color.b = light_green[2]
    self.server.insert(self.int_marker, self.processFeedback)
    self.server.applyChanges()
  
  def setMesh(self,active_id):
    ''' change mesh when model selection changes'''
    mesh_id = 0 # hard-coded based on setup
    self.int_marker.controls[mesh_id].markers.clear()
    self.int_marker.controls[mesh_id].markers.append(mesh_to_interactive_marker(self.models[active_id].file,np.array([0.2, 0.8, 0.2])))
    self.server.insert(self.int_marker, self.processFeedback)
    self.server.applyChanges()

  def publishPose(self,pose):
    ''' publish pose of int marker for backend tracking'''
    pose_s = PoseStamped()
    # pass a reference to the specific marker in the header frame
    pose_s.header.frame_id = str(self.id)
    pose_s.pose = pose
    self.pose_update_pub.publish(pose_s)

  def add_6D_controls(self):
    ''' set up controls (except for menu)'''
    ws = [1,1,1]/np.sqrt(2)
    xs = [1,0,0]/np.sqrt(2)
    ys = [0,1,0]/np.sqrt(2)
    zs = [0,0,1]/np.sqrt(2)
    names = ['x','y','z']
    control = InteractiveMarkerControl()

    for ii in range(0,3):
      control.orientation.w = ws[ii]
      control.orientation.x = xs[ii]
      control.orientation.y = ys[ii]
      control.orientation.z = zs[ii]
      control.name = "rotate_"+names[ii]
      control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
      self.int_marker.controls.append(copy.deepcopy(control))
      control.name = "move_"+names[ii]
      control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
      self.int_marker.controls.append(copy.deepcopy(control))

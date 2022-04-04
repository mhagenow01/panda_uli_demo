#!/usr/bin/env python3

""" Scripts to run/learn various model templates for the ULI demo

 Last Updated: 04/04/2022
"""

__author__ = "Mike Hagenow"

from corrective_shared_autonomy.PreProcessing.PreProcessing import PreProcessing
from corrective_shared_autonomy.TaskModels.DMPLWRhardcoded import DMPLWRhardcoded
from corrective_shared_autonomy.PreProcessing.solidworks_to_bsplinesurface import surfaceFromSTL

from geometry_msgs.msg import PoseStamped

import rospy
import rospkg
import os.path

import numpy as np

class ModelRunner():
    def __init__(self):
        self.runningModel = False
        rospy.Subscriber("/registeredObject", PoseStamped, self.modelExecution)
        rospy.spin()
        
    def modelExecution(self,data):
        model_name = data.header.frame_id
    
        if self.runningModel is False:
            self.runningModel = True

            quat = data.pose.orientation
            pos = data.pose.position
            R_surface = np.array([quat.x, quat.y, quat.z, quat.w])
            t_surface = np.array([pos.x, pos.y, pos.z])

            model = DMPLWRhardcoded(verbose=True)

            rospack = rospkg.RosPack()
            config_dir = rospack.get_path('corrective_shared_autonomy')+'/../../ULIConfig/registration_models/'
            execution_path = config_dir+model_name+'.pkl'

            # Run the model if it exists
            if os.path.isfile(execution_path):
                model.executeModel(model_pkl_file=execution_path, R_surface = R_surface, t_surface=t_surface)
            else:
                print("No Execution Model found for model: ",model_name)

            self.runningModel = False

    
if __name__ == "__main__":
    rospy.init_node('model_runner', anonymous=True)
    modelrunner = ModelRunner()
    
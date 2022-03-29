#!/usr/bin/env python3
# File name: cpp_fitting_testing.py
# Description: Used to compare new C++ fitting with the existing python fitting algorithm
# Author: Mike Hagenow
# Date: 9/23/2021

import time
import os
import numpy as np
import open3d as o3d
import rospkg

from affordance_corrections.affordance_helpers.pcl_helpers import get_cloud_of_interest, get_cloud_of_interest_np
from affordance_corrections.affordance_helpers.Model import Model
from affordance_corrections.affordance_helpers.registration import get_fits_for_models, FitInfo
from affordance_corrections.affordance_helpers.urdf_helpers import getPointsWithAngles

import ctypes
from affordance_corrections.affordance_helpers.cython_helpers import packageFittingInfo, convert_fits, FITHOLDER

def main():
    
    ############################
    # Set up data              #
    ############################
    rospack = rospkg.RosPack()
    root_dir = rospack.get_path('affordance_corrections')+'/../../'

    # Create two test models -- only one is the actual object
    model1path = root_dir+'src/affordance_models/meshes/pvc_ball_valve_assem.STL'
    _ = Model(model1path)

    

    
if __name__ == '__main__':
    main()

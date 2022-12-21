#!/usr/bin/env python3

""" Scripts to run/learn various models

 Last Updated: 09/07/2021
"""

__author__ = "Mike Hagenow"

import sys
import os
print("PATH:", sys.path)
#sys.path.append(os.path.join(os.path.dirname(__file__), 'PreProcessing'))
print()
#print(os.path.join(os.path.dirname(__file__), 'PreProcessing'))
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
print()
print("PATH:", sys.path)
#print(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
print()
print()

# TODO: fix import statement error
# 1. Print sys path and see if module is in path
    # so where is the package? 
    # A: /home/nitzan/Code/panda_ws/src/panda_uli_demo/Execution/corrective_shared_autonomy/nodes/corrective_shared_autonomy/PreProcessing
    # Current Sys path?
    # A: /home/nitzan/Code/panda_ws/src/panda_uli_demo/Execution/corrective_shared_autonomy/nodes/corrective_shared_autonomy
# 2. See if module is listed, as needed, in the CMAKELists and other files
# Try to append to path the upper-level corrective_shared_autonomy folder

#3. Fix compilation issues, starting with the rust one that's cargo build failing. See Kevin's docs for installing rust maybe?
# Done

#4. Fix compilation issue in target_link_libraries of CMakeLists where it can't find DHDC

'''
STEPS for Running run_models.py:

2. Install rust (so cargo build --release would work)
3. Put libdhd.so.3.8.0 in /usr/local/lib
4. Install Pico Tree from source (pip may be fine too) https://github.com/Jaybro/pico_tree
    (I installed in documents/lib_installs.) Note that the build command with --target docs won't compile
    without doxygen. You'll need to use sudo when executing the install command.
5. Install nlopt: https://github.com/stevengj/nlopt
6. If eigen is causing issues, specifically if you have the wrong version of eigen, just ignore that
7. Install nvm: https://github.com/nvm-sh/nvm#installing-and-updating
8. nvm install node (this install npm)
9. In panda_uli_demo/Interface/react: `npm install`
10. `npm start` , but if it doesnt work and you get an error about an envelope, see 
        this: https://stackoverflow.com/questions/69692842/error-message-error0308010cdigital-envelope-routinesunsupported
        I followed this suggestion and it fixed the error: export NODE_OPTIONS=--openssl-legacy-provider
        Now when you do npm start, it will take a while but eventually it loads, even if it suggests
        installing something
11. sudo apt-get install ros-noetic-rosbridge-server 
12. sudo apt-get install ros-noetic-tf2-web-republisher
13. pip install pynput
14. pip install dtw
15. pip install plotly
16. For the frontend, change the lan IP address here: /home/nitzan/Code/panda_ws/src/panda_uli_demo/Interface/react/src/RosStore.js
        
17. pip install open3d
18. pip install trimesh
19. sudo apt install spacenavd
20. sudo apt install ros-noetic-spacenav-node
25. To use the C++ verison of the registration:
    * Get eigen 4.3 (branch 4.3) here: https://gitlab.com/libeigen/eigen
    * /home/nitzan/Code/panda_ws/src/panda_uli_demo/Execution/affordance_corrections/nodes/affordance_corrections/affordance_helpers/cppfitting/CMakeLists.txt
    Needs to have: find_package(Eigen3 EXACT 3.4.0 REQUIRED PATHS /home/nitzan/Documents/lib_installs/eigen/)

1. ONLY From the ./compile directory ONLY, run ./compile ----- BUT, all compiling of uli_interface is
    currently missing from the ./compile script


150. To change whether registration happens with C++ (fast) or Python (slow) go here: /home/nitzan/Code/panda_ws/src/panda_uli_demo/Execution/affordance_corrections/nodes/affordance_corrections/ros_uli_wrapper.py
    and change rosaff.setCppFitting(False)
151. To choose the stl model to fit to: in middleend.launch
152. To choose the 3D device to use: also in middleend.launch

175. Set movement behaviors using jupyter notebook.

200. (nitz_host) Set master URI and ROS IP on both the remote laptop and robot laptop to be the remote laptop IP
202. On robot laptop: `roslaunch uli_config robot.launch` Once "biasing..." print out shows up, the system is running
203. On off-robot laptop: `roslaunch uli_config middleend.launch`
205. In rviz run scan
210. Press C to do registration (aka 'publish point' button in rviz)
215. If model seems to jump around when fitting it using input device, then delete model, and repeat steps beginning with pressing C


TROUBLESHOOTING:
305. If kinect has errors starting up, unplug it from computer, wait a few seconds, and plug it back in.

'''

'''
This file takes 

Nitzan's Tasks:

This file takes imperfect demonstrations (x, y, z, t, force, tool_state?), and aligns them in time (and position?),
then learns a policy from them using __EM generative method?__ ? We need to extract variability, or does it
do that already?

1. Understand PreProcessing. What does preprocessing and set_demonstrations do?

2. What the difference between DMP/LWR and EM Generative method in Ng's LfD paper?

3. How doe dynamic time warping (DTW) work?
    Given two time series: A and B, it correlates points in A to points in B with the premise
    of minimizing distance. One method is uses is mapping one point in A to multiple points 
    in B, or vice verse.

4. What is this learning thing? What are states for? What's the pipeline like?

'''

from corrective_shared_autonomy.PreProcessing.PreProcessing import PreProcessing
from corrective_shared_autonomy.TaskModels.DMPLWR import DMPLWR
from corrective_shared_autonomy.PreProcessing.solidworks_to_bsplinesurface import surfaceFromSTL

import numpy as np

def testLearnCaulking():
    # Preprocessing
    data = PreProcessing(verbose=True)
    data.set_demonstrations("/home/mike/Documents/LearningCorrections/data/10-22-21/demos")
    # data.set_demonstrations("/home/mike/Documents/LearningCorrections/sealant/10-7/")
    data.setRegistrationTransform(None,None,"/home/mike/Documents/LearningCorrections/data/10-22-21/rigid_10-22-21_2.bag")
    data.setTool("caulking")
    names, states = data.calculateStates()
    data.plotPos()

    # Learning
    model = DMPLWR(verbose=True)
    model.learnModel(names,states,"sealant2.pkl")

def plotLearnedCaulking():
    model = DMPLWR(verbose=True)
    model.plotLearned("sealant2.pkl")
    model.plotLearnedCorrections("sealant2.pkl")

def testRunCaulking():
    # Execution
    model = DMPLWR(verbose=True)
    model.executeModel("sealant2.pkl",input_type="3dof")


def surfaceRoller():
    # Set up surface
    rigid_reg_file = '/home/mike/Documents/LearningCorrections/data/11-2-21/rigid_02-11-2021-16-19-19.bag'
    surface_file = '/home/mike/Documents/LearningCorrections/data/11-2-21/layup.csv'
    stl_file = '/home/mike/Documents/LearningCorrections/data/11-2-21/layup.STL'
    surfaceFromSTL(surface_file, stl_file, rigid_reg_file, breadboard_offset=[2, 4])

def testLearnRoller():
    demo_location = '/home/mike/Documents/LearningCorrections/data/11-3-21/demos'
    rigid_reg_file = '/home/mike/Documents/LearningCorrections/data/11-3-21/rigid_02-11-2021-16-19-19.bag'

    # Set up surface
    surface_file = '/home/mike/Documents/LearningCorrections/data/11-3-21/layup.csv'

    # Preprocessing
    data = PreProcessing(verbose=True)
    data.set_demonstrations(demo_location)
    data.setRegistrationTransform(None,None,rigid_reg_file)
    data.setTool("roller")
    names, states = data.calculateStates()
    data.plotPos()

    # Learning
    model = DMPLWR(verbose=True, surfacefile=surface_file)
    model.learnModel(names,states,"rollerregions3.pkl")

def plotLearnedRoller():
    model = DMPLWR(verbose=True)
    model.plotLearned("rollerregions3.pkl")
    model.plotLearnedCorrections("rollerregions3.pkl")

def testRunRoller():
    # Execution
    model = DMPLWR(verbose=True, input_tx=np.array([0, 0, -0.7071, 0.7071]))
    model.executeModel("rollerregions3.pkl",input_type="3dof")

    
if __name__ == "__main__":
    # testLearnCaulking()
    # testRunCaulking()
    # surfaceRoller()
    testLearnRoller()
    # testRunRoller()
    # plotLearnedCaulking()
    # plotLearnedRoller()
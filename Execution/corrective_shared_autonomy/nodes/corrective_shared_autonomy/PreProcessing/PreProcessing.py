""" Takes bag files, interpolates data,
 and ultimately converts into state vectors
 that can be used by the learning algorithms

 Last Updated: 08/31/2021
"""

__author__ = "Mike Hagenow"

import os
import numpy as np
import rospy
from core_robotics.bagfile_reader import bagfile_reader
from corrective_shared_autonomy.PreProcessing.preProcessHelpers import preProcessCaulking, preProcessRoller
from corrective_shared_autonomy.PreProcessing.rigid_registration_panda import calculateRR
from core_robotics.transform import transform_coord_frame_from_A, transform_pos_from_A, transform_quat_from_A, transform_vec_from_A

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import matplotlib.pyplot as plt

class PreProcessing():
    def __init__(self, verbose=False):
        self.demo_files = []
        self.demo_bfrs = []
        self.state_names = [] # keeps track of name of states
        self.states = []

        self.regtf = False # whether there is a transform to be applied to the data
        self.pose_topic = None
        self.ft_topic = None
        self.valve_topic = None
        self.verbose = verbose
        
    def set_demonstrations(self,data):
        # Data can either be a directory, a list of bagfiles, or a single bagfile
        # Should be fully qualified
        if type(data) is str and (data.find(".bag")==-1): # Directory
            for root, dirs, files in os.walk(os.path.abspath(data)):
                for filename in files:
                    if filename.endswith(".bag"):
                        self.demo_files.append(os.path.abspath(os.path.join(root, filename))) 

        else: # Bagfile or list of bagfiles
            if type(data) is str:
                self.demo_files = [data] # single bagfile
            else:
                self.demo_files = data # list of bagfiles already!
        if self.verbose:
            print("-------------------")
            print("Preproccesing: Demonstrations Used")
            print("-------------------")
        for demo in self.demo_files:
            print(demo)
            self.demo_bfrs.append(bagfile_reader(demo))
    
    def calculateStates(self):
        # state_names is the name (length m) of each row in the state information
        # states is a list of the states for each demo (d x (mxn)) where n can be different
        # for each demonstration without dynamic time warping
        
        # Get the state information
        if self.tool == "caulking":
            self.addCaulking()
        elif self.tool == "roller":
            self.addRoller()
        else:
            print("Invalid tool. No data collected")

        # check whether to apply rigid transform
        if self.regtf:
            if self.verbose:
                print("register frames")
            # Apply the registration
            self.applyRegistrationTransform()
        
        return self.state_names, self.states

    ####################################
    # Functions to transform data      #
    ####################################
    def setRegistrationTransform(self,R,t,regfile):
        # Applies a registration, either from a bagfile or from a set transform (R and t)
        self.regtf = True
        if regfile is None:
            # Provided with a transform
            self.RegA = np.eye(4)
            self.RegA[0:3,0:3] = R
            self.RegA[0:3,3] = t.reshape((3,))
        else:
            A_mocap_panda, A_breadboard_mocap, A_breadboard_panda = calculateRR(regfile)
            self.RegA = A_mocap_panda

    def setTool(self,tool):
        self.tool = tool

    ################################################
    # Functions for adding loading topic data      #
    ################################################

    def addCaulking(self):
        self.state_names = ['x','y','z','qx','qy','qz','qw','valve']
        for bfr in self.demo_bfrs:
            self.states.append(preProcessCaulking(bfr))

    def addRoller(self):
        self.state_names = ['x','y','z','qx','qy','qz','qw','fx','fy','fz','tx','ty','tz']
        for bfr in self.demo_bfrs:
            self.states.append(preProcessRoller(bfr))

    def applyRegistrationTransform(self):
        # TODO: do the check in list without an exception
        for state_matrx in self.states:
            # apply transform to position
            try:
                pos_ind = self.state_names.index("x")
                pos_temp = transform_pos_from_A(state_matrx[pos_ind:pos_ind+3,:].T, self.RegA)
                state_matrx[pos_ind:pos_ind+3,:] = pos_temp.T
            except Exception as e:
                # print("Exception: ",str(e))
                pass

            # apply transform to orientation
            try:
                quat_ind = self.state_names.index("qx")
                quat_temp = transform_quat_from_A(state_matrx[quat_ind:quat_ind+4,:].T, self.RegA)
                state_matrx[quat_ind:quat_ind+4,:] = quat_temp.T
            except Exception as e:
                # print("Exception: ", str(e))
                pass

            # apply transform to force
            try:
                force_ind = self.state_names.index("fx")
                force_temp = transform_vec_from_A(state_matrx[force_ind:force_ind+3,:].T, self.RegA)
                state_matrx[force_ind:force_ind+3,:] = force_temp.T
            except Exception as e:
                # print("Exception: ", str(e))
                pass

            # apply transform to torque
            try:
                torque_ind = self.state_names.index("tx")
                torque_temp = transform_vec_from_A(self.state_matrx[torque_ind:torque_ind+3,:].T, self.regA)
                state_matrx[torque_ind:torque_ind+3,:] = torque_temp.T
            except Exception as e:
                # print("Exception: ", str(e))
                pass

    def plotPos(self):
        # used to plot the positions from the various demos as a sanity check
        def set_axes_equal(ax):
            # https://stackoverflow.com/questions/1483429/how-to-print-an-exception-in-python
            x_limits = ax.get_xlim3d()
            y_limits = ax.get_ylim3d()
            z_limits = ax.get_zlim3d()

            x_range = abs(x_limits[1] - x_limits[0])
            x_middle = np.mean(x_limits)
            y_range = abs(y_limits[1] - y_limits[0])
            y_middle = np.mean(y_limits)
            z_range = abs(z_limits[1] - z_limits[0])
            z_middle = np.mean(z_limits)

            # The plot bounding box is a sphere in the sense of the infinity
            # norm, hence I call half the max range the plot radius.
            plot_radius = 0.5 * max([x_range, y_range, z_range])

            ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
            ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
            ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

        try:
            fig = plt.figure()
            ax = fig.gca(projection='3d')
            # ax.set_aspect('equal')
            for state_matrx in self.states:
                pos_ind = self.state_names.index("x")
                scat = ax.scatter(state_matrx[pos_ind,:],state_matrx[pos_ind+1,:],state_matrx[pos_ind+2,:])
            set_axes_equal(ax)
            plt.show()
        except Exception as e:
            # print(e)
            pass


def test():
    test = PreProcessing()
    test.set_demonstrations("/home/mike/Documents/LearningCorrections/sealant")
    test.setTool("caulking")
    names, states = test.calculateStates()
    test.plotPos()
    
if __name__ == "__main__":
    test()
    
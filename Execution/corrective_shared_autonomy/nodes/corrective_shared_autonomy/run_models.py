#!/usr/bin/env python3

""" Scripts to run/learn various models

 Last Updated: 09/07/2021
"""

__author__ = "Mike Hagenow"

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
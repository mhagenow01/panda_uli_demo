# Deformation Shared Control
### (Name Pending)
This package contains code supporting the ability to do real time deformations
to a nominal autonomous trajectory. This includes things such as BSplineSurfaces,
Dynamic Movement Primitives (DMPs), etc.

## Compilation:
This package compiles as part of the bash script at the root, `./compile`. This individual package
can be built with `catkin build dmp_deformations`.

### Running a nominal behavior with deformations
In order to use the deformations code, a few setup steps are required. This includes building
the surface models (BSplineSurfaces), generating a nominal trajectory (LFD or hand-build path), and encoding the
final motion as a series of DMPs
0. First, make sure the package has been built using the instructions above. Start roscore in a terminal and for all windows below, make sure to have ros workspace sourced.
1. Create BSplineSurfaces for each of the surface models. For fitting from an stl or point cloud, this can be done
using the 'solidworks_to_bsplinesurface.py' in the nodes directory. For fitting an analytical model geometry, there are
examples of how to build the surface directly in `PyBSpline.py`. For each surface, you will create a csv
of the control pts which is stored in `root/devel/lib/dmp_deformations/[surface name].csv`.
2. Generate a nominal behavior. This can be done by hand using `generate_path_helper.py` in the nodes directory. This can also be done from
recorded demonstrations using the learning from demonstration (LFD) code in `LFD_Hybrid.py`. Futher instructions for how to record demonstrations are found below. Regardless of the method
to generate the nominal behavior, the resulting behavior is encoded as a csv and stored in `root/devel/lib/dmp_deformations/[behavior].csv`.
3. Convert the behavior to a set of DMPs. To do this, you run the node `DMP_learner.py` and then using a separate terminal, publish the name of
the behavior using `rostopic pub /dmp/filepubsegmented std_msgs/String "data: '[behavior].csv'"`. In the window running `DMP_learner.py`, you should see the behavior get processed
and you will see "LOAD AND CALCULATIONS COMPLETE". This process will create `learneddmp.csv` in the `root/devel/lib/dmp_deformations/` directory.
4. The behavior and deformations can either be run in simulation or on the real robot. To run in simulation, launch CoppeliaSim in a new window and open
the robot ttt file from the `sim` directory. To run on the real robot, `rosrun panda_ros pandaRosWrapper`. For the real robot, make sure the force torque sensor is turned on, the panda joints are unlocked (dashboard), and the e-stop is depressed.
5. The deformation controller can be run using either the Falcon or Force Dimension. This process is kicked off using either `./falcondeformationcontroller` or `./fddeformationcontroller` in the `root/devel/lib/dmp_deformations/` directory.

### Learning from Demonstration
TBD

### Registering the optical breadboard and panda
TBD
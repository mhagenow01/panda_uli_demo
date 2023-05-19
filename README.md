# panda_uli_demo
All code related to the NASA ULI Year 3 Demonstration

## Required Packages
For Azure Kinect:
sudo apt install ros-noetic-camera-info-manager
sudo apt install k4a-tools

For Franka ROS:
sudo apt install ros-noetic-gazebo-dev
sudo apt install ros-noetic-gazebo-ros-control
(can also remove franka_gazebo instead of installing these packages as it is not used by the project)

For Franka ROS Interface:
Depending on your Eigen installation, you may need a symbolic link
between eigen3 and Eigen: https://stackoverflow.com/questions/23284473/fatal-error-eigen-dense-no-such-file-or-directory

For Actuation:
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

For Execution:
sudo apt install ros-noetic-pcl-conversions
sudo apt install ros-noetic-pcl-ros
pip3 install dtw-python
pip3 install plotly
DHDC (force dimension)

STEPS for Running run_models.py:
2. Install rust (so cargo build --release would work)
3. Put libdhd.so.3.8.0 in /usr/local/lib
4. Install Pico Tree from source (pip may be fine too) https://github.com/Jaybro/pico_tree
    (Anywhere is fine, but I installed in documents/lib_installs.) Note that the build command with --target docs won't compile
    without doxygen. You'll need to use sudo when executing the install command.
5. Install nlopt: https://github.com/stevengj/nlopt
6. If eigen is causing issues due to having the wrong version of eigen, just ignore that
7. Install nvm: https://github.com/nvm-sh/nvm#installing-and-updating
8. nvm install node (this install npm)
9. In panda_uli_demo/Interface/react: `npm install`
10. `npm start` , but if it doesnt work and you get an error about an envelope, see 
        this: https://stackoverflow.com/questions/69692842/error-message-error0308010cdigital-envelope-routinesunsupported
        I followed this suggestion and it fixed the error: export NODE_OPTIONS=--openssl-legacy-provider
        Now when you do npm start, it will take a while but eventually it loads, even if it suggests
        installing something
11. `sudo apt-get install ros-noetic-rosbridge-server `
12. `sudo apt-get install ros-noetic-tf2-web-republisher`
13. `pip install pynput`
14. `pip install dtw`
15. `pip install plotly`
16. For the frontend, change the lan IP address here: /home/nitzan/Code/panda_ws/src/panda_uli_demo/Interface/react/src/RosStore.js
17. `pip install open3d`
18. `pip install trimesh`
19. `sudo apt install spacenavd`
20. `sudo apt install ros-noetic-spacenav-node`
25. To use the C++ verison of the registration:
    * Get eigen 4.3 (branch 4.3) here: https://gitlab.com/libeigen/eigen
    * /home/nitzan/Code/panda_ws/src/panda_uli_demo/Execution/affordance_corrections/nodes/affordance_corrections/affordance_helpers/cppfitting/CMakeLists.txt
    Needs to have: find_package(Eigen3 EXACT 3.4.0 REQUIRED PATHS /home/nitzan/Documents/lib_installs/eigen/)

To compile, run `./compile` from the directory it's inside of.


Know-Hows:
150. To change whether registration happens with C++ (fast) or Python (slow) go here: /home/nitzan/Code/panda_ws/src/panda_uli_demo/Execution/affordance_corrections/nodes/affordance_corrections/ros_uli_wrapper.py
    and change rosaff.setCppFitting(False)
151. To choose the stl model to fit to: in middleend.launch
152. To choose the 3D device to use: also in middleend.launch
153. In rviz, after running scan, Press C to do registration (aka 'publish point' button in rviz)







#### CITATIONS
DTW module. When using in academic works please cite:
  T. Giorgino. Computing and Visualizing Dynamic Time Warping Alignments in R: The dtw Package.
  J. Stat. Soft., doi:10.18637/jss.v031.i07.



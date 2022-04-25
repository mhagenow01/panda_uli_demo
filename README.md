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
depending on your Eigen installation, you may need a symbolic link
between eigen3 and Eigen: https://stackoverflow.com/questions/23284473/fatal-error-eigen-dense-no-such-file-or-directory

For Actuation:
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh


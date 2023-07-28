# Human-Robot Collaborative Sanding Using End-User Programming and Shared Autonomy
This package includes all code related to the NASA ULI Year 3 Demonstration of collaborative sanding. This repo includes code as well as example simulations to experience the developed technologies. This code has been tested on ROS Noetic. 

[![Examples of System and Workflows](http://img.youtube.com/vi/2Z__I4u5WaU/0.jpg)](http://www.youtube.com/watch?v=2Z__I4u5WaU "Human-in-the-loop Sanding System")


## Required Packages and Installation
**For Azure Kinect:**
```
sudo apt install ros-noetic-camera-info-manager
sudo apt install k4a-tools
```

**For Franka ROS:**
```
sudo apt install ros-noetic-gazebo-dev
sudo apt install ros-noetic-gazebo-ros-control
```

can either be installed from src or through binaries:
binaries: `sudo apt install ros-noetic-libfranka ros-noetic-franka-ros`
from source: `https://github.com/frankaemika/franka_ros`
if you need a specific version to match with libfranka for the robot hardware, it is probably better to install from source
Note: if installing from source, you can also remove franka_gazebo instead of installing these packages as it is not used by the project (and often causes compilation issues)

**For Franka ROS Interface:**
Depending on your Eigen installation, you may need a symbolic link
between eigen3 and Eigen: https://stackoverflow.com/questions/23284473/fatal-error-eigen-dense-no-such-file-or-directory

**For Actuation:**
```
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
````

**For Execution**:
```
sudo apt install ros-noetic-pcl-conversions
sudo apt install ros-noetic-pcl-ros
pip3 install dtw-python
pip3 install plotly
DHDC (force dimension)
```


## Compilation:
1. Place panda_uli_demo repo into src folder of a catkin workspace
2. Make the `compile` script executable: `chmod +x compile`
3. `./compile`

## Running the code
The system is designed to run across multiple computers using the ROS Master framework. However, it can also be run from a single computer. Here are the required terminal commands:
1. `roslaunch uli_config robot.launch` *run the actual robot hardware including the kinect camera, valve, and inverse kinematics*
2. `roslaunch uli_config middleend.launch` *runs the mapping, motion, and behavior generation code*
3. `npm start` from /demo/src/panda_uli_demo/Interface/react to launch the interface. It will launch on the browser. You can also configure the computer to allow for external access through a phone or other device.
4. May also need to run `rosrun corrective_shared_autonomy kdlik` to avoid blocking during the reachability checking. It should run under one of the other launch files but doesn't seem to always catch the messages.

Notes related to external access:
Example of UFW rules for setting up endpoint for phone to access react~

```
mike@mike-XPS-15-9570:~/Documents/demo/src/panda_uli_demo$ sudo ufw status

Status: activeTo                         Action      From

3000                       ALLOW       192.168.3.3
22/tcp                     ALLOW       Anywhere
3000                       ALLOW       192.168.3.8
11311                      ALLOW       192.168.3.3
9090                       ALLOW       192.168.3.3
9090                       ALLOW       192.168.3.8
80                         ALLOW       192.168.3.3
Anywhere                   ALLOW       192.168.3.3
22/tcp (v6)                ALLOW       Anywhere (v6)
```

Need to run (for external access on the same network):
- sudo ufw enable
- sudo ufw allow ssh


## CITATIONS

#### Project Related
Senft, Emmanuel, Michael Hagenow, Kevin Welsh, Robert Radwin, Michael Zinn, Michael Gleicher, and Bilge Mutlu. "Task-level authoring for remote robot teleoperation." Frontiers in Robotics and AI 8 (2021): 707149.

Hagenow, Michael, Emmanuel Senft, Robert Radwin, Michael Gleicher, Bilge Mutlu, and Michael Zinn. "Corrective shared autonomy for addressing task variability." IEEE robotics and automation letters 6, no. 2 (2021): 3720-3727.

Hagenow, Michael, Emmanuel Senft, Robert Radwin, Michael Gleicher, Bilge Mutlu, and Michael Zinn. "Informing real-time corrections in corrective shared autonomy through expert demonstrations." IEEE Robotics and Automation Letters 6, no. 4 (2021): 6442-6449.

Doshi, Megh Vipul, Michael Hagenow, Robert Radwin, Michael Gleicher, Bilge Mutlu, and Michael Zinn. "Handheld Haptic Device with Coupled Bidirectional Input." arXiv preprint arXiv:2305.19381 (2023).

#### External
DTW module. When using in academic works please cite:
  T. Giorgino. Computing and Visualizing Dynamic Time Warping Alignments in R: The dtw Package.
  J. Stat. Soft., doi:10.18637/jss.v031.i07.



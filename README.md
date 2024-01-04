# Human-Robot Collaborative Sanding Using End-User Programming and Shared Autonomy
This package includes all code related to the 2024 HRI Systems Paper: *A System for Human-Robot Teaming through End-User
Programming and Shared Autonomy*. If you end up using the code or method, please consider citing our HRI paper.

## Building on the Provided Code
This system was developed for a specific set of on-site demonstrations. As such, much of the code is hardware specific and packages were not necessarily designed for modular reuse with other systems. That being said, we do want to encourage other researchers to build on the ideas developed in this system. If you have any questions or are interested in reusing elements of the system, please contact [Mike Hagenow](mailto:hagenow@mit.edu)  or [Emmanuel Senft](mailto:esenft@idiap.ch). Some of the most interesting elements include:
- [Fragmented Execution](https://github.com/mhagenow01/panda_uli_demo/blob/main/Execution/corrective_shared_autonomy/nodes/corrective_shared_autonomy/TaskModels/FragmentedExecution.py) - used to check reacbability of an encoded behavior and plan based on the subset of the task that is remaining.
- [React Interface for Touchscreen programming](https://github.com/mhagenow01/panda_uli_demo/tree/main/Interface/react) - react application that uses AR/VR for robot programming. Uses ROS to send messages to the backend robot setup. The webserver is hosted on the robot machine and served to a web endpoint on a mobile phone.
- [Hybrid Controller](https://github.com/mhagenow01/panda_uli_demo/blob/main/Actuation/hybrid_controller/nodes/hybrid_controller) - admittance controller for contact with the environment (based on measured forces from an ATI Axia FT sensor)
- [CAD Files](https://drive.google.com/drive/folders/1xeusy0CqQZtPNDfgEdTCZORx9_Zy60sm?usp=sharing) - assets for custom robot additions, including the Kinect Azure mount (attached to FT sensor), and the sanding end-effector.



[![Examples of System and Workflows](http://img.youtube.com/vi/2Z__I4u5WaU/0.jpg)](http://www.youtube.com/watch?v=2Z__I4u5WaU "Human-in-the-loop Sanding System")


## Required Packages and Installation
This repo includes code and CAD files for the developed system. This code has been tested on ROS Noetic. 
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

#### Project Related Publications
Senft, Emmanuel, Michael Hagenow, Kevin Welsh, Robert Radwin, Michael Zinn, Michael Gleicher, and Bilge Mutlu. "Task-level authoring for remote robot teleoperation." Frontiers in Robotics and AI 8 (2021): 707149.

Hagenow, Michael, Emmanuel Senft, Robert Radwin, Michael Gleicher, Bilge Mutlu, and Michael Zinn. "Corrective shared autonomy for addressing task variability." IEEE robotics and automation letters 6, no. 2 (2021): 3720-3727.

Hagenow, Michael, Emmanuel Senft, Robert Radwin, Michael Gleicher, Bilge Mutlu, and Michael Zinn. "Informing real-time corrections in corrective shared autonomy through expert demonstrations." IEEE Robotics and Automation Letters 6, no. 4 (2021): 6442-6449.

Doshi, Megh Vipul, Michael Hagenow, Robert Radwin, Michael Gleicher, Bilge Mutlu, and Michael Zinn. "Handheld Haptic Device with Coupled Bidirectional Input." arXiv preprint arXiv:2305.19381 (2023).

Hagenow, Michael, Emmanuel Senft, Evan Laske, Kimberly Hambuchen, Terrence Fong, Robert Radwin, Michael Gleicher, Bilge Mutlu, and Michael Zinn. "Registering Articulated Objects With Human-in-the-loop Corrections." In 2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), pp. 2343-2350. IEEE, 2022.

#### External
DTW module. When using in academic works please cite:
  T. Giorgino. Computing and Visualizing Dynamic Time Warping Alignments in R: The dtw Package.
  J. Stat. Soft., doi:10.18637/jss.v031.i07.



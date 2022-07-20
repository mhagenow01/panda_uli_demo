#!/usr/bin/env python

""" Used to send a desired state matrix
    to the relevant ros topics for execution

 Last Updated: 06/10/2022
"""

__author__ = "Emmanuel Senft"

import time
import rospy
import numpy as np
from hybrid_controller.msg import HybridPose
from geometry_msgs.msg import Quaternion, Pose, Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, String, Bool
from franka_core_msgs.msg import RobotState, JointCommand
from std_msgs.msg import Int32
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import tf2_ros

class Resetter:
    def __init__(self):
        self._tfBuffer = tf2_ros.Buffer()
        self._tl = tf2_ros.TransformListener(self._tfBuffer)
        rospy.sleep(.5)
        # Set up ROS Publishers
        # rospy.init_node('executeROSfromState', anonymous=True)

        self.robot_active = True
        self.interrupt = False
        
        self._current_state = None
        self._trajectory = []

        self.last_robot_state = rospy.get_time()

        self.hybrid_pub = rospy.Publisher('/panda/hybrid_pose', HybridPose, queue_size=1)
        #rospy.Subscriber("/franka_ros_interface/custom_franka_state_controller/robot_state", RobotState, self.storePandaStateTime)
        #rospy.Subscriber("/execution/interrupt", String, self.checkInterrupt)
        rospy.Subscriber("/joint_states",JointState,self.on_state)
        #rospy.Subscriber("/franka_ros_interface/custom_franka_state_controller/joint_states",JointState,self.on_state)
        rospy.Subscriber("/commands",String,self.on_command)
        #self._joint_pub = rospy.Publisher("/franka_ros_interface/motion_controller/arm/joint_commands",JointCommand,queue_size=2)
        self._joint_pub = rospy.Publisher("/franka_ros_interface/motion_controller/arm/joint_commands",JointCommand,queue_size=2)
        self._pause_pub = rospy.Publisher("/execution/interrupt", String, queue_size=1)

        time.sleep(0.5)

    def on_state(self, msg):
        self._current_state = msg

    def on_command(self,msg):
        if msg.data == "reset":
            self.goToReset()

    def robotActive(self):
        # checks whether the robot has sent the state recently. used by task model to know whether to continue
        return self.robot_active

    def isInterrupted(self):
        # checks whether the interrupt has been flagged
        if self.interrupt:
            self.interrupt = False # reset flag
            return True
        else:
            return False

    def checkInterrupt(self,data):
        # check whether to pause the current execution on the next loop
        if data.data=="pause":
            self.interrupt = True

    def publishToRobot(self,hpose):
        # print("-------- ROBOT COMMAND -------")
        # print("pos: ",hpose.pose.position.x,hpose.pose.position.y,hpose.pose.position.z)
        # print("quat: ",hpose.pose.orientation.x,hpose.pose.orientation.y,hpose.pose.orientation.z, hpose.pose.orientation.w)
        # print("force: ",hpose.wrench.force.x,hpose.wrench.force.y,hpose.wrench.force.z)
        # print("CF: ",hpose.constraint_frame.x,hpose.constraint_frame.y,hpose.constraint_frame.z, hpose.constraint_frame.w)
        self.hybrid_pub.publish(hpose)

    def linearInterpolation(self,start,end,val):
        return start + (end-start)*val

    def goToReset(self):
        self._pause_pub.publish("pause")
        try:
            # get desired pose

            g = Pose()
            g.position.x = .3
            g.position.y = 0
            g.position.z = .5
            g.orientation.x = 0
            g.orientation.y = 0
            g.orientation.z = 0
            g.orientation.w = 1.

            # convert to position control
            hpose = HybridPose()
            hpose.underconstrained.data = False
            hpose.constraint_frame.x = 0.0
            hpose.constraint_frame.y = 0.0
            hpose.constraint_frame.z = 0.0
            hpose.constraint_frame.w = 1.0
            hpose.sel_vector = [1,1,1,1,1,1]

            # Get current pose from TF2
            # listener.waitForTransform('panda_link0', 'panda_ee', rospy.Time(), rospy.Duration(2.0))
            trans = self._tfBuffer.lookup_transform("panda_link0", "panda_ee", rospy.Time(), rospy.Duration(1.0))
            x = trans.transform.translation.x; y = trans.transform.translation.y; z = trans.transform.translation.z
            qx = trans.transform.rotation.x; qy = trans.transform.rotation.y; qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w

        except Exception as e:
            print(str(e))
            return

        # Set up SLERP for interpolation
        key_rots = R.from_quat(np.array([[qx,qy,qz,qw],[g.orientation.x,g.orientation.y,g.orientation.z,g.orientation.w]]))
        key_times = [0,1]
        slerp = Slerp(key_times, key_rots)


        # max cartesian speed - 0.1
        cart_dist = np.linalg.norm(np.array([x,y,z]-np.array([g.position.x, g.position.y, g.position.z])))
        num_samples_cart = cart_dist / .3

        # max angular speed - 10 degrees per second = 0.1745
        rot_between = key_rots[1].inv() * key_rots[0]
        ang_dist = np.linalg.norm(rot_between.as_rotvec())
        num_samples_ang = ang_dist / 1.

        # 100 samples per second
        num_interp_samples = int(100*np.max([num_samples_cart, num_samples_ang]))
        self._trajectory = []
        for jj in range(0,num_interp_samples):
            if not self.robotActive():
                return
            hpose.pose.position.x = self.linearInterpolation(x, g.position.x, jj / num_interp_samples)
            hpose.pose.position.y = self.linearInterpolation(y, g.position.y, jj / num_interp_samples)
            hpose.pose.position.z = self.linearInterpolation(z, g.position.z, jj / num_interp_samples)
            hpose.pose.orientation.x = slerp(jj / num_interp_samples).as_quat()[0]
            hpose.pose.orientation.y = slerp(jj / num_interp_samples).as_quat()[1]
            hpose.pose.orientation.z = slerp(jj / num_interp_samples).as_quat()[2]
            hpose.pose.orientation.w = slerp(jj / num_interp_samples).as_quat()[3]
            self.publishToRobot(hpose)
            time.sleep(0.01)

if __name__ == "__main__":
    rospy.init_node('resetter', anonymous=True)
    resetter = Resetter()
    rospy.spin()

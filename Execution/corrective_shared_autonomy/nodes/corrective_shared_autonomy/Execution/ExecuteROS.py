#!/usr/bin/env python3

""" Used to send a desired state matrix
    to the relevant ros topics for execution

 Last Updated: 09/27/2021
"""

__author__ = "Mike Hagenow"

import time
import rospy
import numpy as np
from hybrid_controller.msg import HybridPose
from geometry_msgs.msg import Quaternion, Pose, Vector3
from std_msgs.msg import Float64, Float64MultiArray
from std_msgs.msg import Int32
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

class ExecuteROS:
    def __init__(self,input_tx = np.array([0, 0, 0, 1]),task_R = np.array([0, 0, 0, 1]), task_t = np.zeros((3,))):
        # Set up ROS Publishers
        # rospy.init_node('executeROSfromState', anonymous=True)
        self.hybrid_pub = rospy.Publisher('/panda/hybrid_pose', HybridPose, queue_size=1)
        self.valve_pub = rospy.Publisher('/valvestate',Int32,queue_size=1)
        self.correction_pub = rospy.Publisher('/csacorrection',Float64MultiArray,queue_size=1)
        self.input = np.zeros((3,))
        self.input_button = 0.0
        self.input_tx = input_tx # if the transform should be further rotated

        self.task_R = task_R
        self.task_t = task_t

        rospy.Subscriber("/zdinput/input", Vector3, self.storeZDInput)
        rospy.Subscriber("/zdinput/button", Float64, self.storeZDButton)
        time.sleep(0.5)

    def storeZDInput(self, data):
        R_temp = R.from_quat(self.input_tx)
        self.input = R_temp.apply(np.array([data.x, data.y, data.z]))

    def storeZDButton(self,data):
        self.input_button = data.data

    def getZDInput(self):
        return self.input, self.input_button

    def publishToRobot(self,hpose):
        # print("-------- ROBOT COMMAND -------")
        # print("pos: ",hpose.pose.position.x,hpose.pose.position.y,hpose.pose.position.z)
        # print("quat: ",hpose.pose.orientation.x,hpose.pose.orientation.y,hpose.pose.orientation.z, hpose.pose.orientation.w)
        # print("force: ",hpose.wrench.force.x,hpose.wrench.force.y,hpose.wrench.force.z)
        # print("CF: ",hpose.constraint_frame.x,hpose.constraint_frame.y,hpose.constraint_frame.z, hpose.constraint_frame.w)
        self.hybrid_pub.publish(hpose)

    def execute_states(self,state_names,state_vals,surface,correction):
        cor = Float64MultiArray()
        cor.data = correction
        self.correction_pub.publish(cor)

        # Robot command (either there is a constraint frame -- hybrid) or there isn't
        if ("qx" in state_names):
            # add rest of stuff
            hpose = HybridPose()
            hpose.sel_vector = [1, 1, 1, 1, 1, 1]
            constraint_frame = Quaternion()
            constraint_frame.x = 0.0
            constraint_frame.y = 0.0
            constraint_frame.z = 0.0
            constraint_frame.w = 1.0
            hpose.constraint_frame = constraint_frame

            try:
                pos_ind = state_names.index("x")
                quat_ind = state_names.index("qx")

                # Rotate from task frame into robot frame
                R_temp = R.from_quat(self.task_R)
                rotated_pos = R_temp.apply(np.array([state_vals[pos_ind], state_vals[pos_ind+1], state_vals[pos_ind+2]])) + self.task_t

                hpose.pose.position.x = rotated_pos[0]
                hpose.pose.position.y = rotated_pos[1]
                hpose.pose.position.z = rotated_pos[2]
                
                # Rotate rotation from task frame into robot frame
                norm_q = np.array([state_vals[quat_ind], state_vals[quat_ind+1], state_vals[quat_ind+2], state_vals[quat_ind+3]])/np.linalg.norm(np.array([state_vals[quat_ind], state_vals[quat_ind+1], state_vals[quat_ind+2], state_vals[quat_ind+3]]))
                R_orientation = R.from_quat(norm_q)
                rotated_orientation = (R_temp * R_orientation).as_quat()

                hpose.pose.orientation.x = rotated_orientation[0]
                hpose.pose.orientation.y = rotated_orientation[1]
                hpose.pose.orientation.z = rotated_orientation[2]
                hpose.pose.orientation.w = rotated_orientation[3]

                self.publishToRobot(hpose)

            except:
                return

        elif ("u" in state_names):
            u = state_vals[state_names.index("u")]
            v = state_vals[state_names.index("v")]
            f = state_vals[state_names.index("f")]
            theta_qx = state_vals[state_names.index("theta_qx")]
            theta_qy = state_vals[state_names.index("theta_qy")]
            theta_qz = state_vals[state_names.index("theta_qz")]
            theta_qw = state_vals[state_names.index("theta_qw")]

            norm_q = np.array([theta_qx, theta_qy, theta_qz, theta_qw])/np.linalg.norm(np.array([theta_qx, theta_qy, theta_qz, theta_qw]))

            # Saturate surface values
            if u > 0.99: u = 0.99
            if u < 0.01: u = 0.01
            if v > 0.99: v = 0.99
            if v < 0.01: v = 0.01
            if f > 25.0: f = 25.0
            if f < -25.0: f = -25.0

            # Convert to robot and command
            r, n_hat, r_u_norm, r_v_norm = surface.calculate_surface_point(u,v)

            # Convert everything from task frame into the robot frame
            R_temp = R.from_quat(self.task_R)
            r = R_temp.apply(r) + self.task_t
            n_hat = R_temp.apply(n_hat)
            r_u_norm = R_temp.apply(r_u_norm)
            r_v_norm = R_temp.apply(r_v_norm)


            # get constraint frame
            constraint_frame_matrx = R.from_matrix(np.hstack([r_u_norm.reshape((3,1)),r_v_norm.reshape((3,1)),n_hat.reshape((3,1))]))
            constraint_frame = constraint_frame_matrx.as_quat()

            # rotate position into constraint frame
            xyz_cf = constraint_frame_matrx.inv().apply(np.array(r))

            hpose = HybridPose()
            hpose.pose.position.x = xyz_cf[0]
            hpose.pose.position.y = xyz_cf[1]
            hpose.pose.position.z = xyz_cf[2]
            hpose.wrench.force.x = 0.0
            hpose.wrench.force.y = 0.0
            hpose.wrench.force.z = f
            hpose.pose.orientation.x = norm_q[0]
            hpose.pose.orientation.y = norm_q[1]
            hpose.pose.orientation.z = norm_q[2]
            hpose.pose.orientation.w = norm_q[3]

            hpose.sel_vector = [1, 1, 0, 1, 1, 1]
            constraint_frame_ros = Quaternion()
            constraint_frame_ros.x = constraint_frame[0]
            constraint_frame_ros.y = constraint_frame[1]
            constraint_frame_ros.z = constraint_frame[2]
            constraint_frame_ros.w = constraint_frame[3]
            hpose.constraint_frame = constraint_frame_ros

            self.publishToRobot(hpose)


        if ("valve" in state_names):
            valve_id = state_names.index("valve")
            int_temp = Int32()
            int_temp.data = int(round(state_vals[valve_id]))
            self.valve_pub.publish(int_temp)

    def linearInterpolation(self,start,end,val):
        return start + (end-start)*val

    def goToReplayStart(self,state_names,start,tfBuffer,listener):
        # TODO: make this more robust (amount of time based on distance -- check that it got there)
        # get desired pose
        hpose = HybridPose()
        hpose.sel_vector = [1, 1, 1, 1, 1, 1]
        constraint_frame = Quaternion()
        constraint_frame.x = 0.0
        constraint_frame.y = 0.0
        constraint_frame.z = 0.0
        constraint_frame.w = 1.0
        hpose.constraint_frame = constraint_frame

        try:
            pos_ind = state_names.index("x")
            quat_ind = state_names.index("qx")

            desired_x = start[pos_ind]; desired_y = start[pos_ind + 1]; desired_z = start[pos_ind + 2]
            desired_qx = start[quat_ind]; desired_qy = start[quat_ind + 1]
            desired_qz = start[quat_ind + 2]; desired_qw = start[quat_ind + 3]

            # convert desired from task frame into robot frame
                        
            R_temp = R.from_quat(self.task_R)
            rotated_pos = R_temp.apply(np.array([desired_x, desired_y, desired_z])) + self.task_t

            desired_x = rotated_pos[0]
            desired_y = rotated_pos[1]
            desired_z = rotated_pos[2]
            
            # Rotate rotation from task frame into robot frame
            norm_q = np.array([desired_qx, desired_qy, desired_qz, desired_qw])/np.linalg.norm(np.array([desired_qx, desired_qy, desired_qz, desired_qw]))
            R_orientation = R.from_quat(norm_q)
            rotated_orientation = (R_temp * R_orientation).as_quat()

            desired_qx = rotated_orientation[0]
            desired_qy = rotated_orientation[1]
            desired_qz = rotated_orientation[2]
            desired_qw = rotated_orientation[3]


            # Get current pose from TF2
            # listener.waitForTransform('panda_link0', 'panda_ee', rospy.Time(), rospy.Duration(2.0))
            trans = tfBuffer.lookup_transform("panda_link0", "panda_gripper", rospy.Time(), rospy.Duration(4.0))
            # trans = tfBuffer.lookup_transform("panda_link0", "panda_ee", rospy.Time(), rospy.Duration(4.0))
            x = trans.transform.translation.x; y = trans.transform.translation.y; z = trans.transform.translation.z
            qx = trans.transform.rotation.x; qy = trans.transform.rotation.y; qz = trans.transform.rotation.z;
            qw = trans.transform.rotation.w

        except Exception as e:
            print(str(e))
            return

        # Set up SLERP for interpolation
        key_rots = R.from_quat(np.array([[qx,qy,qz,qw],[desired_qx,desired_qy,desired_qz,desired_qw]]))
        key_times = [0,1]
        slerp = Slerp(key_times, key_rots)


        # max cartesian speed - 0.1
        cart_dist = np.linalg.norm(np.array([x,y,z]-np.array([desired_x, desired_y, desired_z])))
        num_samples_cart = cart_dist / 0.1

        # max angular speed - 10 degrees per second = 0.1745
        rot_between = key_rots[1].inv() * key_rots[0]
        ang_dist = np.linalg.norm(rot_between.as_rotvec())
        num_samples_ang = ang_dist / 0.1745

        # 100 samples per second
        num_interp_samples = int(100*np.max([num_samples_cart, num_samples_ang]))
        for jj in range(0,num_interp_samples):
            hpose.pose.position.x = self.linearInterpolation(x, desired_x, jj / num_interp_samples)
            hpose.pose.position.y = self.linearInterpolation(y, desired_y, jj / num_interp_samples)
            hpose.pose.position.z = self.linearInterpolation(z, desired_z, jj / num_interp_samples)
            hpose.pose.orientation.x = slerp(jj / num_interp_samples).as_quat()[0]
            hpose.pose.orientation.y = slerp(jj / num_interp_samples).as_quat()[1]
            hpose.pose.orientation.z = slerp(jj / num_interp_samples).as_quat()[2]
            hpose.pose.orientation.w = slerp(jj / num_interp_samples).as_quat()[3]
            self.publishToRobot(hpose)
            time.sleep(0.01)

if __name__ == "__main__":
    testExecuter = ExecuteROS()
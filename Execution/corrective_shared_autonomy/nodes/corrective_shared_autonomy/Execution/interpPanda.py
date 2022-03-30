#!/usr/bin/env python

""" For setting the correct end effector on launch
 Created: 08/12/2020
"""

__author__ = "Mike Hagenow"

import rospy
import time
from std_msgs.msg import String, Float64
from panda_ros_msgs.msg import HybridPose


def main():
    rospy.init_node('interp_panda', anonymous=True)
    first_pose_pub = rospy.Publisher('/panda/hybrid_pose', HybridPose, queue_size=1)

    starting = [0.6, 0.09996, 0.24986]
    end = [0.79784, 0.129, 0.072]

    num_samples = 600
    for ii in range(0,num_samples):
        # send first hybrid pose so it takes effect
        hpose = HybridPose()
        hpose.pose.position.x = starting[0]+(end[0]-starting[0])*ii/num_samples
        hpose.pose.position.y = starting[1]+(end[1]-starting[1])*ii/num_samples
        hpose.pose.position.z = starting[2]+(end[2]-starting[2])*ii/num_samples
        hpose.pose.orientation.x = 0.0
        hpose.pose.orientation.y = 0.0
        hpose.pose.orientation.z = 0.0
        hpose.pose.orientation.w = 1.0
        hpose.sel_vector = [1,1,1,1,1,1]
        hpose.wrench.force.x = 0.0
        hpose.wrench.force.y = 0.0
        hpose.wrench.force.z = 0.0
        hpose.wrench.torque.x = 0.0
        hpose.wrench.torque.y = 0.0
        hpose.wrench.torque.z = 0.0
        hpose.constraint_frame.x = 0.0
        hpose.constraint_frame.y = 0.0
        hpose.constraint_frame.z = 0.0
        hpose.constraint_frame.w = 1.0
        first_pose_pub.publish(hpose)
        time.sleep(0.01)


if __name__ == "__main__":
    main()





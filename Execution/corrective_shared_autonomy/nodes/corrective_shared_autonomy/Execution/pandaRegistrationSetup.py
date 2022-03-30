#!/usr/bin/env python

""" For setting the correct end effector on launch
 Created: 08/12/2020
"""

__author__ = "Mike Hagenow"

import rospy
import time
from std_msgs.msg import String, Float64
from panda_ros_msgs.msg import HybridPose
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation
import numpy as np
from threading import Lock
import rosbag
from datetime import datetime

class PandaReg:
    def __init__(self):
        rospy.init_node('panda_mocap_rr')
        self.lock = Lock()
        now = datetime.now()
        self.bagname = "rigid_"+now.strftime("%d-%m-%Y-%H-%M-%S")+".bag"

        self.curr_recording=False

    def tfCallback(self, data):
        if self.curr_recording:
            self.lock.acquire()
            self.activebag.write("/tf", data)
            self.lock.release()

    def runRR(self):
        pub = rospy.Publisher('/panda/hybrid_pose', HybridPose, queue_size=1)
        pub_ee = rospy.Publisher('/panda/set_ee_link', String, queue_size=1)
        rospy.Subscriber("/tf", TFMessage,self.tfCallback, queue_size=1)
        rospy.sleep(0.5)

        # Change the end effector
        strEE = String()
        strEE.data = "pandaMocap"
        pub_ee.publish(strEE)
        rospy.sleep(2.0)

        self.activebag = rosbag.Bag(self.bagname, 'w')
        self.curr_recording=True

        t = 2 + rospy.Time.now().secs + rospy.Time.now().nsecs / 1000000000
        hpose = HybridPose()
        for theta in np.arange(0, 6 * np.pi, 0.01):
            hpose.pose.position.x = 0.03 * np.sin(theta) + 0.04 * np.sin(1.5 * theta) + 0.55
            hpose.pose.position.y = 0.15 * np.sin(theta) + 0.07 * np.sin(2.0 * theta) - 0.10
            hpose.pose.position.z = 0.04 * np.sin(theta) + 0.06 * np.sin(2.5 * theta) + 0.42

            R = Rotation.from_euler('XYZ', [-0.3, 0.0, 0.1 * np.sin(theta) + 0.2])
            # R = Rotation.from_euler('XYZ', [0.1*np.sin(theta), 0.1*np.sin(theta)-0.3, 0.1*np.sin(theta)+0.2])
            q = R.as_quat()
            hpose.pose.orientation.x = q[0]
            hpose.pose.orientation.y = q[1]
            hpose.pose.orientation.z = q[2]
            hpose.pose.orientation.w = q[3]
            pub.publish(hpose)
            time.sleep(0.005)
            t += 0.005

        time.sleep(1)
        self.curr_recording=False
        time.sleep(0.5)
        self.activebag.close()


if __name__ == "__main__":
    rigid = PandaReg()
    rigid.runRR()





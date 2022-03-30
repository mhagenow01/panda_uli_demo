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
    rospy.init_node('ee_link_setter', anonymous=True)
    ee_link_pub = rospy.Publisher('/panda/set_ee_link', String, queue_size=1)
    first_pose_pub = rospy.Publisher('/panda/hybrid_pose', HybridPose, queue_size=1)
    hgain_pub = rospy.Publisher('/panda/hybrid_gain', Float64, queue_size=1)

    time.sleep(3)

    # send new EE link
    temp_str = String()
    temp_str.data = "pandaCaulking"
    ee_link_pub.publish(temp_str)

     # set hybrid gain
    gain = Float64()
    gain.data = 0.0015
    hgain_pub.publish(gain)
    
    time.sleep(0.5)

    # send first hybrid pose so it takes effect
    hpose = HybridPose()
    hpose.pose.position.x = 0.6 # 0.060 more for caulking EE
    hpose.pose.position.y = 0.09996
    hpose.pose.position.z = 0.24986
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


if __name__ == "__main__":
    main()





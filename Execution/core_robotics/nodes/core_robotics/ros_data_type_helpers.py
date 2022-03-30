"""Preprocessing helpers that specifically change the format
of data taken from ROS (among other simple actions)

 Last Updated: 02/21/2019
"""

__author__ = "Guru Subramani and Mike Hagenow"

import numpy as np
from geometry_msgs.msg import PoseStamped,Pose
#from tf import transformations as tf
from core_robotics.quaternion import filter_quaternions, qtoA
# quaternion_inverse
from core_robotics.filters import butter_lowpass_filter

def pose_array_to_np_array(pos_array):
    """
        TODO function documentation

        :param bfr: bagfile with demonstration data
        :param cutoff: break frequency of filter
        :type bfr: bagfile object
        :type cutoff: float
        :returns: outstruct
        :rtype: struct with pose, and FT data
        """
    q_array = np.zeros((len(pos_array),4))
    t_array = np.zeros((len(pos_array),3))
    for id,p in enumerate(pos_array):
        pos = p.pose
        t_array[id] = [pos.position.x,pos.position.y,pos.position.z]
        q_array[id] = [pos.orientation.x,pos.orientation.y,pos.orientation.z,pos.orientation.w]
    return t_array,q_array

def float_to_np_array(float_array):
    """
        TODO function documentation

        :param bfr: bagfile with demonstration data
        :param cutoff: break frequency of filter
        :type bfr: bagfile object
        :type cutoff: float
        :returns: outstruct
        :rtype: struct with pose, and FT data
        """
    out_array = np.zeros((len(float_array),))
    for id,f in enumerate(float_array):
        val = f.data
        out_array[id] = val
    return out_array


def np_array_to_pose_array(t_array,q_array):
    """
        TODO function documentation

        :param bfr: bagfile with demonstration data
        :param cutoff: break frequency of filter
        :type bfr: bagfile object
        :type cutoff: float
        :returns: outstruct
        :rtype: struct with pose, and FT data
        """
    poses = []

    for t,q in zip(t_array,q_array):
        pos = Pose()
        pos.orientation.x = q[0]
        pos.orientation.y = q[1]
        pos.orientation.z = q[2]
        pos.orientation.w = q[3]
        pos.position.x = t[0]
        pos.position.y = t[1]
        pos.position.z = t[2]
        poses.append(pos)
    return poses

def fd_to_np_array(ros_Vector3_array):
    """
        TODO function documentation

        :param bfr: bagfile with demonstration data
        :param cutoff: break frequency of filter
        :type bfr: bagfile object
        :type cutoff: float
        :returns: outstruct
        :rtype: struct with pose, and FT data
        """

    __doc__ = "Change a ros Vector3 list to a numpy nx3 array"
    # return np.array([[f.vector.x,f.vector.y,f.vector.z] for f in ros_Vector3_array])
    force_array = np.array([[f.x, f.y, f.z] for f in ros_Vector3_array])
    return force_array

def vec3_to_np_array(ros_Vector3_array):
    """
        TODO function documentation

        :param bfr: bagfile with demonstration data
        :param cutoff: break frequency of filter
        :type bfr: bagfile object
        :type cutoff: float
        :returns: outstruct
        :rtype: struct with pose, and FT data
        """

    __doc__ = "Change a ros Vector3 list to a numpy nx3 array"
    # return np.array([[f.vector.x,f.vector.y,f.vector.z] for f in ros_Vector3_array])
    force_array = np.array([[f.vector.x, f.vector.y, f.vector.z] for f in ros_Vector3_array])
    force_tangent = np.array([np.append(f[:2],[0])for f in force_array])
    force_normal = np.array([[0,0,f[2]] for f in force_array])
    return force_array, force_tangent, force_normal

def wrench_to_np_force_array(ros_Vector3_array,applied = False):
    """
        TODO function documentation

        :param bfr: bagfile with demonstration data
        :param cutoff: break frequency of filter
        :type bfr: bagfile object
        :type cutoff: float
        :returns: outstruct
        :rtype: struct with pose, and FT data
        """

    __doc__ = "Change a ros Vector3 list to a numpy nx3 array"

    force_array = np.array([[f.wrench.force.x, f.wrench.force.y, f.wrench.force.z] for f in ros_Vector3_array])
    force_tangent = np.array([np.append(f[:2],[0])for f in force_array])
    force_normal = np.array([[0,0,f[2]] for f in force_array])
    if applied == True:
        return -force_array, -force_tangent, -force_normal

    return force_array, force_tangent, force_normal

def wrench_to_np_array(ros_wrench_array):
    """
        TODO function documentation

        :param bfr: bagfile with demonstration data
        :param cutoff: break frequency of filter
        :type bfr: bagfile object
        :type cutoff: float
        :returns: outstruct
        :rtype: struct with pose, and FT data
        """

    __doc__ = "Change a ros wrench list to a numpy nx6 array"

    np_array = np.array([[f.wrench.force.x, f.wrench.force.y, f.wrench.force.z,
                             f.wrench.torque.x, f.wrench.torque.y, f.wrench.torque.z] for f in ros_wrench_array])
    return np_array


def rotate_vec(p,f):
    """
        TODO function documentation

        :param bfr: bagfile with demonstration data
        :param cutoff: break frequency of filter
        :type bfr: bagfile object
        :type cutoff: float
        :returns: outstruct
        :rtype: struct with pose, and FT data
        """

    w = p.pose.orientation.w
    x = p.pose.orientation.x
    y = p.pose.orientation.y
    z = p.pose.orientation.z
    q = np.array([x, y, z, w])

    return q_rotate_vec(q,f)

def q_rotate_vec(q,f):
    """
        Pulls tongs data from bagfile, combines the two signals, filters signals, and interpolates all arrays to common length

        :param bfr: bagfile with demonstration data
        :param cutoff: break frequency of filter
        :type bfr: bagfile object
        :type cutoff: float
        :returns: outstruct
        :rtype: struct with pose, and FT data
        """
    A = qtoA(q)
    f_rot = np.dot(A,f)

    return f_rot[:3]


# def inverse_rotate_vec(p):
# TODO Can i get rid of this function
#     w = p.pose.orientation.w
#     x = p.pose.orientation.x
#     y = p.pose.orientation.y
#     z = p.pose.orientation.z
#     q = quaternion_inverse(np.array([[x, y, z, w]]))
#     p_out = PoseStamped()
#     p_out.pose.orientation.w = q[0][3]
#     p_out.pose.orientation.x = q[0][0]
#     p_out.pose.orientation.y = q[0][1]
#     p_out.pose.orientation.z = q[0][2]
#
#     return p_out

def filter_pose_array(pose_array,cutOff = 1.0,fs = 100,order = 6):
    t_array,q_array = pose_array_to_np_array(pose_array)

    q_array_filtered = filter_quaternions(q_array,cutOff = cutOff,fs = fs,order = order)
    t_array_filtered = butter_lowpass_filter(t_array, cutOff, fs, order)
    pose_array_out  = []
    for t,q in zip(t_array_filtered,q_array_filtered):
        p = PoseStamped()
        p.pose.position.x = t[0]
        p.pose.position.y = t[1]
        p.pose.position.z = t[2]

        p.pose.orientation.x = q[0]
        p.pose.orientation.y = q[1]
        p.pose.orientation.z = q[2]
        p.pose.orientation.w = q[3]
        pose_array_out.append(p)

    return pose_array_out


if __name__  == "__main__":
    print(rotate_vec(PoseStamped(), np.array([1,1,1])))

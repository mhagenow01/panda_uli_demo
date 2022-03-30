"""Functions for performing basic quaternion operations

Quaternions of the form [x,y,z,w]

 Last Updated: 02/25/2019
"""

__author__ = "Guru Subramani and Mike Hagenow"

import numpy as np
from copy import deepcopy
from core_robotics.filters import butter_lowpass_filter
from core_robotics.rotation_identities import skew

def quaternion_multiply(q0,q1):
    """
    Multiplies two sets of quaternion time data
    Order q0*q1
    :param q0: first quaternion
    :param q1: second quaterion
    :type q0: time array of quaternion representations
    :type q1: time array of quaternion representations
    :returns:
    :rtype: rotation matrix, translation vector
    """

    #Decompose into 4 time vectors for each quaternion parameter
    x0 = q0[:, 0]
    y0 = q0[:, 1]
    z0 = q0[:, 2]
    w0 = q0[:, 3]

    x1 = q1[:, 0]
    y1 = q1[:, 1]
    z1 = q1[:, 2]
    w1 = q1[:, 3]

    #Form the quaternion product t=q0 * q1
    product =  np.array([
         x1*w0 - y1*z0 + z1*y0 + w1*x0,
         x1*z0 + y1*w0 - z1*x0 + w1*y0,
        -x1*y0 + y1*x0 + z1*w0 + w1*z0,
        -x1*x0 - y1*y0 - z1*z0 + w1*w0], dtype=np.float64)
    product = np.transpose(product)

    #Return normalized since it is just a rotation
    # norm_product=norm_vec(product)
    return product

def quaternion_flip(q_array):
    """
    Checks to see if the quaternion has been flipped (second orientation)
    If so, it will flip it back

    :param q_array
    :type q_array: 1x4 representation of a quaternion
    :returns: ct
    :rtype: 1x4 corrected quaternion
    """

    ct = deepcopy(q_array)

    # flip antipodals
    for ii in range(len(ct) - 1):
        if np.sum((ct[ii + 1] - ct[ii]) ** 2) > \
                np.sum((ct[ii + 1] + ct[ii]) ** 2):
            ct[ii + 1] = -ct[ii + 1]
    return ct

def rotq(axis,angle):
    """
    TODO DOCUMENTATION FOR THIS!!!
    :param axis: axis to rotate (e.g. [1,0,0] or [0,1,0])
    :param angle: rotation around axis (radians)
    :type axis: 3x1 vector
    :type angle: float
    :returns:
    :rtype: rotation matrix, translation vector
            """
    axis = np.array(axis)
    axis = axis.reshape(3)
    e = np.sin(angle / 2.0) * axis
    e0 = np.cos(angle / 2)
    q = np.append(e, [e0])
    return q

def norm_vec(quart_array):
    """
    Normalizes vector
    :param quart_array: vector to be normalized
    :param q1: second quaterion
    :type q0: array of quaternion representations
    :type q1: array of quaternion representations
    :returns: r_est, t_est
    :rtype: rotation matrix, translation vector
    """

    #What does this do?! -> If transpose?!
    if len(np.shape(quart_array)) == 1:
        quart_array = np.array(quart_array)
        quart_array.shape = (1,len(quart_array))

    #Calculate vector magnitude
    mag_quart = np.sqrt(np.sum(quart_array**2,axis = 1))
    mag_quart = np.transpose(mag_quart[np.newaxis])

    mag_quart[mag_quart == 0.0] = 1.0 #Avoid divide by zero

    return quart_array/mag_quart

def filter_quaternions(quart_array,cutOff = 1.0,fs = 100,order = 6):
    """
        TODO DOCUMENTATION FOR THIS!!!
        :param quart_array: vector to be normalized
        :param q1: second quaterion
        :type q0: array of quaternion representations
        :type q1: array of quaternion representations
        :returns: r_est, t_est
        :rtype: rotation matrix, translation vector
        """

    quart_array = norm_vec(quart_array) # ensuring quaternions represent rotations
    exp_coord = log_q(quart_array) # map quaternions to tangent space about 1
    y = butter_lowpass_filter(exp_coord, cutOff, fs, order) # filter in tangent space which is a vector space
    q_filtered = exp_to_quart(y) # convert back to quaternion space
    return  q_filtered

def exp_to_quart(v):
    """
        TODO DOCUMENTATION FOR THIS!!!
        :param quart_array: vector to be normalized
        :param q1: second quaterion
        :type q0: array of quaternion representations
        :type q1: array of quaternion representations
        :returns: r_est, t_est
        :rtype: rotation matrix, translation vector
        """

    angle = np.transpose(np.sqrt(np.sum(v**2,axis = 1))[np.newaxis])
    xyz = np.sinc(angle/np.pi ) * v
    w = np.cos(angle)
    quart = np.append(xyz,w,axis = 1)
    return norm_vec(quart)

def log_q(q):
    """
        TODO DOCUMENTATION FOR THIS!!!
        :param quart_array: vector to be normalized
        :param q1: second quaterion
        :type q0: array of quaternion representations
        :type q1: array of quaternion representations
        :returns: r_est, t_est
        :rtype: rotation matrix, translation vector
        """

    axis,angle = q_to_axis_angle(q)
    return np.transpose(angle[np.newaxis])*axis

def q_to_axis_angle(q):
    """
        TODO DOCUMENTATION FOR THIS!!!
        :param quart_array: vector to be normalized
        :param q1: second quaterion
        :type q0: array of quaternion representations
        :type q1: array of quaternion representations
        :returns: r_est, t_est
        :rtype: rotation matrix, translation vector
        """

    q = np.array(q)
    if len(np.shape(q)) == 1:
        q = np.array(q)
        q.shape = (1,4)

    q[(q[:,3] == 1),3] = 0
    axis = norm_vec(q[:,:3])
    angle = np.arccos(q[:, 3])
    return axis,angle

def quaternion_inverse(q):
    """
    TODO DOCUMENTATION FOR THIS!!!
    :param quart_array: vector to be normalized
    :param q1: second quaterion
    :type q0: array of quaternion representations
    :type q1: array of quaternion representations
    :returns: r_est, t_est
    :rtype: rotation matrix, translation vector
    """
    q_negative = -q[:,:3]
    w = np.transpose([q[:,3]])
    q_negative = np.append(q_negative,w,axis = 1)
    return q_negative

def qtoA(q, type = 'xyzw'):
    if type == 'xyzw':
        e0 = q[3]
        e = q[0:3].reshape(3,1)
    else: # wxyz
        e0 = q[0]
        e = q[1:4].reshape(3,1)

    A = (e0**2-np.matmul(e.transpose(),e)[0][0])*np.eye(3)+2*np.matmul(e,e.transpose())+2*e0*skew(e.reshape(3))
    return A

def get_angular_velocity(q_array,fixed_frame = True):
    """
    Computes angular velocity given quaternions and defaults to global frame
    :param q_array: quaternion numpy array Nx4
    :param fixed_frame : angular velocity in the global frame (True) or body frame (False)
    :type q0: array of quaternion representations
    :type q1: array of quaternion representations
    :returns: r_est, t_est
    :rtype: rotation matrix, translation vector
    """
    q_dot_array=diff0(q_array)
    if fixed_frame:
        omega_array = 2*quaternion_multiply(q_dot_array,quaternion_inverse(q_array))
    else:
        omega_array = 2*quaternion_multiply(quaternion_inverse(q_array),q_dot_array)


    return omega_array[:,0:3]


def get_G(q_single):
    """
    TODO DOCUMENTATION FOR THIS!!!
    :param quart_array: vector to be normalized
    :param q1: second quaterion
    :type q0: array of quaternion representations
    :type q1: array of quaternion representations
    :returns: r_est, t_est
    :rtype: rotation matrix, translation vector
    """
    e0 = q_single[3]
    e1 = np.array(q_single[0])
    e2 = np.array(q_single[1])
    e3 = np.array(q_single[2])

    G = np.array([[e0,e3,-e2,-e1],[-e3,e0,e1,-e2],[e2,-e1,e0,-e3]])
    return G

def diff0(p_array):
    v_array = np.diff(p_array,axis=0)
    zeros = np.zeros((1,np.shape(p_array)[1]))
    v_array = np.concatenate((v_array,zeros),axis = 0)
    return v_array




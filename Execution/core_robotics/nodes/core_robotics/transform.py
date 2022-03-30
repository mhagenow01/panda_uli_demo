"""Rotate data to a different frame (can be used for 3D (p,T,F) or 4D (q))

Quaternions of the form [x,y,z,w]

 Last Updated: 01/21/2021
"""

import numpy as np
from tf.transformations import quaternion_matrix,translation_matrix,quaternion_from_matrix

def transform_coord_frame_from_pq(p_array,q_array,p_trans,q_rot):
    Mrot = quaternion_matrix(q_rot)
    Mtrans = translation_matrix(p_trans)
    Mregister = np.dot(Mtrans,Mrot)
    p_array_transformed = []
    q_array_transformed = []
    for p,q in zip(p_array,q_array):
        goal_trans = np.dot(translation_matrix(p),quaternion_matrix(q))
        goal_new = np.dot(Mregister,goal_trans)
        goal_rot = goal_new.copy()
        goal_rot[:,3] = 0
        goal_rot[3,3] = 1
        goalq = quaternion_from_matrix(goal_rot)
        goalp = goal_new[0:3,3]
        p_array_transformed.append(goalp)
        q_array_transformed.append(goalq)
    p_array_transformed = np.array(p_array_transformed)
    q_array_transformed = np.array(q_array_transformed)
    return p_array_transformed,q_array_transformed

def transform_coord_frame_from_A(p_array,q_array,A):
    Mregister = np.array(A)
    p_array_transformed = []
    q_array_transformed = []
    for p,q in zip(p_array,q_array):
        goal_trans = np.dot(translation_matrix(p),quaternion_matrix(q))
        goal_new = np.dot(Mregister,goal_trans)
        goal_rot = goal_new.copy()
        goal_rot[:,3] = 0
        goal_rot[3,3] = 1
        goalq = quaternion_from_matrix(goal_rot)
        goalp = goal_new[0:3,3]
        p_array_transformed.append(goalp)
        q_array_transformed.append(goalq)
    p_array_transformed = np.array(p_array_transformed)
    q_array_transformed = np.array(q_array_transformed)
    return p_array_transformed,q_array_transformed


def transform_pos_from_A(p_array,A):
    Mregister = np.array(A)
    p_array_transformed = []
    for p in p_array:
        goal_new = np.dot(Mregister, translation_matrix(p))
        goalp = goal_new[0:3, 3]
        p_array_transformed.append(goalp)
    p_array_transformed = np.array(p_array_transformed)
    return p_array_transformed

def transform_quat_from_A(q_array,A):
    Mregister = np.array(A)
    q_array_transformed = []
    for q in q_array:
        goal_new = np.dot(Mregister, quaternion_matrix(q))
        goalq = quaternion_from_matrix(goal_new)
        q_array_transformed.append(goalq)
    q_array_transformed = np.array(q_array_transformed)
    return q_array_transformed

def transform_vec_from_A(v_array,A):
    Mregister = np.array(A)
    v_array_transformed = []
    for v in v_array:
        goal_new = np.matmul(Mregister[0:3,0:3],v.reshape((3,)))
        v_array_transformed.append(goal_new)
    v_array_transformed = np.array(v_array_transformed)
    return v_array_transformed
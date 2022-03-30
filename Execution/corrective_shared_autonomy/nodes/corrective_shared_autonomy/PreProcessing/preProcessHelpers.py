""" Functions that convert the topics
from bagfiles into useful state data

 Last Updated: 08/31/2021
"""

import numpy as np
from core_robotics.ros_data_type_helpers import wrench_to_np_array, rotate_vec,pose_array_to_np_array, q_rotate_vec, float_to_np_array
from scipy import interpolate
from core_robotics.quaternion import quaternion_multiply, quaternion_flip, rotq, filter_quaternions, quaternion_inverse, qtoA
from core_robotics.filters import butter_lowpass_filter

# TODO: slim down this example badly
def preProcessSander(bfr,cutoff = 10.0):
    """
    Pulls sanding data from bagfile, combines the two signals, filters signals, and interpolates all arrays to common length

    :param cutoff: break frequency of filter
    :type bfr: bagfile object
    :type cutoff: float
    :returns: outstruct
    :rtype: struct with pose, and FT data
    """

    #Tongs Topic Configuration
    ftSensor = "/ftmini40"
    sanderMocap = "/vrpn_client_node/ft_tool/pose"
    sandingSpeed = '/sanderSpeed'

    # get forces, torques, positions from bag file
    wrench_v, wrench_t = bfr.get_topic_msgs(ftSensor)
    mocap_v, mocap_t = bfr.get_topic_msgs(sanderMocap)
    sanderSpeed_v, sanderSpeed_t = bfr.get_topic_msgs(sandingSpeed)

    # Sample times constructed by diffing the timestamp datasets
    wrench_sample_time = np.mean(np.diff(wrench_t))*1.0
    mocap_sample_time = np.mean(np.diff(mocap_t))*1.0
    sander_sample_time = np.mean(np.diff(sanderSpeed_t))*1.0

    #convert wrench and poses to NP array and filter wrenches
    wrenches_values = wrench_to_np_array(wrench_v)

    # flip for reaction forces/wrenches
    wrenches_values = -wrenches_values

    p_array, q_array = pose_array_to_np_array(mocap_v)
    sanding_values = float_to_np_array(sanderSpeed_v)
    wrench_values_filt = butter_lowpass_filter(wrenches_values,
                                                   mocap_sample_time/wrench_sample_time,
                                                   1.0/wrench_sample_time, order=4, axis=0)

    # Get lowest capture rate for interpolation
    slowest_rate_index = np.argmax([wrench_sample_time, mocap_sample_time, sander_sample_time])
    timesamples = (wrench_t,mocap_t,sanderSpeed_t)[slowest_rate_index]

    # Interpolate all data to the slowest rate
    wrenches_interp = interpolate.interp1d(wrench_t, wrench_values_filt, axis = 0, fill_value="extrapolate")(timesamples)
    p_array_interp = interpolate.interp1d(mocap_t, p_array, axis=0, fill_value="extrapolate")(timesamples)
    q_array_interp = interpolate.interp1d(mocap_t, q_array, axis=0, fill_value="extrapolate")(timesamples)
    sander_speed_interp = interpolate.interp1d(sanderSpeed_t, sanding_values, axis=0, fill_value="extrapolate")(timesamples)

    #Split data into forces and torques
    calibrated_forces = wrenches_interp[:,:3]
    calibrated_torques = wrenches_interp[:,3:]

    # forces are not aligned with the mocap frame (flipped, then rotated 120 degrees in +Z) -NOTE: depends on how ft is mounted
    q_ft_in_mocap = np.array([0.8660255, -0.50, 0.0, 0.0]) #xyzw quaternion
    calibrated_forces_aligned = np.array([np.matmul(qtoA(q_ft_in_mocap),f) for f in calibrated_forces])
    calibrated_torques_aligned = np.array([np.matmul(qtoA(q_ft_in_mocap), t) for t in calibrated_torques])


    # Transform the force and torques out of the motion capture frame
    calibrated_forces_global = np.array([rotate_vec(p, f) for p, f in zip(mocap_v, calibrated_forces_aligned)])
    calibrated_torques_global = np.array([rotate_vec(p, t) for p, t in zip(mocap_v, calibrated_torques_aligned)])

    ##################################################################################################################################
    # Move everything to the point of interest (force torque sensor and mocap location are not located at the end-effector position) #
    # i.e. need to transform the position and torque (orientation and forces are the same)                                           #
    ##################################################################################################################################
    # Compute the net force and torque as per equation 2.66 in Murray (A Mathematical Introduction to Robotic Manipulation)  #
    # RbcT is wrong - should be Rbc                                                                                          #
    ##########################################################################################################################

    p_local_sander = np.array([0.08012, 0.0, -0.04905]).reshape((3,1))

    p_array_interp_ee = np.array([p+np.matmul(qtoA(q),p_local_sander).flatten() for p,q in zip(p_array_interp,q_array_interp)])

    calibrated_torques_ee = np.array([np.cross((p_ee - p),f).flatten() + t for p, f, t, p_ee in zip(p_array_interp, calibrated_forces_global, calibrated_torques_global, p_array_interp_ee)])

    # TODO: is this a thing?
    # Optitrack has Y-axis as the identity so all matrices are rotated by positive pi/2 about x
    # q_offset = np.array([rotq([1,0,0],np.pi/2)])
    # q_offset = np.repeat(q_offset,len(q_array_center),axis = 0)
    # q_array_center = quaternion_multiply(q_array_center,q_offset)

    # filtering position and rotation data
    p_array_filt = butter_lowpass_filter(p_array_interp_ee,cutoff,1.0/mocap_sample_time,axis = 0)
    q_array_filt = filter_quaternions(q_array_interp,cutOff=cutoff,fs=1.0/mocap_sample_time)

    #Rotate torques into the local frame -> for formulation in derive constraint equations (required)
    # calibrated_torques_local = np.array([np.matmul(np.transpose(qtoA(q)),t) for q,t in zip(q_array_center,calibrated_torques_sum)])

    w_array_center = get_angular_velocity(q_array_interp)
    w_array_center_local = get_angular_velocity(q_array_interp,fixed_frame = False)
    v_array_center = diff0(p_array_filt)

    #Return data to workspace
    out_struct = {}
    out_struct["timestamps"] = timesamples
    out_struct["f"] = calibrated_forces_global
    out_struct["f_local"] = calibrated_forces
    # out_struct["t_local"] = calibrated_torques_local # in local frame
    out_struct["t"] = calibrated_torques_ee # in global frame

    out_struct["p"] = p_array_filt# position of sander
    out_struct["q"] = q_array_filt # orientation of sander

    out_struct["sandingspeed"] = sander_speed_interp

    out_struct["w_global"] = w_array_center # angular velocity global frame
    out_struct["w_local"] = w_array_center_local # angular velocity local frame
    out_struct["v"] = v_array_center # linear velocity

    # Adding labels if they exist in bagfile
    if '/labels' in bfr.topics:
        labels, label_timestamps = bfr.get_topic_msgs('/labels')
        labels = np.array([label.data for label in labels], dtype='S100')
        still_on = (labels == 'constraint').astype(float)
        labels_interp = interpolate.interp1d(label_timestamps, still_on, kind='nearest', fill_value='extrapolate')(
            timesamples)
        labels_interp = np.squeeze([labels_interp > 0.5]).astype(bool)
        out_struct['labels'] = labels_interp
    return out_struct


# TODO: slim down this example
def preProcessCaulking(bfr,cutoff = 10.0):

    # Relevant Topics for State Information
    mocap_topic = "/vrpn_client_node/sealant_gun/pose"
    valve_topic = '/valvestate'

    # get data from bag file
    mocap, mocap_timestamps = bfr.get_topic_msgs(mocap_topic)
    valve, valve_timestamps = bfr.get_topic_msgs(valve_topic)

    # Sample times constructed by diffing the timestamp datasets
    mocap_sample_time = np.mean(np.diff(mocap_timestamps))*1.0
    valve_sample_time = np.mean(np.diff(valve_timestamps))*1.0

    # Convert Data to Numpy Arrays
    p_array, q_array = pose_array_to_np_array(mocap)
    valve_array = float_to_np_array(valve)

    # Get lowest capture rate for interpolation
    slowest_rate_index = np.argmax([mocap_sample_time,valve_sample_time])
    timesamples = (mocap_timestamps,valve_timestamps)[slowest_rate_index]

    # Interpolate all data to the slowest rate
    p_array_interp = interpolate.interp1d(mocap_timestamps, p_array, axis=0, fill_value="extrapolate")(timesamples)
    q_array_interp = interpolate.interp1d(mocap_timestamps, q_array, axis=0, fill_value="extrapolate")(timesamples)
    valve_array_interp = interpolate.interp1d(valve_timestamps, valve_array, axis=0, fill_value="extrapolate")(timesamples)

    # filtering position and rotation data
    p_array_filt = butter_lowpass_filter(p_array_interp,cutoff,1.0/mocap_sample_time,axis = 0)
    q_array_filt = filter_quaternions(q_array_interp,cutOff=cutoff,fs=1.0/mocap_sample_time)

    #Return data to workspace
    data_out = np.zeros((8,len(timesamples)))
    data_out[0:3,:]=p_array_filt.T
    data_out[3:7,:]=q_array_filt.T
    data_out[7,:] = valve_array_interp.T

    return data_out

def preProcessRoller(bfr,cutoff = 10.0):
    #Tongs Topic Configuration
    ftSensor = "/ftmini40"
    sanderMocap = "/vrpn_client_node/roller/pose"

    # get forces, torques, positions from bag file
    wrench_v, wrench_t = bfr.get_topic_msgs(ftSensor)
    mocap_v, mocap_t = bfr.get_topic_msgs(sanderMocap)

    # Sample times constructed by diffing the timestamp datasets
    wrench_sample_time = np.mean(np.diff(wrench_t))*1.0
    mocap_sample_time = np.mean(np.diff(mocap_t))*1.0

    #convert wrench and poses to NP array and filter wrenches
    wrenches_values = wrench_to_np_array(wrench_v)

    # flip for reaction forces/wrenches
    wrenches_values = -wrenches_values

    p_array, q_array = pose_array_to_np_array(mocap_v)
    wrench_values_filt = butter_lowpass_filter(wrenches_values,
                                                   mocap_sample_time/wrench_sample_time,
                                                   1.0/wrench_sample_time, order=4, axis=0)

    # Get lowest capture rate for interpolation
    slowest_rate_index = np.argmax([wrench_sample_time, mocap_sample_time])
    timesamples = (wrench_t,mocap_t)[slowest_rate_index]

    # Interpolate all data to the slowest rate
    wrenches_interp = interpolate.interp1d(wrench_t, wrench_values_filt, axis = 0, fill_value="extrapolate")(timesamples)
    p_array_interp = interpolate.interp1d(mocap_t, p_array, axis=0, fill_value="extrapolate")(timesamples)
    q_array_interp = interpolate.interp1d(mocap_t, q_array, axis=0, fill_value="extrapolate")(timesamples)

    #Split data into forces and torques
    calibrated_forces = wrenches_interp[:,:3]
    calibrated_torques = wrenches_interp[:,3:]

    # forces are not aligned with the mocap frame (there is a flip and some rotations around z)
    q_ft_in_mocap = np.array([0.9678615, 0.2514838, 0.0, 0.0]) #xyzw quaternion
    calibrated_forces_aligned = np.array([np.matmul(qtoA(q_ft_in_mocap),f) for f in calibrated_forces])
    calibrated_torques_aligned = np.array([np.matmul(qtoA(q_ft_in_mocap), t) for t in calibrated_torques])


    # Transform the force and torques out of the motion capture frame
    calibrated_forces_global = np.array([rotate_vec(p, f) for p, f in zip(mocap_v, calibrated_forces_aligned)])
    calibrated_torques_global = np.array([rotate_vec(p, t) for p, t in zip(mocap_v, calibrated_torques_aligned)])

    ##################################################################################################################################
    # Move everything to the point of interest (force torque sensor and mocap location are not located at the end-effector position) #
    # i.e. need to transform the torque (orientation and forces are the same)                                           #

    p_local_sander = np.array([0.0, 0.0, -0.11588]).reshape((3,1))

    # this is an offset to be rotated for computation of the computed torque -- this is not actually
    # where the end effector is located. p_array_interp is already the ee.
    p_array_interp_ee = np.array(
        [p + np.matmul(qtoA(q), p_local_sander).flatten() for p, q in zip(p_array_interp, q_array_interp)])

    calibrated_torques_ee = np.array([np.cross((p_ee - p),f).flatten() + t for p, f, t, p_ee in zip(p_array_interp, calibrated_forces_global, calibrated_torques_global, p_array_interp_ee)])

    # filtering position and rotation data
    p_array_filt = butter_lowpass_filter(p_array_interp,cutoff,1.0/mocap_sample_time,axis = 0)
    q_array_filt = filter_quaternions(q_array_interp,cutOff=cutoff,fs=1.0/mocap_sample_time)

    # Return data to workspace
    data_out = np.zeros((13, len(timesamples)))
    data_out[0:3, :] = p_array_filt.T
    data_out[3:7, :] = q_array_filt.T
    data_out[7:10, :] = calibrated_forces_global.T
    data_out[10:, :] = calibrated_torques_ee.T
    return data_out
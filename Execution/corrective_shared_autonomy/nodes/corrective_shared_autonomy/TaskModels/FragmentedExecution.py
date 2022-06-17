#!/usr/bin/env python3

""" Take a set of surface trajectories and determine
    the subset that can be executed given the current
    robot pose

 Last Updated: 06/13/2022
"""

__author__ = "Mike Hagenow"

import rospy
import rospkg
import copy
import numpy as np
import time
import pickle
from core_robotics.PyBSpline import BSplineSurface
from corrective_shared_autonomy.srv import IK
from corrective_shared_autonomy.TaskModels.DMPLWRhardcoded import HybridSegment, DMPLWRhardcoded
from scipy.spatial.transform import Rotation as ScipyR
from scipy.spatial.transform import Slerp
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped, Twist
from visualization_msgs.msg import Marker, MarkerArray
from joblib import Parallel, delayed


#########################################################################
# RELATED TO REACHABILITY CHECKS                                        #
#########################################################################

def ang_btwn_quats(q1,q2):
    # assumes xyzw notation
    R_1 = ScipyR.from_quat(q1)
    R_2 = ScipyR.from_quat(q2)
    ang_diff = np.linalg.norm(((R_1.inv()) * R_2).as_rotvec())
    return ang_diff

def queryReachability(pos,quat,urdf,baselink,eelink, pos_tol, quat_tol, jointnames):
    rospy.wait_for_service('/corrective_shared_autonomy/solve_ik')
    try:
        ik_soln = rospy.ServiceProxy('/corrective_shared_autonomy/solve_ik', IK)

        urdf_file = String()
        urdf_file.data = urdf
        base = String()
        base.data = baselink
        ee = String()
        ee.data = eelink

        des_pose = Pose()
        des_pose.orientation.x = quat[0]
        des_pose.orientation.y = quat[1]
        des_pose.orientation.z = quat[2]
        des_pose.orientation.w = quat[3]
        des_pose.position.x = pos[0]
        des_pose.position.y = pos[1]
        des_pose.position.z = pos[2]

        # package jointnames
        jnames = []
        for ii in range(0,len(jointnames)):
            jnames.append(String(jointnames[ii]))


        startt = time.time()
        # print(des_pose)
        resp = ik_soln(urdf_file,base,ee,des_pose,jnames)
        # print("Time: ",time.time()-startt)
        # print(resp)

        # convert response to numpy
        soln_pos = resp.soln_pose.position
        soln_pos_np = np.array([soln_pos.x, soln_pos.y, soln_pos.z])
        soln_quat = resp.soln_pose.orientation
        soln_quat_np = np.array([soln_quat.x, soln_quat.y, soln_quat.z, soln_quat.w])

        pos_error = np.linalg.norm(soln_pos_np-pos)
        quat_error = ang_btwn_quats(soln_quat_np,quat)

        if (pos_error<pos_tol and quat_error<quat_tol):
            return True, np.array(resp.soln_joints.data)
        else: 
            # print("err: ",pos_error, quat_error)
            return False, None           

    except rospy.ServiceException as e:
#         print("Service call failed: %s"%e)
        return False, None

def getContMask(mask):
    # build a new list of trajectories (start and end samples) based on reachable subsets in the mask
    trajs_inds = []
    for ii in range(0,len(mask)):
        num_samps = len(mask[ii])
        active_traj = False
        traj_inds = []
        for jj in range(0,num_samps):
            if not active_traj and mask[ii][jj]==1: # new traj section
                active_traj = True
                start_ind = jj
            elif active_traj and mask[ii][jj]==0: # hit an unreachable
                active_traj = False
                end_ind = jj
                traj_inds.append((start_ind,end_ind))
            elif jj==num_samps-1 and active_traj: # hit end of overall traj
                active_traj = False
                end_ind = jj
                traj_inds.append((start_ind,end_ind))
        trajs_inds.append(traj_inds)

    return trajs_inds

def getIKSingleSolution(pose_trajs,ii,jj,jangles):
    # get a single solution from collision free ik for the purposes of checking path execution
    try:
        ik_soln = rospy.ServiceProxy('/collision_free_ik/solve_ik/', IK)

        des_pose = Pose()
        des_pose.position.x = pose_trajs[ii][0,jj]
        des_pose.position.y = pose_trajs[ii][1,jj]
        des_pose.position.z = pose_trajs[ii][2,jj]
        des_pose.orientation.x = pose_trajs[ii][3,jj]
        des_pose.orientation.y = pose_trajs[ii][4,jj]
        des_pose.orientation.z = pose_trajs[ii][5,jj]
        des_pose.orientation.w = pose_trajs[ii][6,jj]
        pos = np.array([pose_trajs[ii][0,jj], pose_trajs[ii][1,jj], pose_trajs[ii][2,jj]])
        quat = np.array([pose_trajs[ii][3,jj], pose_trajs[ii][4,jj], pose_trajs[ii][5,jj],pose_trajs[ii][6,jj]])

        starting_joints = Float64MultiArray(data=list(jangles))

        # print(des_pose)
        resp = ik_soln(des_pose,starting_joints)

        # convert response to numpy
        soln_pos = resp.soln_pose.position
        soln_pos_np = np.array([soln_pos.x, soln_pos.y, soln_pos.z])
        soln_quat = resp.soln_pose.orientation
        soln_quat_np = np.array([soln_quat.x, soln_quat.y, soln_quat.z, soln_quat.w])

        pos_error = np.linalg.norm(soln_pos_np-pos)
        quat_error = ang_btwn_quats(soln_quat_np,quat)

        return pos_error, quat_error, np.array(resp.soln_joints.data)      

    except rospy.ServiceException as e:
#         print("Service call failed: %s"%e)
        return -1, -1, jangles

def getIKPathsolution(pose_trajs,mask,curr_joints):
    
    trajs_inds = getContMask(mask)
    rospy.wait_for_service('/collision_free_ik/solve_ik/')

    for ii in range(0,len(pose_trajs)):
        for jj in range(0,len(traj_inds[ii])):
            # set up starting point
            jangles = np.copy(curr_joints)
            for kk in range(traj_inds[ii][jj][0],traj_inds[ii][jj][1]):
                pos_error, quat_error, curr_joints = getIKSingleSolution(pose_trajs,ii,kk,curr_joints)


# assumes kdlik service in corrective_shared_autonomy is running
# takes in a (7xnum_poses) array of poses
# output a num_pts array of 0-not reachable, 1-reachable
def checkReachabilityOfPoses(poses, pos_tol=0.002, quat_tol=0.05):
    rospack = rospkg.RosPack()
    configpath = rospack.get_path('uli_config') + '/../Actuation/config/'
    urdf_file = configpath + 'panda.urdf'
    jointnames = ['panda_joint1','panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
    baselink = 'panda_link0'
    eelink = 'panda_orbital'

    num_samps = np.shape(poses)[1]

    reachable = np.zeros((num_samps,))
    
    return Parallel(n_jobs=10, backend='loky')(delayed(checkReachabilityOfPose)(poses[:,i].flatten(),urdf_file,baselink,eelink, pos_tol, quat_tol, jointnames) for i in range(num_samps))


def checkReachabilityOfPose(p,urdf_file,baselink,eelink, pos_tol, quat_tol, jointnames):
    pos = p[0:3]
    quat = p[3:7]
    success, jangles = queryReachability(pos,quat,urdf_file,baselink,eelink, pos_tol, quat_tol, jointnames)
    return success

def checkDoneAndReachability(pos,quat,urdf_file,baselink,eelink, pos_tol, quat_tol, jointnames, curr_mask, ii, jj):
    if curr_mask is None or curr_mask[ii][jj]!=2: # not already done
        success, jangles = queryReachability(pos,quat,urdf_file,baselink,eelink, pos_tol, quat_tol, jointnames)
        if success:
            return 1
    return 0
    
# NOTE: this assumes that the kdlik service is running
def getReachable(trajectories, curr_mask):
    # TODO: based on config
    # TODO: speed -- parallel type stuff http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning#Multi-threaded_Spinning
    rospack = rospkg.RosPack()
    configpath = rospack.get_path('uli_config') + '/../Actuation/config/'
    urdf_file = configpath + 'panda.urdf'
    jointnames = ['panda_joint1','panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
    baselink = 'panda_link0'
    eelink = 'panda_orbital'

    pos_tol = 0.002
    quat_tol = 0.05

    taskmask = []
    for ii in range(0,len(trajectories)):
        trajmask = np.zeros((np.shape(trajectories[ii])[1],))
        taskmask.append(trajmask)

    # call for each element of each trajectory
    for ii in range(0,len(trajectories)):
        # TODO: base this off of distance -- tough to do when it is parallelized
        # TODO: maybe think of a preliminary step as a keyframe extraction
        downsamp = 10
        successes = Parallel(n_jobs=10, backend='loky')(delayed(checkDoneAndReachability)(trajectories[ii][0:3,jj].flatten(), trajectories[ii][3:7,jj].flatten(), urdf_file,baselink,eelink, pos_tol, quat_tol, jointnames,curr_mask, ii,jj) for jj in range(0,np.shape(trajectories[ii])[1],downsamp))
        for jj in range(0,np.shape(trajectories[ii])[1]):
            if successes[int(jj/downsamp)]==1:
                taskmask[ii][jj]=1
        
        # for jj in range(0,np.shape(trajectories[ii])[1]):
        #     pos = trajectories[ii][0:3,jj].flatten()
        #     quat = trajectories[ii][3:7,jj].flatten()
        #     if curr_mask is None or curr_mask[ii][jj]!=2: # not already done
        #         success, jangles = queryReachability(pos,quat,urdf_file,baselink,eelink, pos_tol, quat_tol, jointnames)
        #         if success: # found soln
        #             taskmask[ii][jj]=1

    return taskmask


# Takes in a set of states and converts to the desired robot tool pose
# Assumes the state values include u and v (surface coordinates)
# NOTE: this is similar to the execution code in execute ROS
def getPoseFromState(surface,state_names,state_vals,q_surf = np.array([0, 0, 0, 1]),t_surf = np.zeros((3,))):
    pose_out = np.zeros((7))

    # get pose associated with surface coordinates
    u = state_vals[state_names.index("u")]
    v = state_vals[state_names.index("v")]
    r, n_hat, r_u_norm, r_v_norm = surface.calculate_surface_point(u,v)

    # convert based on surface pose
    R_temp = ScipyR.from_quat(q_surf)
    r = R_temp.apply(r) + t_surf
    n_hat = R_temp.apply(n_hat)
    r_u_norm = R_temp.apply(r_u_norm)
    r_v_norm = R_temp.apply(r_v_norm)

    R_cf = ScipyR.from_matrix(np.hstack([r_u_norm.reshape((3,1)),r_v_norm.reshape((3,1)),n_hat.reshape((3,1))]))

    norm_q_surf = np.array([0, 0, 0, 1])
    if ("theta_qx" in state_names):
        theta_qx = state_vals[state_names.index("theta_qx")]
        theta_qy = state_vals[state_names.index("theta_qy")]
        theta_qz = state_vals[state_names.index("theta_qz")]
        theta_qw = state_vals[state_names.index("theta_qw")]
        norm_q_surf = np.array([theta_qx, theta_qy, theta_qz, theta_qw])/np.linalg.norm(np.array([theta_qx, theta_qy, theta_qz, theta_qw]))
    R_theta = ScipyR.from_quat(norm_q_surf)

    constraint_frame_matrx = ScipyR.from_matrix(np.hstack([r_u_norm.reshape((3,1)),r_v_norm.reshape((3,1)),n_hat.reshape((3,1))]))

    if("tool_offset_x" in state_names):
        to_ind = state_names.index("tool_offset_x")
        tool_offset = np.array([state_vals[to_ind], state_vals[to_ind+1], state_vals[to_ind+2]])
        tool_offset_global = (constraint_frame_matrx * R_theta).apply(tool_offset)
        r[0] -= tool_offset_global[0]
        r[1] -= tool_offset_global[1]
        r[2] -= tool_offset_global[2]

    pose_out[0:3] = r
    pose_out[3:7] = (R_cf * R_theta).as_quat()
    return pose_out

# Takes in a surface on which the behaviors take place (BSplineSurface object)
# Takes in a list of the state names
# Takes in a list of trajs [num_states x num samps]
# Takes in a rotation (q_surf) and translation (t_surf) of the surface model
# Returns a mask of the trajectories that are reachable (list of booleans) [num_samps]
# TODO: min number of continuous samples
def getFragmentedTraj(surface,state_names,trajs,q_surf,t_surf,min_length,curr_mask = None):
    
    num_trajs = len(trajs)

    ##########################################
    # Step 0: convert to poses               #
    ##########################################
    pose_trajs = []
    for ii in range(0,num_trajs):
        num_samps_temp = np.shape(trajs[ii])[1]
        pose_traj = np.zeros((7,num_samps_temp))
        for jj in range(0,num_samps_temp):
            # for each sample, convert to pose
            pose_traj[:,jj] = getPoseFromState(surface,state_names,trajs[ii][:,jj].flatten(),q_surf,t_surf) 
        pose_trajs.append(pose_traj)

    # TODO: downsample if required (also, stride based on culling path length)

    ##########################################
    # Step 1: cull based on reachability     #
    ##########################################
    mask = getReachable(pose_trajs,curr_mask)

    ##########################################
    # Step 1.5: look at pathwise solution    #
    ##########################################
    # curr_joints = np.zeros((7,))
    # ik_pos_errors, ik_ang_errors = getIKPathsolution(pose_trajs,mask,curr_joints)

    ##########################################
    # Step 2: cull based on min length       #
    ##########################################
    # TODO: min length should be incorporated into reachability
    for ii in range(0,num_trajs):
        num_samps_temp = np.shape(trajs[ii])[1]
        active_traj = False
        for jj in range(0,num_samps_temp):
            if not active_traj and mask[ii][jj] == 1: # new traj section
                active_traj = True
                start_ind = jj
            elif (active_traj and mask[ii][jj] == 0) or (active_traj and jj == num_samps_temp-1): # check if long enough
                active_traj = False
                if (jj - start_ind) <= min_length:
                    mask[ii][start_ind:jj] = 0

    # Check whether any new points have been enabled (to prevent trying to learn 0-length behaviors)
    new_pts = False
    for ii in range(0,len(mask)):
        if len(np.where(mask[ii]==1)[0].flatten()) > 0:
            new_pts = True

    return mask, pose_trajs, new_pts

#########################################################################
# RELATED TO TRAJECTORY CONSTRUCTION                                    #
#########################################################################

def interpMultD(starting_vals,ending_vals,num_pts,quat_vars=[], super_pos_vars = [], super_pos_freq=[], super_pos_amp = []):
    vals = []
    
    # LERP ALL VARIABLES
    for ii in range(0,num_pts):
       
        # super is to potentially add a circular motion on top of the LERP motion
        freq_super = 0.0
        amp_super = 0.0
        added_sine = np.zeros((np.shape(starting_vals)))
        for jj in range(0,len(super_pos_vars)):
            amp = super_pos_amp[jj]
            freq = super_pos_freq[jj]
            added_sine[super_pos_vars[jj]] = np.sin(np.pi*float(ii)/num_pts)*amp*np.sin(freq*float(ii)/2*np.pi)
            added_sine[super_pos_vars[jj]+1] = np.sin(np.pi*float(ii)/num_pts)*amp*np.cos(freq*float(ii)/2*np.pi)
        c_i = float(ii)/float(num_pts) # interpolation coefficient
        vals.append(list(c_i*(e-s)+s+a for s,e,a in zip(starting_vals,ending_vals,added_sine)))
    
    vals = np.asarray(vals) # currently,num_samples x num_vars
    
    # SLERP QUATERIONS
    for jj in range(0,len(quat_vars)):
        starting_q = np.array(starting_vals[quat_vars[jj]:quat_vars[jj]+4])
        ending_q = np.array(ending_vals[quat_vars[jj]:quat_vars[jj]+4])
        if num_pts > 1: # slerp can't work on 1 pt
            key_times = [0,num_pts-1]
            key_rots = ScipyR.from_quat(np.concatenate((starting_q.reshape((1,4)),ending_q.reshape((1,4))),axis=0))
            slerper = Slerp(key_times, key_rots)
            
            pts = list(range(0,num_pts))
            slerped = slerper(pts)
            vals[:,quat_vars[jj]:quat_vars[jj]+4] = np.asarray(slerped.as_quat())
    
    return vals.T # num vars x num_samples

def gen_approach(surfaceBSpline,samps_per_sec,R_tool_surf,starting_coords,vel,tool_offset):
    approach_dists = [0.05, 0.005]
    time = np.abs(approach_dists[0]-approach_dists[1]) / vel
    
    segment = HybridSegment()
    segment.hybrid = False
    segment.num_samples = int(time*samps_per_sec) # 2 seconds
    segment.num_samples = max(5,segment.num_samples) # at least 5 samples to avoid crashing dmp
    segment.state_names = ['x','y','z','qx','qy','qz','qw','delta_s','valve','tool_offset_x','tool_offset_y','tool_offset_z']
    segment.corrections.append(np.zeros((len(segment.state_names),segment.num_samples)))

    approach_pt, n_hat, r_u_norm, r_v_norm = surfaceBSpline.calculate_surface_point(starting_coords[0], starting_coords[1])
    R_surf = ScipyR.from_matrix(np.hstack([r_u_norm.reshape((3,1)), r_v_norm.reshape((3,1)), n_hat.reshape((3,1))]))
    q_app = (R_surf * R_tool_surf).as_quat()

    starting = [approach_pt[0], approach_pt[1], approach_pt[2], q_app[0], q_app[1], q_app[2], q_app[3], 1.0, 0.0, 0.0, 0.0, 0.0]
    ending = [approach_pt[0], approach_pt[1], approach_pt[2], q_app[0], q_app[1], q_app[2], q_app[3], 1.0, 1.0, tool_offset[0], tool_offset[1], tool_offset[2]]
    starting[0:3] = starting[0:3] + approach_dists[0] * n_hat
    ending[0:3] = ending[0:3] + approach_dists[1] * n_hat
    
    orig_vals = interpMultD(starting,ending,segment.num_samples,quat_vars=[3])

    segment.original_vals = []
    segment.original_vals.append(orig_vals)
    
    return segment

def gen_retract(surfaceBSpline,samps_per_sec,R_tool_surf,ending_coords,vel, tool_offset):
    retraction_dists = [0.005, 0.05]
    time = np.abs(retraction_dists[0]-retraction_dists[1]) / vel

    segment = HybridSegment()
    segment.hybrid = False
    segment.num_samples = int(time*samps_per_sec) 
    segment.num_samples = max(5,segment.num_samples) # at least 5 samples to avoid crashing dmp
    segment.state_names = ['x','y','z','qx','qy','qz','qw','delta_s','valve','tool_offset_x','tool_offset_y','tool_offset_z']
    segment.original_vals = []
    segment.corrections.append(np.zeros((len(segment.state_names),segment.num_samples)))

    retract_pt, n_hat, r_u_norm, r_v_norm = surfaceBSpline.calculate_surface_point(ending_coords[0], ending_coords[1])
    R_surf = ScipyR.from_matrix(np.hstack([r_u_norm.reshape((3,1)), r_v_norm.reshape((3,1)), n_hat.reshape((3,1))]))
    q_ret = (R_surf * R_tool_surf).as_quat()

    starting = [retract_pt[0], retract_pt[1], retract_pt[2], q_ret[0], q_ret[1], q_ret[2], q_ret[3], 1.0, 1.0, tool_offset[0], tool_offset[1], tool_offset[2]]
    ending = [retract_pt[0], retract_pt[1], retract_pt[2], q_ret[0], q_ret[1], q_ret[2], q_ret[3], 1.0, 0.0, 0.0, 0.0, 0.0]
    starting[0:3] = starting[0:3] + retraction_dists[0] * n_hat
    ending[0:3] = ending[0:3] + retraction_dists[1] * n_hat
    orig_vals = interpMultD(starting,ending,segment.num_samples,quat_vars=[3])
    segment.original_vals.append(orig_vals)
    
    return segment

def gen_btw_passes(surfaceBSpline,samps_per_sec,R_tool_surf_start,R_tool_surf_end,starting_coords,ending_coords,vel):
    retract_pt, n_hat1, r_u_norm, r_v_norm = surfaceBSpline.calculate_surface_point(starting_coords[0], starting_coords[1])
    R_surf = ScipyR.from_matrix(np.hstack([r_u_norm.reshape((3,1)), r_v_norm.reshape((3,1)), n_hat1.reshape((3,1))]))
    q_ret = (R_surf * R_tool_surf_start).as_quat()
    
    approach_pt, n_hat2, r_u_norm, r_v_norm = surfaceBSpline.calculate_surface_point(ending_coords[0], ending_coords[1])
    R_surf = ScipyR.from_matrix(np.hstack([r_u_norm.reshape((3,1)), r_v_norm.reshape((3,1)), n_hat2.reshape((3,1))]))
    q_app = (R_surf * R_tool_surf_end).as_quat()

    time = np.linalg.norm(approach_pt-retract_pt) / vel

    segment = HybridSegment()
    segment.hybrid = False
    segment.num_samples = int(time*samps_per_sec) 
    segment.num_samples = max(5,segment.num_samples) # at least 5 samples to avoid crashing dmp
    segment.state_names = ['x','y','z','qx','qy','qz','qw','delta_s','valve']
    segment.original_vals = []
    segment.corrections.append(np.zeros((len(segment.state_names),segment.num_samples)))

    uvs = interpMultD(starting_coords,ending_coords,segment.num_samples)
    orig_vals = np.zeros((len(segment.state_names),segment.num_samples))
    orig_vals[7,:] = 1.0
    for ii in range(segment.num_samples):
        r, n_hat, r_u_norm, r_v_norm = surfaceBSpline.calculate_surface_point(uvs[0,ii],uvs[1,ii])
        orig_vals[0:3,ii] = r + 0.05 * n_hat 
        R_surf = ScipyR.from_matrix(np.hstack([r_u_norm.reshape((3,1)), r_v_norm.reshape((3,1)), n_hat.reshape((3,1))]))
        orig_vals[3:7,ii] = (R_surf * R_tool_surf_end).as_quat()

    # starting = [retract_pt[0], retract_pt[1], retract_pt[2], q_ret[0], q_ret[1], q_ret[2], q_ret[3], 1.0, 0.0]
    # starting[0:3] = starting[0:3] + 0.05 * n_hat1
    # ending = [approach_pt[0], approach_pt[1], approach_pt[2], q_app[0], q_app[1], q_app[2], q_app[3], 1.0, 0.0]
    # ending[0:3] = ending[0:3] + 0.05 * n_hat2
    # orig_vals = interpMultD(starting,ending,segment.num_samples,quat_vars=[3])
    segment.original_vals.append(orig_vals)
    
    return segment

def constructConnectedTraj(surface,state_names,states,mask,corrections,samps_per_sec):
    # TODO: this should actually also consider reachability
    segments = []
    vel_between = 0.05
    vel_appret = 0.05
    
    trajs_inds = getContMask(mask) # get start and end samples for each continuous segment

    last_uv = None # for keeping track of prev for genbetweenpasses

    for ii in range(0,len(states)):
        for jj in range(0,len(trajs_inds[ii])):
            # Get pass related details
            start_ind_tmp = trajs_inds[ii][jj][0]
            end_ind_tmp = trajs_inds[ii][jj][1]
            pass_state_vals = states[ii][:,start_ind_tmp:end_ind_tmp]
            starting_u = pass_state_vals[state_names.index("u"),0]
            starting_v = pass_state_vals[state_names.index("v"),0]
            ending_u = pass_state_vals[state_names.index("u"),-1]
            ending_v = pass_state_vals[state_names.index("v"),-1]

            R_tool_surf_app = ScipyR.from_quat([0, 0, 0, 1])
            R_tool_surf_ret = ScipyR.from_quat([0, 0, 0, 1])
            if ("theta_qx" in state_names):
                # starting surf orientation
                theta_qx = pass_state_vals[state_names.index("theta_qx"),0]
                theta_qy = pass_state_vals[state_names.index("theta_qy"),0]
                theta_qz = pass_state_vals[state_names.index("theta_qz"),0]
                theta_qw = pass_state_vals[state_names.index("theta_qw"),0]
                # ending surface orientation
                R_tool_surf_app = ScipyR.from_quat([theta_qx, theta_qy, theta_qz, theta_qw])
                theta_qx = pass_state_vals[state_names.index("theta_qx"),-1]
                theta_qy = pass_state_vals[state_names.index("theta_qy"),-1]
                theta_qz = pass_state_vals[state_names.index("theta_qz"),-1]
                theta_qw = pass_state_vals[state_names.index("theta_qw"),-1]
                R_tool_surf_ret = ScipyR.from_quat([theta_qx, theta_qy, theta_qz, theta_qw])
            
            tool_offset_app = np.zeros((3,))
            tool_offset_ret = np.zeros((3,))
            if("tool_offset_x" in state_names):
                to_ind = state_names.index("tool_offset_x")
                tool_offset_app = np.array([pass_state_vals[to_ind,0], pass_state_vals[to_ind,0], pass_state_vals[to_ind,0]])
                tool_offset_ret = np.array([pass_state_vals[to_ind,-1], pass_state_vals[to_ind,-1], pass_state_vals[to_ind,-1]])

            # Generate in between passes if this is not the first overall pass
            if last_uv is not None:
                segmentTemp = gen_btw_passes(surface, samps_per_sec, last_R_tool_surf, R_tool_surf_app,last_uv,[starting_u, starting_v], vel_between)
                segments.append(segmentTemp)

            # Generate approach
            segmentTemp = gen_approach(surface,samps_per_sec,R_tool_surf_app,[starting_u, starting_v], vel_appret, tool_offset_app)
            segments.append(segmentTemp)

            # Generate pass
            segment = HybridSegment()
            segment.hybrid = True
            segment.surface = surface
            segment.num_samples = np.shape(pass_state_vals)[1]
            segment.state_names = state_names
            segment.original_vals = []

            segment.corrections.append(interpMultD(corrections,corrections,segment.num_samples,quat_vars=[3]))
            segment.original_vals.append(pass_state_vals)
            segments.append(segment)

            # Generate retract
            segmentTemp = gen_retract(surface,samps_per_sec,R_tool_surf_ret,[ending_u, ending_v], vel_appret, tool_offset_ret)
            segments.append(segmentTemp)

            last_uv = [ending_u, ending_v] # store previous end of path
            last_R_tool_surf = R_tool_surf_ret

    model = DMPLWRhardcoded(verbose=False, dt=1./samps_per_sec)
    learnedSegments = model.learnModel(segments) # second argument is the outfile
    return learnedSegments

    #############

def circ_marker(index, pos, size, color=[0.0, 1.0, 0.0], frame="map"):
    """ Creates circular markers on points of interest """
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = rospy.Time.now()
    marker.ns = "pts"
    marker.id = index
    marker.type = 2 # sphere
    marker.action = 0 # Add/Modify
    marker.pose.position.x = pos[0]
    marker.pose.position.y = pos[1]
    marker.pose.position.z = pos[2]
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1
    marker.scale.x = size
    marker.scale.y = size
    marker.scale.z = size
    marker.color.a = 0.9
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    return marker

class FragmentedExecutionManager():
    def __init__(self):
        self.taskmask = None
        self.registrationsub = rospy.Subscriber("/registeredObject", PoseStamped, self.computeTask)
        self.executesub = rospy.Subscriber("/executeModel", String, self.executeModel)
        self.correctionsub = rospy.Subscriber("/correction", Twist, self.modelMoved)
        self.rvizpub = rospy.Publisher("rviz_triggers", String, queue_size =1, latch = True)
        self.resumeexec = rospy.Subscriber("/execution/interrupt", String, self.checkResume)
        self.pathmarkerarraypub = rospy.Publisher("/reachabilitymap", MarkerArray, queue_size =1, latch = True)
        self.readtriggers = rospy.Subscriber("rviz_triggers", String, self.readTriggers)
        # TODO: plan -> execute
        self.model_name = ''
        self.min_length = 40
        self.dt = 40
        self.plotting_size = 0.01
        self.max_num_traj_pts = 0
        self.fragmentedBehavior = None
        self.resume = False # resume execution after paused
        
        time.sleep(0.5)
        rospy.spin()
        
    def checkResume(self,data):
        if data.data=='resume':
            self.resume = True

    def readTriggers(self,data):
        if data.data=='deleteactive':
            self.resetExecution()
            self.clearReachability()
        if data.data=='resetexec':
            self.model_name = ''
            self.resetExecution()

    def resetExecution(self):
        self.taskmask = None

    def modelMoved(self,data):
        # when the model is moved, clear reachability and make sure the "compute trajectory" button is renabled
        self.clearReachability()
        self.rvizpub.publish(String("execdone"))

    def executeModel(self,data):
        if self.fragmentedBehavior is not None:
            model = DMPLWRhardcoded(verbose=False, dt=1.0/self.dt)
            doneExecution = False
            segID=0; s=0
            while not doneExecution:
                segID, s = model.executeModel(learnedSegments=self.fragmentedBehavior, R_surface = self.q_surf, t_surface=self.t_surf, input_type='1dof', segID_start=segID, s_start=s)
                if segID==-1 and s==-1:
                    doneExecution = True
                else:
                    while not self.resume:
                        time.sleep(0.5)
                    self.resume = False
            self.rvizpub.publish(String("execdone"))
            new_taskmask = []

            # first time, need to create taskmask
            if self.taskmask is None:
                self.taskmask = []
                for ii in range(0,len(self.tempmask)):
                    self.taskmask.append(np.zeros((np.shape(self.tempmask[ii]))))

            for ii in range(0,len(self.taskmask)):
                new_taskmask.append(np.add(self.taskmask[ii],2*self.tempmask[ii])) # makes 2 for completed

            # zero out tempmask
            newtempmask = []
            for ii in range(0,len(self.tempmask)):
                newtempmask.append(np.zeros((np.shape(self.tempmask[ii]))))
            self.tempmask = newtempmask

            self.taskmask = new_taskmask
            self.fragmentedBehavior = None

            # update visualization
            self.displayReachability(self.trajs)


    def clearReachability(self):
        # need to clear max possible traj length before plotting
        markers = MarkerArray()
        for ii in range(0,self.max_num_traj_pts):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.action = marker.DELETEALL # for some reason, DELETE doesn't work
            markers.markers.append(marker)
        self.pathmarkerarraypub.publish(markers)

    def displayReachability(self,trajs):
        self.clearReachability()
        markerarr = MarkerArray()
        mark_id = 0

        # on first run, taskmask hasn't yet been populated (use tempmask)
        if self.taskmask is None:
            mask_tmp = copy.deepcopy(self.tempmask)
        else:
            mask_tmp = []
            for ii in range(0,len(self.taskmask)):
                mask_tmp.append(np.add(self.taskmask[ii],self.tempmask[ii]))

        for ii in range(0,len(trajs)):
            tmp_num_samps = np.shape(trajs[ii])[1]
            for jj in range(0,tmp_num_samps):
                pos = trajs[ii][0:3,jj]
                if mask_tmp[ii][jj]==0: # not possible
                    markerarr.markers.append(circ_marker(mark_id, pos, self.plotting_size, color=[1.0, 150./255, 138./255], frame="map"))
                elif mask_tmp[ii][jj]==1: # possible
                    markerarr.markers.append(circ_marker(mark_id, pos, self.plotting_size, color=[85./255, 203./255, 205./255], frame="map"))
                elif mask_tmp[ii][jj]==2: # already completed
                    markerarr.markers.append(circ_marker(mark_id, pos, self.plotting_size, color=[128./255, 128./255, 128./255], frame="map"))
                mark_id +=1
        if mark_id > self.max_num_traj_pts:
            self.max_num_traj_pts = mark_id
        
        # publish marker array representing path
        self.pathmarkerarraypub.publish(markerarr)

    def computeTask(self,data):
        # UI: gray out 'compute button'
        model_name = data.header.frame_id
        if model_name != self.model_name:
            self.resetExecution()
            self.model_name = model_name
            rospack = rospkg.RosPack()
            config_dir = rospack.get_path('uli_config')+'/registration_models/'
            self.surfacefile = config_dir + self.model_name + ".csv"
            self.surface = BSplineSurface()
            self.surface.loadSurface(self.surfacefile)
            self.behaviorfile = config_dir + self.model_name + "_frag.pkl"
            self.state_names, self.state_vals, self.corrections = pickle.load(open(self.behaviorfile, "rb"))

        # Current surface location in robot coordinate frame
        quat = data.pose.orientation
        pos = data.pose.position
        self.q_surf = np.array([quat.x, quat.y, quat.z, quat.w])
        self.t_surf = np.array([pos.x, pos.y, pos.z])

        # Compute and display current reachability
        print("getting frag")
        # if self.taskmask is not None:
            # print("-----------------------------")
            # print("TASKMASK")
            # print(self.taskmask)
        self.tempmask, self.trajs, self.new_pts = getFragmentedTraj(self.surface,self.state_names,self.state_vals,self.q_surf,self.t_surf,self.min_length,self.taskmask)
        print("-----------------------------")
        print("TEMPMASK")
        print(np.shape(self.tempmask))
        # print(self.tempmask)
        print("-----------------------------")
        print("disp reach")
        self.displayReachability(self.trajs)
        print("learning behav")
        if self.new_pts:
            self.fragmentedBehavior = constructConnectedTraj(self.surface,self.state_names,self.state_vals,self.tempmask,self.corrections,40.0)
            print("done")
            self.rvizpub.publish(String("computetrajdone"))
        else:
            self.rvizpub.publish(String("execdone"))
        # UI: enabled 'execution' button
            
if __name__ == "__main__":
    rospy.init_node('fragmented_exec', anonymous=True)
    fem = FragmentedExecutionManager()
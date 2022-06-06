""" Take a set of surface trajectories and determine
    the subset that can be executed given the current
    robot pose

 Last Updated: 06/06/2022
"""

__author__ = "Mike Hagenow"

import rospy
import rospkg
import numpy as np
import time
from core_robotics.PyBSpline import BSplineSurface
from corrective_shared_autonomy.srv import IK
from corrective_shared_autonomy.TaskModels.DMPLWRhardcoded import HybridSegment
from scipy.spatial.transform import Rotation as ScipyR
from scipy.spatial.transform import Slerp
from std_msgs.msg import String
from geometry_msgs.msg import Pose

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

#
# NOTE: this presumes that the kdlik service is running
def getReachable(trajectories):
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
        for jj in range(0,np.shape(trajectories[ii])[1]):
            pos = trajectories[ii][0:3,jj].flatten()
            quat = trajectories[ii][3:7,jj].flatten()
            success, jangles = queryReachability(pos,quat,urdf_file,baselink,eelink, pos_tol, quat_tol, jointnames)
            if success:
                taskmask[ii][jj]=1

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
def getFragmentedTraj(surface,state_names,trajs,q_surf,t_surf,min_length):
    
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

    # TODO: downsample if required

    ##########################################
    # Step 1: cull based on reachability     #
    ##########################################
    mask = getReachable(pose_trajs)

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


    return mask, pose_trajs

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

def gen_approach(surfaceBSpline,samps_per_sec,R_tool_surf,starting_coords,time,tool_offset):
    segment = HybridSegment()
    segment.hybrid = False
    segment.num_samples = int(time*samps_per_sec) # 2 seconds
    segment.state_names = ['x','y','z','qx','qy','qz','qw','delta_s','valve','tool_offset_x','tool_offset_y','tool_offset_z']
    segment.corrections.append(np.zeros((len(segment.state_names),segment.num_samples)))

    approach_pt, n_hat, r_u_norm, r_v_norm = surfaceBSpline.calculate_surface_point(starting_coords[0], starting_coords[1])
    R_surf = ScipyR.from_matrix(np.hstack([r_u_norm.reshape((3,1)), r_v_norm.reshape((3,1)), n_hat.reshape((3,1))]))
    q_app = (R_surf * R_tool_surf).as_quat()

    starting = [approach_pt[0], approach_pt[1], approach_pt[2], q_app[0], q_app[1], q_app[2], q_app[3], 1.0, 0.0, 0.0, 0.0, 0.0]
    ending = [approach_pt[0], approach_pt[1], approach_pt[2], q_app[0], q_app[1], q_app[2], q_app[3], 1.0, 1.0, tool_offset[0], tool_offset[1], tool_offset[2]]
    starting[0:3] = starting[0:3] + 0.05 * n_hat
    ending[0:3] = ending[0:3] + 0.005 * n_hat
    
    orig_vals = interpMultD(starting,ending,segment.num_samples,quat_vars=[3])

    segment.original_vals = []
    segment.original_vals.append(orig_vals)
    
    return segment

def gen_retract(surfaceBSpline,samps_per_sec,R_tool_surf,ending_coords,time, tool_offset):
    segment = HybridSegment()
    segment.hybrid = False
    segment.num_samples = int(time*samps_per_sec) # 2 seconds
    segment.state_names = ['x','y','z','qx','qy','qz','qw','delta_s','valve','tool_offset_x','tool_offset_y','tool_offset_z']
    segment.original_vals = []
    segment.corrections.append(np.zeros((len(segment.state_names),segment.num_samples)))

    retract_pt, n_hat, r_u_norm, r_v_norm = surfaceBSpline.calculate_surface_point(ending_coords[0], ending_coords[1])
    R_surf = ScipyR.from_matrix(np.hstack([r_u_norm.reshape((3,1)), r_v_norm.reshape((3,1)), n_hat.reshape((3,1))]))
    q_ret = (R_surf * R_tool_surf).as_quat()
    

    starting = [retract_pt[0], retract_pt[1], retract_pt[2], q_ret[0], q_ret[1], q_ret[2], q_ret[3], 1.0, 1.0, tool_offset[0], tool_offset[1], tool_offset[2]]
    ending = [retract_pt[0], retract_pt[1], retract_pt[2], q_ret[0], q_ret[1], q_ret[2], q_ret[3], 1.0, 0.0, 0.0, 0.0, 0.0]
    starting[0:3] = starting[0:3] + 0.005 * n_hat
    ending[0:3] = ending[0:3] + 0.05 * n_hat
    orig_vals = interpMultD(starting,ending,segment.num_samples,quat_vars=[3])
    segment.original_vals.append(orig_vals)
    
    return segment

def gen_btw_passes(surfaceBSpline,samps_per_sec,R_tool_surf_start,R_tool_surf_end,starting_coords,ending_coords,time):
    # TODO: switch this to follow the surface
    segment = HybridSegment()
    segment.hybrid = False
    segment.num_samples = int(time*samps_per_sec) # 4 seconds
    segment.state_names = ['x','y','z','qx','qy','qz','qw','delta_s','valve']
    segment.original_vals = []
    segment.corrections.append(np.zeros((len(segment.state_names),segment.num_samples)))

    retract_pt, n_hat1, r_u_norm, r_v_norm = surfaceBSpline.calculate_surface_point(starting_coords[0], starting_coords[1])
    R_surf = ScipyR.from_matrix(np.hstack([r_u_norm.reshape((3,1)), r_v_norm.reshape((3,1)), n_hat1.reshape((3,1))]))
    q_ret = (R_surf * R_tool_surf_start).as_quat()
    
    approach_pt, n_hat2, r_u_norm, r_v_norm = surfaceBSpline.calculate_surface_point(ending_coords[0], ending_coords[1])
    R_surf = ScipyR.from_matrix(np.hstack([r_u_norm.reshape((3,1)), r_v_norm.reshape((3,1)), n_hat2.reshape((3,1))]))
    q_app = (R_surf * R_tool_surf_end).as_quat()
    
    starting = [retract_pt[0], retract_pt[1], retract_pt[2], q_ret[0], q_ret[1], q_ret[2], q_ret[3], 1.0, 0.0]
    starting[0:3] = starting[0:3] + 0.05 * n_hat1
    ending = [approach_pt[0], approach_pt[1], approach_pt[2], q_app[0], q_app[1], q_app[2], q_app[3], 1.0, 0.0]
    ending[0:3] = ending[0:3] + 0.05 * n_hat2
    orig_vals = interpMultD(starting,ending,segment.num_samples,quat_vars=[3])
    segment.original_vals.append(orig_vals)
    
    return segment

def constructConnectedTraj(surface,state_names,states,mask,corrections,samps_per_sec):
    # TODO: this should actually also consider reachability
    segments = []
    # TODO: parameterize in some better way
    time_appret = 1.0
    time_between = 2.0

    # build a new list of trajectories based on reachable subsets
    trajs_inds = []
    for ii in range(0,len(states)):
        num_samps = np.shape(states[ii])[1]
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
                segmentTemp = gen_btw_passes(surface, samps_per_sec, last_R_tool_surf, R_tool_surf_app,last_uv,[starting_u, starting_v], time_between)
                segments.append(segmentTemp)

            # Generate approach
            segmentTemp = gen_approach(surface,samps_per_sec,R_tool_surf_app,[starting_u, starting_v], time_appret, tool_offset_app)
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
            segmentTemp = gen_retract(surface,samps_per_sec,R_tool_surf_ret,[ending_u, ending_v], time_appret, tool_offset_ret)
            segments.append(segmentTemp)

            last_uv = [ending_u, ending_v] # store previous end of path
            last_R_tool_surf = R_tool_surf_ret

    return segments

    class FragmentedExecutionManager():
        def __init__(self):
            print("hi")


""" Take a hardcoded, manual trajectory and corrections
    and transform into DMP+LWR

 Last Updated: 03/30/2022
"""

__author__ = "Mike Hagenow"

import numpy as np
import copy
from dtw import dtw
from core_robotics.filters import butter_lowpass_filter
from core_robotics.dataviz import threedplot, highlightedthreedplot
from core_robotics.PyBSpline import convertToUV, BSplineSurface
from corrective_shared_autonomy.TaskModels.utilities.PerSamplePCA import PerSamplePCA
from corrective_shared_autonomy.Execution.ExecuteROS import ExecuteROS
from scipy.spatial.transform import Rotation as R
from scipy import signal
from scipy import interpolate
from std_msgs.msg import String
import rospy
import rospkg
import pickle
import time
import matplotlib
import matplotlib.pyplot as plt
import math

from joblib import Parallel, delayed

import tf2_ros

def interp_matrx(Y,ind):
    return Y[:,math.floor(ind)]+(ind-math.floor(ind))*(Y[:,math.ceil(ind)]-Y[:,math.floor(ind)])

#######################################
# DMP and LWR #
#######################################

class DMP:
    # For a single state variable
    def __init__(self,k,b,dt):
        self.k = k
        self.b = b
        self.dt = dt
        self.num_kernels = 20
        self.kernel_per_overlap = 0.10  # absolute e.g., 5 percent -> 0.05
        self.input = 0.0

    def getPath(self,start, end, forces):
        ''' reconstructs a variable from the forcing function and DMP parameters'''
        x = start
        dx = 0
        x_out = np.zeros((len(forces) + 1))
        x_out[0] = x
        for ii in range(0, len(forces)):
            ddx = self.k * (end - x) - self.b * dx + forces[ii]
            dx = dx + ddx * self.dt
            x = x + dx * self.dt
            x_out[ii + 1] = x
        return x_out

    # Calculate required forcing to produce
    def getDMPForcing(self,x, xf, dt):
        # Finit differencing to get the desired velocity and acceleration to compute the force
        vel = np.gradient(x) / dt
        acc = np.gradient(vel) / dt

        # Calculate the forcing required to produce this motion
        # Using the ODE and solving for f: F = x_ddot-k(x_final-x)+bx_dot
        NLF = acc - self.k * np.array([xf - p for p in x]) + self.b * vel
        return NLF

    # For Locally Weighted Regression
    def radial_kernel(self, x0, all_centers, X, tau):
        temp = np.exp(np.sum((X - x0) ** 2, axis=1) / (-2 * tau * tau))
        sum = 0.0
        for center in all_centers:
            sum += np.exp(np.sum((X - center) ** 2, axis=1) / (-2 * tau * tau))
        return temp / sum

    def getForcing(self,data):
        # Data is expected to a list of equal length arrays
        num_demos = len(data)
        num_samples = len(data[0])
        num_samples_orig = num_samples
        dt = self.dt

        target_samples = 120
        ds_factor = round(num_samples/target_samples)
        ds_factor = max(ds_factor,1)

        dt = dt*ds_factor

        data_new = []
        for ii in range(len(data)):
            data_new.append(data[ii][::ds_factor])
        data = data_new
        num_samples = len(data[0])

        start_sum=0.0
        end_sum=0.0
        for ii in range(0,num_demos):
            start_sum += data[ii][0]
            end_sum += data[ii][-1]
        start = start_sum /float(num_demos)
        end = end_sum / float(num_demos)

        # Determine the placement of kernels
        input_split = np.linspace(0, num_samples, self.num_kernels + 1)
        kernel_centers = np.zeros((self.num_kernels,))
        for ii in range(0, self.num_kernels):
            kernel_centers[ii] = int((input_split[ii] + input_split[ii + 1]) / 2)

        # calculate percentage
        distance_between_kernels = input_split[1] - input_split[0]
        sigma_decay = -np.log(self.kernel_per_overlap)
        kernel_var = (distance_between_kernels / 2.0) / sigma_decay

        # Set up data (concatenate all of the demonstrations)
        f_data_total = np.zeros((0,))
        samples_total = np.zeros((0,))
        for ii in range(0,num_demos):
            f_data = self.getDMPForcing(data[ii], end, dt)
            samples = np.arange(0, len(f_data))

            # canonical system
            s_alpha = 3 / (len(samples) * dt)
            canonical = np.exp(-s_alpha * np.arange(0, len(samples)) * dt)  # decay function

            # normalize required forcing w.r.t. canonical system
            f_data_canonical = np.divide(f_data, canonical)

            # add to total vector for the demo
            f_data_total = np.concatenate((f_data_total,f_data_canonical))
            samples_total = np.concatenate((samples_total,samples))

        all_centers = np.c_[np.ones(len(kernel_centers)), kernel_centers]
        X = np.c_[np.ones(len(samples_total)), samples_total]
        y = f_data_total

        # For each kernel, determine the linear regression
        lin_coeffs = []
        for ii in range(0, self.num_kernels):
            W_temp = np.diag(self.radial_kernel(np.r_[1, kernel_centers[ii]], all_centers, X, kernel_var))
            lin_coeffs.append(np.linalg.pinv(X.T @ W_temp @ X) @ X.T @ W_temp @ y)

        # Calculate the forcing from the regression coefficients
        res = np.zeros((num_samples,))
        samples = np.arange(0, num_samples)
        X_temp = np.c_[np.ones(len(samples)), samples]
        for ii in range(0, self.num_kernels):
            res += np.multiply(self.radial_kernel(np.r_[1, kernel_centers[ii]], all_centers, X_temp, kernel_var),
                               X_temp @ lin_coeffs[ii])

        # Final forcing needs to be re-normalized by the canonical system
        forcing = np.multiply(res,canonical)

        # import matplotlib.pyplot as plt
        # plt.plot(samples,f_data_canonical[0:119])
        # plt.plot(samples,forcing)
        # plt.show()

        forcing = np.repeat(forcing,ds_factor)
        f = interpolate.interp1d(np.linspace(0,1,len(forcing)), forcing)
        forcing = f(np.linspace(0,1,num_samples_orig))

        return start,end,forcing

##############################
# DMP + LWR Learning Model   #
##############################

class HybridSegment:
    def __init__(self):
        self.k = 50
        self.b = 2*np.sqrt(50)
        self.surface=''
        self.num_samples = 0
        self.state_names=[] # list of names e.g., ['x','y','z']
        self.forcing_vals=[]
        self.rev_forcing_vals = []
        self.original_vals = []
        self.corrections = [] # list (one correction per append) with num_state_var by num_samples
        self.start_vals = []
        self.end_vals = []
        self.hybrid=False


def learnDMP(segmentTemp,jj,num_demos,k,b,dt):
    data_temp = []
    data_temp_r = []
    for kk in range(0,num_demos):
        data_temp.append(segmentTemp.original_vals[kk][jj,:])
        data_temp_r.append(np.flip(segmentTemp.original_vals[kk][jj, :]))

    dmptemp = DMP(k,b,dt)
    start,end,forcing = dmptemp.getForcing(data_temp)
    _,_,reverse_forcing = dmptemp.getForcing(data_temp_r)

    # # To see individual DMP plots
    # import matplotlib.pyplot as plt
    # for oo in range(0,num_demos):
    #     plt.plot(data_temp[oo],color='gray')
    # plt.plot(dmptemp.getPath(start, end, forcing),color='red')
    # plt.plot(dmptempr.getPath(end,start,reverse_forcing),color='blue')
    # plt.show()

    return start, end, forcing, reverse_forcing


def parallelSurfPt(surface,u_temp,v_temp):
    r, _, _, _ = surface.calculate_surface_point(u_temp,v_temp)
    return r




class DMPLWRhardcoded:
    def __init__(self,surfacefile='', verbose=False, input_tx=np.array([0, 0, 0, 1]),dt = 0.1):
        # DMP parameters
        self.k = 50
        self.b = 2*np.sqrt(self.k)
        self.dt = dt

        self.surface_file = surfacefile

        self.state_names = [] # keeps track of name of states
        self.states = []

        self.verbose = verbose

        self.input_tx = input_tx

        self.events_pub = rospy.Publisher("/interaction_events",String,queue_size = 1)

        self.create_var_range_dictionary()

    def create_var_range_dictionary(self):
        self.range_per_state = dict()
        self.range_per_state['x'] = 2.0
        self.range_per_state['y'] = 2.0
        self.range_per_state['z'] = 2.0
        self.range_per_state['qx'] = 1.0
        self.range_per_state['qy'] = 1.0
        self.range_per_state['qz'] = 1.0
        self.range_per_state['qw'] = 1.0
        self.range_per_state['u'] = 1.0
        self.range_per_state['v'] = 1.0
        self.range_per_state['f'] = 20.0
        self.range_per_state['theta_qx'] = 1.0
        self.range_per_state['theta_qy'] = 1.0
        self.range_per_state['theta_qz'] = 1.0
        self.range_per_state['theta_qw'] = 1.0
        self.range_per_state['valve'] = 1.0
        self.range_per_state['delta_s'] = 2


    def getTraj(self,learnedSegments,orig=False, demo_id = 0):
        x = []
        y = []
        z = []
        dmptemp = DMP(self.k, self.b, self.dt)
        for segment in learnedSegments:
            startt = time.time()
            if segment.hybrid:
                u_ind = segment.state_names.index("u")
                v_ind = segment.state_names.index("v")

                if orig == False:
                    fu = segment.forcing_vals[u_ind, :]
                    fv = segment.forcing_vals[v_ind, :]
                    u = dmptemp.getPath(segment.start_vals[u_ind], segment.end_vals[u_ind], fu)
                    v = dmptemp.getPath(segment.start_vals[v_ind], segment.end_vals[v_ind], fv)
                else:
                    u = segment.original_vals[demo_id][u_ind, :]
                    v = segment.original_vals[demo_id][v_ind, :]

                # get x,y,z from BSpline
                surface = segment.surface
                backend = 'loky'
                # parallelize retrieval of surface locations
                behaviors = Parallel(n_jobs=12, backend=backend)(delayed(
                parallelSurfPt)(surface,u_temp,v_temp) for u_temp, v_temp in zip(u,v))
                behaviors = np.array(behaviors)
                x.extend(behaviors[:,0])
                y.extend(behaviors[:,1])
                z.extend(behaviors[:,2])

            else:
                if orig == False:
                    fx = segment.forcing_vals[0, :]
                    fy = segment.forcing_vals[1, :]
                    fz = segment.forcing_vals[2, :]
                    x_temp = dmptemp.getPath(segment.start_vals[0], segment.end_vals[0], fx)
                    y_temp = dmptemp.getPath(segment.start_vals[1], segment.end_vals[1], fy)
                    z_temp = dmptemp.getPath(segment.start_vals[2], segment.end_vals[2], fz)
                else:
                    x_temp = segment.original_vals[demo_id][0, :]
                    y_temp = segment.original_vals[demo_id][1, :]
                    z_temp = segment.original_vals[demo_id][2, :]

                x.extend(x_temp)
                y.extend(y_temp)
                z.extend(z_temp)
        
            # print("seg time: ",time.time()-startt)
        return np.array(x), np.array(y), np.array(z)

    def plotModel(self, num_demos, learnedSegments):
        # Plot Kinematics

        # Demos
        xs = []
        ys = []
        zs = []

        for ii in range(0, num_demos):
            x_temp,y_temp,z_temp = self.getTraj(learnedSegments,orig=True, demo_id=ii)
            xs.append(x_temp)
            ys.append(y_temp)
            zs.append(z_temp)

        # Learned
        x,y,z = self.getTraj(learnedSegments,orig=False)

        highlightedthreedplot(x, y, z, xs, ys, zs)

    def learnModel(self,learnedSegments,outfile=''):
        num_demos = len(learnedSegments[0].original_vals)
        start_time = time.time()
        if self.verbose:
            print("-------------------")
            print("Model Learning: DMP + LWR Hardcoded")
            print("-------------------")
        # Overview of Process
        # 0. User provides list of segments
        # 1. Learn DMPs using LWR for each segment (forwards and backwards)
        # 2. Learn Admissible corrections from provided values

       
        ####### 1. DMP + LWR #############

        for ii in range(0,len(learnedSegments)): # note: this is (segments-1) for the real thing from some segmentation-related issue
            segmentTemp = learnedSegments[ii]
            if self.verbose:
                print("DMP for segment ", ii, " of ",(len(learnedSegments) - 1)," with ",segmentTemp.num_samples," samples")

            segmentTemp.start_vals = np.zeros((len(segmentTemp.state_names,)))
            segmentTemp.end_vals = np.zeros((len(segmentTemp.state_names, )))
            segmentTemp.forcing_vals = np.zeros((len(segmentTemp.state_names),segmentTemp.num_samples))
            segmentTemp.rev_forcing_vals = np.zeros((len(segmentTemp.state_names), segmentTemp.num_samples))
            
            backend = 'loky'
            num_states = len(segmentTemp.state_names)
            behaviors = Parallel(n_jobs=8, backend=backend)(delayed(
            learnDMP)(segmentTemp,jj,num_demos,self.k,self.b,self.dt) for jj in range(num_states))

            for jj in range(0,num_states):
                start, end, forcing, reverse_forcing = behaviors[jj]
                segmentTemp.start_vals[jj] = start
                segmentTemp.end_vals[jj] = end
                segmentTemp.forcing_vals[jj,:] = forcing
                segmentTemp.rev_forcing_vals[jj,:] = reverse_forcing


            # for jj in range(0,len(segmentTemp.state_names)):
            #     start, end, forcing, reverse_forcing = learnDMP(segmentTemp,jj,num_demos,self.k,self.b,self.dt)
            #     segmentTemp.start_vals[jj] = start
            #     segmentTemp.end_vals[jj] = end
            #     segmentTemp.forcing_vals[jj,:] = forcing
            #     segmentTemp.rev_forcing_vals[jj,:] = reverse_forcing


        # rospack = rospkg.RosPack()
        # root_dir = rospack.get_path(
        #     'corrective_shared_autonomy') + '/nodes/corrective_shared_autonomy/TaskModels/learnedmodels/'
        # pickle.dump(learnedSegments, open(root_dir + "yoyo.pkl", "wb"), protocol=2)

        # ####### 2. Corrections for provided values #############
        # for segment in learnedSegments:
        #     var_ranges = []
        #     for xx in range(0,len(segment.state_names)):
        #         if segment.state_names[xx] in self.range_per_state.keys():
        #             var_ranges.append(self.range_per_state[segment.state_names[xx]])
        #         else:
        #             var_ranges.append(1.0)
        #     var_ranges = np.array(var_ranges)
        #     segment.corrections = PerSamplePCA(segment.original_vals, var_ranges)

        # Save segments to file for execution
        if outfile != '':
            rospack = rospkg.RosPack()
            pickle.dump(learnedSegments,open(outfile,"wb"), protocol=2)

        if self.verbose:
            print("Time to learn model: %s seconds" % (time.time() - start_time))
            self.plotModel(num_demos, learnedSegments)

        return learnedSegments


    def linearInterpolation(self,start,end,val):
        return start + (end-start)*val

    def getStartValue(self,s_curr,segment):
        # for a partial start, quickly figure out the current state based on the starting point and dmps
        X = segment.start_vals
        dX = np.zeros(np.shape(segment.start_vals))
        ddX = np.zeros(np.shape(segment.start_vals))
        delta_s = 1.0
        for s in range(0,round(s_curr)):
            ddX, dX, X = self.getNewState(segment,ddX,dX,X,delta_s,s)
        return X

    def getNewState(self,segment,ddX,dX,X,delta_s,s):
        length_s = np.shape(segment.forcing_vals)[1]-1
        if delta_s>0: # forwards
            ddX = segment.k * (segment.end_vals - X) - segment.b * dX + interp_matrx(segment.forcing_vals,s)
        else: # backwards
            ddX = segment.k * (segment.start_vals - X) - segment.b * dX + interp_matrx(segment.rev_forcing_vals,length_s-s)
        dX = dX + ddX * self.dt * abs(delta_s)
        X = X + dX * self.dt * abs(delta_s)
        return ddX, dX, X

    def getSaturatedZ(self,X,Y,state_names):
        ''' combine the state and correction and saturate values if required'''
        Z = X+Y

        for var in ['qx','theta_qx']:
            if var in state_names:
                rot_orig = R.from_quat(X[state_names.index(var):state_names.index(var)+4])

                corr = np.copy(Y[state_names.index(var):state_names.index(var)+4])
                if np.linalg.norm(corr)==0.0:
                    corr[3] = 1.0
                corr = corr/np.linalg.norm(corr)

                rot_corr = R.from_quat(corr)
                rot_combined = rot_corr * rot_orig
                quat_combined = rot_combined.as_quat()
                Z[state_names.index(var):state_names.index(var)+4] = quat_combined
        for var in ['u','v']:
            if var in state_names:
                if Z[state_names.index(var)]>0.99:
                    Z[state_names.index(var)] = 0.99
                if Z[state_names.index(var)]<0.01:
                    Z[state_names.index(var)] = 0.01
        return Z

    def getFilteredCorrection(self,ddY, dY, Y, correction, delta_s, statenames):
        k = 50
        b = 2.0*np.sqrt(k)

        for var in ['theta_qx','qx']:
            if var in statenames:
                # convert Y and correction to filter in rotvec space
                Y_R = R.from_quat(Y[statenames.index(var):statenames.index(var)+4])
                Y[statenames.index(var):statenames.index(var)+3] = Y_R.as_rotvec()
                corr_R = R.from_quat(correction[statenames.index(var):statenames.index(var)+4])
                correction[statenames.index(var):statenames.index(var)+3] = corr_R.as_rotvec()

        ddY = - k * Y - b * dY + correction * k
        dY = dY + ddY * self.dt * abs(delta_s)
        Y = Y + dY * self.dt * abs(delta_s)

        # Separate filter for delta_s to preserve stability of system under delta_s > 1.0
        if 'delta_s' in statenames:
            var = statenames.index('delta_s')
            dY[var] = dY[var] + ddY[var] * self.dt
            Y[var] = Y[var] + dY[var] * self.dt

        for var in ['theta_qx','qx']:
            if var in statenames:
                # convert Y back to quaternion
                Y_R = R.from_rotvec(Y[statenames.index(var):statenames.index(var)+3])
                Y[statenames.index(var):statenames.index(var)+4] = Y_R.as_quat()

        return ddY, dY, Y

    def setupSegment(self,learnedSegments,segID,from_back=False, initial_segment = False, s_start = 0, start_val_temp = None):
        segment = learnedSegments[segID]
        num_states = len(segment.state_names)
        if segment.hybrid:
            print("hybrid")
            # force onloading -- maybe
        else:
            print("not hybrid")

        ddX = np.zeros((num_states,))
        dX = np.zeros((num_states,))
        
        X = segment.start_vals
        
        if initial_segment:
            X = start_val_temp

        ddY = np.zeros((num_states,))
        dY = np.zeros((num_states,))
        Y = np.zeros((num_states,))

        for var in ['theta_qx','qx']:
            if var in segment.state_names:
                Y[segment.state_names.index(var)+3] = 1.0 #initialize quaternions to 0,0,0,1 x,y,z,w

        s = 0.0 # canonical variable
        if initial_segment:
            s = s_start
        if from_back:
            s = np.shape(segment.forcing_vals)[1] - s - 1.0
            X = segment.end_vals

        return segment, num_states, ddX, dX, X, s, ddY, dY, Y

    def getSpatialMap(self,eigen_scaling,state_names,s,surface,state_vals,dstate_vals):
        # Given a set of states, take the subset that correspond to a spatial direction (i.e., x,y,z)
        # and compute a best fit spatial map for each eigenvector
        eigen_dirs = []

        # For a potential max of 3 principal components
        for ii in range(0, 3):
            x = 0
            y = 0
            z = 0

            if 'x' in state_names:
                x += interp_matrx(eigen_scaling[ii],s)[state_names.index('x')]/self.range_per_state['x']
                y += interp_matrx(eigen_scaling[ii],s)[state_names.index('y')]/self.range_per_state['y']
                z += interp_matrx(eigen_scaling[ii],s)[state_names.index('z')]/self.range_per_state['z']

                # Add delta_s based on velocity direction
                # first, get velocity direction, then multiply by PC magnitude
                # then add to x,y,z
                vel = np.array([dstate_vals[state_names.index('x')],dstate_vals[state_names.index('y')],dstate_vals[state_names.index('z')]])
                if np.linalg.norm(vel)!=0.0:
                    vel = vel / np.linalg.norm(vel)
                scaled_delta_s = interp_matrx(eigen_scaling[ii],s)[state_names.index('delta_s')]/self.range_per_state['delta_s']
                vel_scaled = scaled_delta_s*vel
                x += vel_scaled[0]
                y += vel_scaled[1]
                z += vel_scaled[2]


            if 'u' in state_names:
                r, n_hat, r_u_norm, r_v_norm = surface.calculate_surface_point(state_vals[state_names.index('u')],state_vals[state_names.index('v')])
                constraint_frame = R.from_matrix(
                    np.hstack((r_u_norm.reshape((3, 1)), r_v_norm.reshape((3, 1)), n_hat.reshape((3, 1)))))
                temp_eigen = np.array([interp_matrx(eigen_scaling[ii],s)[state_names.index('u')]/self.range_per_state['u'],
                                       interp_matrx(eigen_scaling[ii],s)[state_names.index('v')] / self.range_per_state['v'],
                                       interp_matrx(eigen_scaling[ii],s)[state_names.index('f')] / self.range_per_state['f']])
                rotated_eigen = constraint_frame.apply(temp_eigen)
                x += rotated_eigen[0]
                y += rotated_eigen[1]
                z += rotated_eigen[2]

                # Add delta_s based on velocity direction
                vel = dstate_vals[state_names.index('u')] * r_u_norm + dstate_vals[state_names.index('v')] * r_v_norm
                if np.linalg.norm(vel) != 0.0:
                    vel = vel / np.linalg.norm(vel)
                scaled_delta_s = interp_matrx(eigen_scaling[ii], s)[state_names.index('delta_s')] / self.range_per_state[
                    'delta_s']
                vel_scaled = scaled_delta_s * vel
                x += vel_scaled[0]
                y += vel_scaled[1]
                z += vel_scaled[2]

                
            norm_temp = np.linalg.norm(np.array([x,y,z]))
            if norm_temp!=0:
                eigen_dirs.append(np.array([x,y,z]) / norm_temp)
            else:
                eigen_dirs.append(np.array([x, y, z]))

        # Make the directions orthogonal
        for ii in range(1,3):
            for jj in range(ii-1,-1,-1):
                dp = np.multiply(eigen_dirs[ii],eigen_dirs[jj])
                eigen_dirs[ii]-dp*eigen_dirs[jj]
                if np.linalg.norm(eigen_dirs[ii])!=0.0:
                    eigen_dirs[ii] = eigen_dirs[ii] / np.linalg.norm(eigen_dirs[ii])
        return eigen_dirs

    def getCorrection(self,eigen_scaling,input_type,input,state_names,s,surface,state_vals,dstate_vals):

        if input_type=="3dof":
            correction = np.zeros((len(state_names),))

            # compute best-fit spatial mapping
            eigen_dirs = self.getSpatialMap(eigen_scaling, state_names, s, surface, state_vals,dstate_vals)

            for ii in range(0,3):
                correction += np.dot(np.array([input[0],input[1],input[2]]),eigen_dirs[ii]) * eigen_scaling[ii][:,int(round(s))]

        elif input_type=="1dof":
            # simply multiply the input by the first eigenvalue
            correction = np.multiply(eigen_scaling[0][:,int(round(s))],input[0])

            # account for orientation
            for var in ['theta_qx','qx']:
                if var in state_names:
                    # convert eigen scaling to rot vector and apply scaling to that -- back to quat
                    eigscale = eigen_scaling[0][:,int(round(s))]

                    # If correction for orientation is set to zero! (shouldn't happen once upstream is reliable)
                    if np.linalg.norm(eigscale[state_names.index(var):state_names.index(var)+4])==0.0:
                        eigscale[state_names.index(var)+3] = 1.0

                    eig_quat = R.from_quat(eigscale[state_names.index(var):state_names.index(var)+4])
                    eig_rotvec = eig_quat.as_rotvec()
                    eig_quat_scaled = R.from_rotvec(eig_rotvec*input[0])
                    correction[state_names.index(var):state_names.index(var)+4] = eig_quat_scaled.as_quat()

        elif input_type=="mdof":
            correction = np.zeros((len(state_names),))

            # multiply each correction by each input dimension
            for ii in range(0,len(eigen_scaling)):
                correction_temp = np.multiply(eigen_scaling[ii][:,int(round(s))],input[ii])

                # account for orientation
                for var in ['theta_qx','qx']:
                    if var in state_names:
                        # convert eigen scaling to rot vector and apply scaling to that -- back to quat
                        eigscale = eigen_scaling[ii][:,int(round(s))]

                        # If correction for orientation is set to zero! (shouldn't happen once upstream is reliable)
                        if np.linalg.norm(eigscale[state_names.index(var):state_names.index(var)+4])==0.0:
                            eigscale[state_names.index(var)+3] = 1.0

                        eig_quat = R.from_quat(eigscale[state_names.index(var):state_names.index(var)+4])
                        eig_rotvec = eig_quat.as_rotvec()
                        eig_quat_scaled = R.from_rotvec(eig_rotvec*input[ii])
                        correction_temp[state_names.index(var):state_names.index(var)+4] = eig_quat_scaled.as_quat()
                
                correction = correction + correction_temp


        else: # otherwise, don't allow any corrections
            correction = np.zeros((len(state_names),))
            for var in ['theta_qx','qx']:
                # If correction for orientation is set to zero! (shouldn't happen once upstream is reliable)
                    if np.linalg.norm(correction[state_names.index(var):state_names.index(var)+4])==0.0:
                        correction[state_names.index(var)+3] = 1.0
        
        return correction


    def plotLearned(self,model_pkl_file):
        rospack = rospkg.RosPack()
        root_dir = rospack.get_path(
            'corrective_shared_autonomy') + '/nodes/corrective_shared_autonomy/TaskModels/learnedmodels/'
        learnedSegments = pickle.load(open(root_dir + model_pkl_file, "rb"))
        num_demos = len(learnedSegments[0].original_vals)
        self.plotModel(num_demos,learnedSegments)


    def plotLearnedCorrections(self,model_pkl_file):
        rospack = rospkg.RosPack()
        root_dir = rospack.get_path(
            'corrective_shared_autonomy') + '/nodes/corrective_shared_autonomy/TaskModels/learnedmodels/'
        learnedSegments = pickle.load(open(root_dir + model_pkl_file, "rb"))
        num_demos = len(learnedSegments[0].original_vals)


        ###############################################################
        # The below is a redo of the plotting code from per-frame PCA #
        ###############################################################
        for segment in learnedSegments:
            num_variables = len(segment.state_names)
            num_samples = np.shape(segment.original_vals[0])[1]

            matplotlib.rcParams['pdf.fonttype'] = 42

            # import matplotlib.font_manager as fm
            # minionpro = fm.FontProperties(fname='/home/mike/Desktop/minionproiit/MinionPro-Regular.otf')
            fig, ax = plt.subplots(nrows=num_variables, ncols=1, figsize=((13 / 2.54), (7 / 2.54)))
            for row_id in range(0, len(ax)):
                row = ax[row_id]
                temp2 = np.zeros((num_demos, num_samples))
                for demo_ind in range(num_demos):
                    temp2[demo_ind, :] = segment.original_vals[demo_ind][row_id, :]

                temp2 = temp2 - np.mean(temp2, axis=0)
                for demo_ind in range(num_demos):
                    row.plot(range(0, num_samples), temp2[demo_ind, :], color='gray')
                row.plot(segment.corrections[0][row_id, :], color='blue', label='PC1')
                row.plot(segment.corrections[1][row_id, :], color='green', label='PC2')
                row.plot(segment.corrections[2][row_id, :], color='red', label='PC3')
                # plt.locator_params(axis="y", nbins=2)
                # row.set_ylabel(labelsy[row_id], fontproperties=minionpro)
                row.set_ylabel(segment.state_names[row_id])
                row.margins(0, 0)
                row.tick_params(axis='y', labelsize='medium')

                # if row_id == 2:
                #     row.set_yticks([0, 10])
                # if row_id == 1:
                #     row.set_yticks([0, 0.1])
                # if row_id == 3:
                #     row.set_yticks([0, 2.5])
                if row_id < num_variables-1:
                    row.tick_params(
                        axis='x',  # changes apply to the x-axis
                        which='both',  # both major and minor ticks are affected
                        bottom=False,  # ticks along the bottom edge are off
                        top=False,  # ticks along the top edge are off
                        labelbottom=False)  # labels along the bottom edge are off
                if row_id == (num_variables-1):
                    # row.set_xlabel("Sample (n)", fontproperties=minionpro)
                    row.set_xlabel("Sample (n)")
                if row_id == (num_variables-1):
                    plt.legend(bbox_to_anchor=(0.0, -0.5), loc='upper left', ncol=3, fontsize='small')

            plt.show()


    def executeModel(self,learnedSegments = None, model_pkl_file = '',R_surface = np.array([0, 0, 0, 1]),t_surface = np.zeros((3,)),input_type="none", segID_start=0, s_start=0.0):
        self.events_pub.publish("motion_start")
        #############################
        # Load model                #
        #############################
        if learnedSegments is None:
            learnedSegments = pickle.load(open(model_pkl_file, "rb"))

        #############################
        # Execute the learned model #
        #############################
        self.rosExecution = ExecuteROS(input_tx=self.input_tx, task_R=R_surface, task_t=t_surface)

        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        rate = rospy.Rate(1.0/self.dt)  # 100hz

        num_segments = len(learnedSegments)

        # go to replay start (might be in the middle of a behavior)
        initial_segment = True
        start_val_temp = self.getStartValue(s_start,learnedSegments[segID_start])
        self.rosExecution.goToReplayStart(s_start,learnedSegments[segID_start].state_names,start_val_temp,learnedSegments[segID_start].surface,tfBuffer,listener)

        delta_s = 1.0

        # replay through each of the segments
        segID = segID_start # default 0 but can start partially through execution
        while segID < num_segments:
            segment, num_states, ddX, dX, X, s, ddY, dY, Y = self.setupSegment(learnedSegments, segID, from_back=False, initial_segment=initial_segment, s_start=s_start, start_val_temp = start_val_temp)
            
            # Confirm that the actual system has reached the approximate desired state
            pos_error = 0.04
            quat_error = 0.175 # 10 deg

            # Quit if the robot isn't active anymore or if the behavior is paused
            if not self.rosExecution.robotActive() or self.rosExecution.isInterrupted():
                self.rosExecution.shutdown()
                return segID, s

            self.rosExecution.checkCloseToSegmentStart(X,segment.surface,segment.state_names,pos_error,quat_error,tfBuffer,listener)
            
            Z = X
            initial_segment = False

            if segment.hybrid:
                surface = segment.surface
            else:
                surface = None

            while np.ceil(s) < segment.num_samples:
                input_vals, input_button = self.rosExecution.getZDInput()

                # Quit if the robot isn't active anymore or if the behavior is paused
                if not self.rosExecution.robotActive() or self.rosExecution.isInterrupted():
                    self.rosExecution.shutdown()
                    return segID, s

                correction = self.getCorrection(segment.corrections,input_type,input_vals,segment.state_names,s,surface,Z,dX)
                # correction = np.zeros(np.shape(correction))
                # Get new state value
                ddX, dX, X = self.getNewState(segment, ddX, dX, X, delta_s, s)
                ddY, dY, Y = self.getFilteredCorrection(ddY, dY, Y, correction, delta_s,segment.state_names)
                Z = self.getSaturatedZ(X,Y,segment.state_names)
            
                # Send to robot (converting if necessary)

                self.rosExecution.execute_states(segment.state_names, Z, surface,correction)

                # print("s: ", s, " ", segID)
                if 'theta_qx' in segment.state_names:
                    indtempmh = segment.state_names.index('theta_qx')
                    # print("or:",correction[indtempmh:indtempmh+4],Y[indtempmh:indtempmh+4],Z[indtempmh:indtempmh+4])
                
                delta_s = 1.0
                if 'delta_s' in segment.state_names:
                    delta_s = Z[segment.state_names.index('delta_s')]
                    if delta_s<0.05:
                        delta_s = 0.05 # Avoid traps (delta_s ==0)
                if input_button == 1.0:
                    delta_s *= -1.0

                # Increment demonstration
                # print("deltas:", delta_s," ",Y[segment.state_names.index('delta_s')])
                s += delta_s

                # If backwards, check for segment change
                if(s < 0 and segID > 0): # revert to previous segment
                    segID = segID - 1
                    segment, num_states, ddX, dX, X, s, ddY, dY, Y = self.setupSegment(learnedSegments, segID, from_back=True)
                    Z = self.getSaturatedZ(X,Y,segment.state_names)
                    if segment.hybrid:
                        surface = segment.surface
                elif(s < 0):
                    s = 0.0 # nowhere else to go backwards -- sit at this position
                    delta_s = 0.0
                rate.sleep()

            segID = segID + 1

        self.events_pub.publish("motion_finished")

        return -1, -1 # segID is -1 and s is -1 if successful









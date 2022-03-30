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
import rospy
import rospkg
import pickle
import time
import matplotlib
import matplotlib.pyplot as plt
import math

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
        self.num_kernels = 60
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
    def getDMPForcing(self,x, xf):
        # Finit differencing to get the desired velocity and acceleration to compute the force
        vel = np.gradient(x) / self.dt
        acc = np.gradient(vel) / self.dt

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
        start_sum=0.0; end_sum=0.0
        for ii in range(0,num_demos):
            start_sum+=data[ii][0]
            end_sum+=data[ii][-1]
        start = start_sum/float(num_demos); end = end_sum/float(num_demos)

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
            f_data = self.getDMPForcing(data[ii], end)
            samples = np.arange(0, len(f_data))

            # canonical system
            s_alpha = 3 / (len(samples) * self.dt)
            canonical = np.exp(-s_alpha * np.arange(0, len(samples)) * self.dt)  # decay function

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
        return start,end,forcing

##############################
# DMP + LWR Learning Model   #
##############################

class HybridSegment:
    k = 50
    b = 2*np.sqrt(50)
    surface=''
    num_samples = 0
    state_names=[]
    forcing_vals=[]
    rev_forcing_vals = []
    original_vals = []
    corrections = []
    start_vals = []
    end_vals = []
    hybrid=False


class DMPLWR:
    def __init__(self,surfacefile='', verbose=False, input_tx=np.array([0, 0, 0, 1])):
        # DMP parameters
        self.k = 50
        self.b = 2*np.sqrt(self.k)
        self.dt = 0.01

        self.surface_file = surfacefile

        self.state_names = [] # keeps track of name of states
        self.states = []

        self.verbose = verbose

        self.input_tx = input_tx

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
        x = np.zeros((0,))
        y = np.zeros((0,))
        z = np.zeros((0,))
        dmptemp = DMP(self.k, self.b, self.dt)
        for segment in learnedSegments:
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
                surface = BSplineSurface()
                surface.loadSurface(segment.surface)

                for u_temp,v_temp in zip(u,v):
                    r, _, _, _ = surface.calculate_surface_point(u_temp,v_temp)
                    x = np.append(x,r[0])
                    y = np.append(y,r[1])
                    z = np.append(z,r[2])

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

                x = np.append(x,x_temp)
                y = np.append(y,y_temp)
                z = np.append(z,z_temp)
        return x, y, z

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

    def learnModel(self,segments,outfile='output.pkl'):
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

        for ii in range(0,len(segments)-1): # TODO: why one less than end?
            segmentTemp = learnedSegments[ii]
            if self.verbose:
                print("DMP for segment ", ii, " of ",(len(segments) - 1))

            segmentTemp.start_vals = np.zeros((len(segmentTemp.state_names,)))
            segmentTemp.end_vals = np.zeros((len(segmentTemp.state_names, )))
            segmentTemp.forcing_vals = np.zeros((len(segmentTemp.state_names),segmentTemp.num_samples))
            segmentTemp.rev_forcing_vals = np.zeros((len(segmentTemp.state_names), segmentTemp.num_samples))
            for jj in range(0,len(segmentTemp.state_names)):
                data_temp = []
                data_temp_r = []
                for kk in range(0,num_demos):
                    data_temp.append(segmentTemp.original_vals[kk][jj,:])
                    data_temp_r.append(np.flip(segmentTemp.original_vals[kk][jj, :]))

                dmptemp = DMP(self.k,self.b,self.dt)
                start,end,forcing = dmptemp.getForcing(data_temp)
                _,_,reverse_forcing = dmptemp.getForcing(data_temp_r)
                segmentTemp.start_vals[jj] = start
                segmentTemp.end_vals[jj] = end
                segmentTemp.forcing_vals[jj,:] = forcing
                segmentTemp.rev_forcing_vals[jj,:] = reverse_forcing

                # # To see individual DMP plots
                # import matplotlib.pyplot as plt
                # for oo in range(0,num_demos):
                #     plt.plot(data_temp[oo],color='gray')
                # plt.plot(dmptemp.getPath(start, end, forcing),color='red')
                # plt.show()

        rospack = rospkg.RosPack()
        root_dir = rospack.get_path(
            'corrective_shared_autonomy') + '/nodes/corrective_shared_autonomy/TaskModels/learnedmodels/'
        pickle.dump(learnedSegments, open(root_dir + "yoyo.pkl", "wb"))

        ####### 2. Corrections for provided values #############
        for segment in learnedSegments:
            var_ranges = []
            for xx in range(0,len(segment.state_names)):
                if segment.state_names[xx] in self.range_per_state.keys():
                    var_ranges.append(self.range_per_state[segment.state_names[xx]])
                else:
                    var_ranges.append(1.0)
            var_ranges = np.array(var_ranges)
            segment.corrections = PerSamplePCA(segment.original_vals, var_ranges)

        # Save segments to file for execution
        rospack = rospkg.RosPack()
        root_dir = rospack.get_path('corrective_shared_autonomy') + '/nodes/corrective_shared_autonomy/TaskModels/learnedmodels/'
        pickle.dump(learnedSegments,open(root_dir+outfile,"wb"))

        if self.verbose:
            print("Time to learn model: %s seconds" % (time.time() - start_time))
            self.plotModel(num_demos, learnedSegments)


    def linearInterpolation(self,start,end,val):
        return start + (end-start)*val


    def getNewState(self,segment,ddX,dX,X,delta_s,s):
        length_s = len(segment.forcing_vals)
        if delta_s>=0: # forwards
            ddX = segment.k * (segment.end_vals - X) - segment.b * dX + interp_matrx(segment.forcing_vals,s)
        else: # backwards
            ddX = segment.k * (segment.start_vals - X) - segment.b * dX + interp_matrx(segment.rev_forcing_vals,length_s-s)
        dX = dX + ddX * self.dt * abs(delta_s)
        X = X + dX * self.dt * abs(delta_s)
        return ddX, dX, X

    def getSaturatedZ(self,X,Y,state_names):
        Z = X+Y
        if 'u' in state_names:
            if Z[state_names.index('u')]>0.99:
                Z[state_names.index('u')] = 0.99
            if Z[state_names.index('u')]<0.01:
                Z[state_names.index('u')] = 0.01
        if 'v' in state_names:
            if Z[state_names.index('v')]>0.99:
                Z[state_names.index('v')] = 0.99
            if Z[state_names.index('v')]<0.01:
                Z[state_names.index('v')] = 0.01
        return Z

    def getFilteredCorrection(self,ddY, dY, Y, correction, delta_s):
        k = 50
        b = 50*np.sqrt(2)
        ddY = - k * Y - b * dY + correction * k
        dY = dY + ddY * self.dt * abs(delta_s)
        Y = Y + dY * self.dt * abs(delta_s)
        return ddY, dY, Y

    def setupSegment(self,learnedSegments,segID,from_back=False):
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
        ddY = np.zeros((num_states,))
        dY = np.zeros((num_states,))
        Y = np.zeros((num_states,))

        s = 0.0  # canonical variable
        if from_back:
            s = np.shape(segment.forcing_vals)[1] - 1.0
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

        else: # otherwise, don't allow any corrections
            correction = np.zeros((len(state_names),))

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


    def executeModel(self,model_pkl_file,input_type="none"):
        #############################
        # Load model                #
        #############################
        rospack = rospkg.RosPack()
        root_dir = rospack.get_path(
            'corrective_shared_autonomy') + '/nodes/corrective_shared_autonomy/TaskModels/learnedmodels/'
        learnedSegments = pickle.load(open(root_dir+model_pkl_file, "rb"))

        #############################
        # Execute the learned model #
        #############################
        rosExecution = ExecuteROS(input_tx=self.input_tx)

        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        rate = rospy.Rate(1.0/self.dt)  # 100hz

        num_segments = len(learnedSegments)

        # go to replay start
        rosExecution.goToReplayStart(learnedSegments[0].state_names,learnedSegments[0].start_vals,tfBuffer,listener)

        delta_s = 1.0

        # replay through each of the segments
        segID = 0
        while segID < num_segments:
            segment, num_states, ddX, dX, X, s, ddY, dY, Y = self.setupSegment(learnedSegments, segID, from_back=False)
            Z = X

            if segment.hybrid:
                surface = BSplineSurface()
                surface.loadSurface(segment.surface)
            else:
                surface = None

            while np.ceil(s) < segment.num_samples:
                input_vals, input_button = rosExecution.getZDInput()

                correction = self.getCorrection(segment.corrections,input_type,input_vals,segment.state_names,s,surface,Z,dX)
                # correction = np.zeros(np.shape(correction))

                # Get new state value
                ddX, dX, X = self.getNewState(segment, ddX, dX, X, delta_s, s)
                ddY, dY, Y = self.getFilteredCorrection(ddY, dY, Y, correction, delta_s)
                Z = self.getSaturatedZ(X,Y,segment.state_names)

                # print(segment.state_names)
                # print("C:", correction)
                # print("Y:", Y)
                # print(input_vals)
                if segment.hybrid:
                    print("Fc: ",Y[3]," ",Z[3])

                # print("X:",X[0],X[1],X[2])
                # print("start:",segment.start_vals[0],segment.start_vals[1],segment.start_vals[2])
                # print("end:",segment.end_vals[0],segment.end_vals[1],segment.end_vals[2])

                # Send to robot (converting if necessary)

                rosExecution.execute_states(segment.state_names, Z, surface,correction)

                # print("s: ", s, " ", segID)

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
                        surface = BSplineSurface()
                        surface.loadSurface(segment.surface)
                elif(s < 0):
                    s = 0.0 # nowhere else to go backwards -- sit at this position
                    delta_s = 0.0
                rate.sleep()

            segID = segID + 1











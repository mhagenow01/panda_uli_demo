""" Uniform B-spline surfaces
    based on user input of control
    points and curve order

 Created: 05/07/2020
"""

__author__ = "Mike Hagenow"

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import math
import csv
from scipy.optimize import minimize, Bounds
import rospkg
from core_robotics.quaternion import qtoA
from core_robotics.filters import butter_lowpass_filter
from scipy.spatial.transform import Rotation as R
import time
import math

def convertToUV(surface_file, num_samples, x, y, z, fx, fy, fz, qx, qy, qz, qw):
    surfaceModel = BSplineSurface()
    surfaceModel.loadSurface(surface_file)

    temp_u = np.copy(np.zeros(num_samples, ))
    temp_v = np.copy(np.zeros(num_samples, ))
    temp_f = np.copy(np.zeros(num_samples, ))
    temp_rot_u = np.copy(np.zeros(num_samples, ))
    temp_rot_v = np.copy(np.zeros(num_samples, ))
    temp_rot_n = np.copy(np.zeros(num_samples, ))

    temp_rot_qx = np.copy(np.zeros(num_samples, ))
    temp_rot_qy = np.copy(np.zeros(num_samples, ))
    temp_rot_qz = np.copy(np.zeros(num_samples, ))
    temp_rot_qw = np.copy(np.zeros(num_samples, ))

    # default u and v for optimization
    u = 0; v = 0
    for jj in range(0, num_samples):
        u, v = surfaceModel.getClosestParams(x[jj], y[jj], z[jj], u, v)
        temp_u[jj] = u
        temp_v[jj] = v
        _, n_hat, r_u_norm, r_v_norm = surfaceModel.calculate_surface_point(u, v)

        # get forces in the normal direction
        temp_f[jj] = np.dot(np.array([fx[jj], fy[jj], fz[jj]]), n_hat)

        # get the roll and pitch angles by doing projections of the orientation z-direction
        q_temp = np.array([qx[jj], qy[jj], qz[jj], qw[jj]]).reshape((4,))
        A_temp = qtoA(q_temp)
        z_dir = A_temp[0:3, 2]

        constraint_frame = R.from_matrix(np.hstack((r_u_norm.reshape((3,1)),r_v_norm.reshape((3,1)),n_hat.reshape((3,1)))))
        q_scipy = R.from_quat([qx[jj], qy[jj], qz[jj], qw[jj]])
        q_in_frame = constraint_frame.inv() * q_scipy
        euler = q_in_frame.as_euler('xyz')
        quat_frame = q_in_frame.as_quat()

        # try to avoid Euler flipping
        for ee in range(0,3):
            if euler[ee]<0:
                euler[ee]+= 2*np.pi
        temp_rot_u[jj] = euler[0]
        temp_rot_v[jj] = euler[1]
        temp_rot_n[jj] = euler[2]
        temp_rot_qx[jj] = quat_frame[0]
        temp_rot_qy[jj] = quat_frame[1]
        temp_rot_qz[jj] = quat_frame[2]
        temp_rot_qw[jj] = quat_frame[3]

        # # project into u-n plane and determine rotation
        # proj_un = z_dir - np.dot(z_dir, r_v_norm) * r_v_norm
        # proj_un_norm = proj_un / np.linalg.norm(proj_un)
        # angle_temp_u = np.arccos(np.dot(proj_un_norm, n_hat))
        # dir = np.dot(np.cross(n_hat, r_u_norm), np.cross(n_hat, proj_un_norm))
        # temp_rot_v[jj] = np.sign(dir) * angle_temp_u
        #
        # # project into v-n plane and determine rotation
        # proj_vn = z_dir - np.dot(z_dir, r_u_norm) * r_u_norm
        # proj_vn_norm = proj_vn / np.linalg.norm(proj_vn)
        # angle_temp_v = np.arccos(np.dot(proj_vn_norm, n_hat))
        # dir = -np.dot(np.cross(n_hat, r_v_norm), np.cross(n_hat, proj_vn_norm))
        # temp_rot_u[jj] = np.sign(dir) * angle_temp_v

    # filter the rotations
    temp_rot_u = butter_lowpass_filter(temp_rot_u, 110 / 20.0, 110.0, order=1)
    temp_rot_v = butter_lowpass_filter(temp_rot_v, 110 / 20.0, 110.0, order=1)
    temp_rot_n = butter_lowpass_filter(temp_rot_n, 110 / 20.0, 110.0, order=1)

    return temp_u, temp_v, temp_f, temp_rot_qx, temp_rot_qy, temp_rot_qz, temp_rot_qw

def exp_map_np(w):
    w = np.array(w)
    w = w.reshape(3, 1)
    theta = (w[0] ** 2 + w[1] ** 2 + w[2] ** 2) ** 0.5 + 1e-30
    w = w / theta
    w_hat = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    return np.eye(3) + w_hat * np.sin(theta) + np.dot(w_hat, w_hat) * (1 - np.cos(theta))


class BSplineSurface:

    def initialize(self,k=3,control_pts=np.zeros((1,1,3)),u_dir=None, v_dir=None):
        # Parameters for building B-Spline Surface
        self.k = k # order of the B-spline
        self.controls_pts = control_pts # n control points by 3 (x,y,z)
        num_control_pts = np.shape(control_pts)[0:2]
        if num_control_pts[0] <= k or num_control_pts[1]<=k:
            raise Exception("Parameter","Not enough control pts for curve degree")
        self.m = num_control_pts[0]-1 # number of control points in u-direction 0-> m
        self.n = num_control_pts[1]-1  # number of control points in v-direction 0 -> n

        # Need to add the first and last value k times (total of (m-k+2) = ((m+k+2) - 2k)
        self.knots_u = np.concatenate([np.zeros((self.k,)), np.linspace(0, 1, (self.m-self.k+2)),
                                       np.ones((self.k,))])  # number uniform knots in u-direction
        self.knots_v = np.concatenate([np.zeros((self.k,)), np.linspace(0, 1, (self.n-self.k+2)),
                                       np.ones((self.k,))])  # number uniform knots in v-direction


        # u and v directions to be used for input mapping
        if u_dir is not None and v_dir is not None:
            self.u_dir = u_dir
            self.v_dir = v_dir
        else:
            self.u_dir = np.array([1,0,0])
            self.v_dir = np.array([0,1,0])

        #self.u_dir, self.v_dir = self.calculate_planar_directions()

    def exp_map_np(self,w):
        w = np.array(w)
        w = w.reshape(3, )
        theta = (w[0] ** 2 + w[1] ** 2 + w[2] ** 2) ** 0.5 + 1e-30
        w = w / theta
        w_hat = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
        return np.eye(3) + w_hat * np.sin(theta) + np.dot(w_hat, w_hat) * (1 - np.cos(theta))

    # x are the parameters, stored as wx,wy,d
    # y are the points in the point cloud
    def obj_plane_dist(self,x, pts):
        sum = 0.0
        for pt in pts:
            # equation for the plane - sum of residual is used as the LSQ error
            vec_to_plane_center = np.transpose(
                np.matmul(exp_map_np([x[0], x[1], 0.0]), np.array([0.0, 0.0, x[2]]).reshape((3, 1))) - pt.reshape(
                    (3, 1)))
            sum += np.abs(np.matmul(vec_to_plane_center, np.matmul(exp_map_np([x[0], x[1], 0.0]),
                                                                   np.array([0.0, 0.0, 1.0]).reshape((3, 1))))[0][0])
        # print x[0], x[1], x[2], sum
        return sum

    def findPlane(self,pts):
        best_opt = np.inf
        for ii in range(0, 5):
            wx = np.random.rand() * np.pi / 2
            wy = np.random.rand() * np.pi / 2
            d = 1.0 * (np.random.rand() - 0.5)

            x0 = np.array([wx, wy, d])
            res = minimize(self.obj_plane_dist, x0, method='Nelder-Mead',
                           options={'disp': False}, bounds=None, args=(pts))

            if res.fun < best_opt:
                best_opt = res.fun
                wx_best = res.x[0]
                wy_best = res.x[1]
                d_best = res.x[2]

        return wx_best, wy_best, d_best

    def calculate_surface_point_old(self,u,v):
        # Calculate all of the basis functions for the given u,v
        # using recursive formulation
        r = np.array([0.0, 0.0, 0.0])

        ########################################
        # Calculate the interpolated point     #
        ########################################
        uu_cache = np.zeros((self.m+1,))
        uu_partial_cache = np.zeros((self.m,))
        vv_cache = np.zeros((self.n+1,))
        vv_partial_cache = np.zeros((self.n + 1,))
        for ii in range(0,self.m+1):
            uu_cache[ii] = self.getN(ii,self.k,u,self.knots_u)
            if ii!=self.m:
                uu_partial_cache[ii] = self.getN(ii, self.k - 1, u, self.knots_u[1:-1])
        for jj in range(0,self.n+1):
            vv_cache[jj] = self.getN(jj,self.k,v,self.knots_v)
            if jj != self.n:
                vv_partial_cache[jj] = self.getN(jj, self.k-1, v,self.knots_v[1:-1])

        # Calculate the sum of basis functions for the point interpolation
        for ii in range(0,self.m+1):
            # print("\n")
            for jj in range(0,self.n+1):
                r += uu_cache[ii]*vv_cache[jj]*self.controls_pts[ii,jj,:]
                # print(str(uu_cache[ii]*vv_cache[jj])+",",end='')

        ########################################
        # Calculate the normal                 #
        ########################################

        # Partial in the U-direction
        r_u = np.array([0.0, 0.0, 0.0])
        for ii in range(0,self.m):
            for jj in range(0,self.n+1):
                scaling_temp =0.0
                if (self.knots_u[ii+self.k+1]-self.knots_u[ii+1]) != 0:
                    scaling_temp = self.k/(self.knots_u[ii+self.k+1]-self.knots_u[ii+1])
                r_u+= uu_partial_cache[ii]*vv_cache[jj]*scaling_temp*(self.controls_pts[ii+1,jj,:]-self.controls_pts[ii,jj,:])

        # Partial in the V-direction
        r_v = np.array([0.0, 0.0, 0.0])
        for ii in range(0, self.m+1):
            uu = self.getN(ii, self.k, u, self.knots_u)
            for jj in range(0, self.n):
                scaling_temp = 0.0
                if (self.knots_v[jj + self.k + 1] - self.knots_v[jj + 1]) != 0:
                    scaling_temp = self.k / (self.knots_v[jj + self.k + 1] - self.knots_v[jj + 1])
                r_v += uu_cache[ii] * vv_partial_cache[jj] * scaling_temp * (self.controls_pts[ii, jj + 1, :] - self.controls_pts[ii, jj, :])


        # Get the surface normal from the cross-product
        r_u_norm = np.divide(r_u,np.linalg.norm(r_u))
        r_v_norm = np.divide(r_v,np.linalg.norm(r_v))
        n_hat = np.cross(r_u_norm,r_v_norm)

        # return the calculated point
        return r, n_hat, r_u_norm, r_v_norm


    def calculate_surface_point(self,u,v):
        # Calculate all of the basis functions for the given u,v
        # using recursive formulation
        r = np.array([0.0, 0.0, 0.0])

        approx_u = int(u*self.m+1)
        approx_v = int(v*self.n+1)
        lower_u = approx_u - self.k
        upper_u = approx_u + self.k
        if lower_u < 0: lower_u = 0
        if upper_u > self.m+1: upper_u = self.m+1
        lower_v = approx_v - self.k
        upper_v = approx_v + self.k
        if lower_v < 0: lower_v = 0
        if upper_v > self.n+1: upper_v = self.n+1

        # print(int(math.ceil(self.k/2.0)))
        # print(lower_u,upper_u,lower_v,upper_v)

        ########################################
        # Calculate the interpolated point     #
        ########################################
        uu_cache = np.zeros((self.m+1,))
        uu_partial_cache = np.zeros((self.m,))
        vv_cache = np.zeros((self.n+1,))
        vv_partial_cache = np.zeros((self.n + 1,))
        for ii in range(lower_u,upper_u):
            uu_cache[ii] = self.getN(ii,self.k,u,self.knots_u)
            if ii!=self.m:
                uu_partial_cache[ii] = self.getN(ii, self.k - 1, u, self.knots_u[1:-1])
        for jj in range(lower_v,upper_v):
            vv_cache[jj] = self.getN(jj,self.k,v,self.knots_v)
            if jj != self.n:
                vv_partial_cache[jj] = self.getN(jj, self.k-1, v,self.knots_v[1:-1])

        # Calculate the sum of basis functions for the point interpolation
        for ii in range(lower_u,upper_u):
            for jj in range(lower_v,upper_v):
                r += uu_cache[ii]*vv_cache[jj]*self.controls_pts[ii,jj,:]

        ########################################
        # Calculate the normal                 #
        ########################################

        upper_u_partial = upper_u
        if upper_u_partial == self.m+1: upper_u_partial = self.m
        upper_v_partial = upper_v
        if upper_v_partial == self.n+1: upper_v_partial = self.n

        # Partial in the U-direction
        r_u = np.array([0.0, 0.0, 0.0])
        for ii in range(lower_u,upper_u_partial):
            for jj in range(lower_v,upper_v):
                scaling_temp =0.0
                if (self.knots_u[ii+self.k+1]-self.knots_u[ii+1]) != 0:
                    scaling_temp = self.k/(self.knots_u[ii+self.k+1]-self.knots_u[ii+1])
                r_u+= uu_partial_cache[ii]*vv_cache[jj]*scaling_temp*(self.controls_pts[ii+1,jj,:]-self.controls_pts[ii,jj,:])

        # Partial in the V-direction
        r_v = np.array([0.0, 0.0, 0.0])
        for ii in range(lower_u, upper_u):
            uu = self.getN(ii, self.k, u, self.knots_u)
            for jj in range(lower_v, upper_v_partial):
                scaling_temp = 0.0
                if (self.knots_v[jj + self.k + 1] - self.knots_v[jj + 1]) != 0:
                    scaling_temp = self.k / (self.knots_v[jj + self.k + 1] - self.knots_v[jj + 1])
                r_v += uu_cache[ii] * vv_partial_cache[jj] * scaling_temp * (self.controls_pts[ii, jj + 1, :] - self.controls_pts[ii, jj, :])


        # Get the surface normal from the cross-product
        r_u_norm = np.divide(r_u,np.linalg.norm(r_u))
        r_v_norm = np.divide(r_v,np.linalg.norm(r_v))
        n_hat = np.cross(r_u_norm,r_v_norm)

        # return the calculated point
        return r, n_hat, r_u_norm, r_v_norm

    def getN(self,i,p,x,t):
        # Recursive function that calculates the basis
        # function value using De Boor's
        # (simple version of the algorithm with nan checking)

        if p==0: # 0-th level is a boolean of sorts
            if x>=t[i] and x<t[i+1]: # Firing condition
                return 1.0
            else: # No Fire Condition
                return 0.0

        else: # other levels are recursions based on lower levels
            part_a = 0.0
            part_b = 0.0

            # Check for the two potential divide by zero cases (should be 0)
            if((t[i+p]-t[i]) != 0):
                part_a = (x-t[i])/(t[i+p]-t[i])*self.getN(i,p-1,x,t)
            if (t[i+p+1]-t[i+1]) !=0:
                part_b = (t[i+p+1]-x)/(t[i+p+1]-t[i+1])*self.getN(i+1,p-1,x,t)
            return part_a + part_b

    def obj_closest_surface(self, x, surface, xf, yf, zf):
        r, n_hat, r_u_norm, r_v_norm = surface.calculate_surface_point(x[0], x[1])
        return (r[0] - xf) * (r[0] - xf) + (r[1] - yf) * (r[1] - yf) + (r[2] - zf) * (r[2] - zf)

    def getClosestParams(self, x, y, z, u, v):
        x0 = np.array([u, v])
        bounds = Bounds([0.0, 0.0], [1.0, 1.0])
        res = minimize(self.obj_closest_surface, x0, method='L-BFGS-B',
                       options={'disp': False}, bounds=bounds, args=(self, x, y, z))
        return res.x[0], res.x[1]

    def writeSurface(self,filename):
        rospack = rospkg.RosPack()
        path_devel = rospack.get_path('corrective_shared_autonomy') + "/nodes/corrective_shared_autonomy/TaskModels/learnedmodels/"
        with open(filename, 'w') as csvfile:
            # Only parameters needed are degree and control points
            csvfile.write(str(self.k) + ',' + str(self.m) + ',' + str(self.n))
            csvfile.write('\n')

            # Load all of the control points
            for ii in range(0, self.m + 1):
                for jj in range(0, self.n + 1):
                    csvfile.write(str(self.controls_pts[ii,jj,0])+' '+str(self.controls_pts[ii,jj,1])+' '+
                                  str(self.controls_pts[ii,jj,2])+',')
                csvfile.write('\n')

            # write u and v directions
            csvfile.write(str(self.u_dir[0]) + ',' + str(self.u_dir[1]) + ',' +
            str(self.u_dir[2]) + ',')
            csvfile.write('\n')
            csvfile.write(str(self.v_dir[0]) + ',' + str(self.v_dir[1]) + ',' +
            str(self.v_dir[2]) + ',')
            csvfile.write('\n')

    def loadSurface(self, filename):
        rospack = rospkg.RosPack()
        path_devel = ""
        with open(filename) as csvfile:
            surfacereader = csv.reader(csvfile,delimiter=',')

            # Load the parameters and control points
            row = 0
            for row_temp in surfacereader:
                if row==0:
                    k = int(row_temp[0])
                    num_control_u = int(row_temp[1])
                    num_control_v = int(row_temp[2])
                    control_pts = np.zeros((num_control_u+1,num_control_v+1,3))
                else:
                    col=0
                    if row<=(num_control_u+1): # don't include u and v directions at the end
                        for ii in range(0,num_control_v+1):
                                point = row_temp[ii]
                                values = point.split(' ')
                                # one extra row for parameters
                                control_pts[row-1,col,0]=float(values[0])
                                control_pts[row-1,col,1]=float(values[1])
                                control_pts[row-1,col,2]=float(values[2])
                                col+=1
                row+=1

            # initialize the B-Spline
            self.initialize(k=k,control_pts=control_pts)


def createFastenerSurface():
    num_ctrl_pts = 30
    theta = np.linspace(279,251,num_ctrl_pts)
    radius = np.linspace(0.52959,0.546,num_ctrl_pts) #original .54864, but goes into surface
    thetav, radiusv = np.meshgrid(theta, radius)
    xv = 0.0644364+radiusv*np.cos(np.deg2rad(thetav))
    yv = 0.44098+radiusv*np.sin(np.deg2rad(thetav))
    zv = 0.128228+0.0*thetav

    control_pts = np.transpose([xv, yv, zv])

    # Apply a rotation and a translation to all of the points
    R = np.array([[0.99978826, 0.00928849, -0.01836173],
                  [-0.00907831, 0.9998927, 0.01149706],
                  [0.01846655, -0.01132793, 0.9997653]])

    t = np.array([0.38238362, -0.29635461, 0.012553])

    t_addtl = np.matmul(R, np.array([0.025 * 5, 0.025 * 13, 0.0]).reshape((3, 1))).reshape((3,))
    t = t + t_addtl

    for ii in range(0, num_ctrl_pts):
        for jj in range(0, num_ctrl_pts):
            vector = np.array([control_pts[ii,jj,0],control_pts[ii,jj,1],control_pts[ii,jj,2]])
            control_pts[ii,jj,:] = (np.matmul(R, vector.reshape((3, 1))) + t.reshape((3, 1))).reshape(3, )

    pt1_u = np.array([0.0644364+0.52959*np.cos(np.deg2rad(279)),0.44098+0.52959*np.sin(np.deg2rad(279)),0.128228])
    pt2_u = np.array([0.0644364+0.52959*np.cos(np.deg2rad(251)),0.44098+0.52959*np.sin(np.deg2rad(251)),0.128228])
    pt1_v = np.array([0.0644364+0.52959*np.cos(np.deg2rad(265)),0.44098+0.52959*np.sin(np.deg2rad(265)),0.128228])
    pt2_v = np.array([0.0644364+0.54864*np.cos(np.deg2rad(265)),0.44098+0.54864*np.sin(np.deg2rad(265)),0.128228])
    u_dir = (pt2_u-pt1_u)/np.linalg.norm(pt2_u-pt1_u)
    v_dir = (pt2_v-pt1_v)/np.linalg.norm(pt2_v-pt1_v)

    print("----------------")

    for ii in range(0,num_ctrl_pts):
        for jj in range(0,num_ctrl_pts):
            print("point = {"+str(control_pts[ii,jj,0])+", "+str(control_pts[ii,jj,1])+", "+str(control_pts[ii,jj,2])+"}")
            print("pt=sim.addDrawingObject(dr,0.001,0.0,-1,30000,color)")
            print("point[1]=point[1]+panda_frame[1]")
            print("point[2]=point[2]+panda_frame[2]")
            print("point[3]=point[3]+panda_frame[3]")
            print("sim.addDrawingObjectItem(pt,point)")


    curve = BSplineSurface()
    curve.initialize(k=3, control_pts=control_pts,u_dir = u_dir ,v_dir= v_dir)
    curve.writeSurface('fastener1')


def testSubset():
    surf_path = '/home/mike/Documents/demo/src/panda_uli_demo/ULIConfig/registration_models/anna_study1.csv'
    test = BSplineSurface()
    test.loadSurface(surf_path)
    startt = time.time()
    u = 0.2
    v = 0.8
    x = test.calculate_surface_point_old(u,v)
    t1 = time.time()-startt
    startt = time.time()
    y = test.calculate_surface_point(u,v)
    t2 = time.time()-startt
    print("before: ",t1,x)
    print("after: ",t2,y)
    print("Speedup BsplineSurface: ",t1/t2)


if __name__ == "__main__":
    testSubset()





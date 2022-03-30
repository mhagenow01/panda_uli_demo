#!/usr/bin/env python

""" Performs FPCA using stacked, concatenated PCA
 Created: 01/21/2021
"""

__author__ = "Mike Hagenow"

import numpy as np
from scipy import signal
from scipy.ndimage import interpolation
import matplotlib.pyplot as plt


def varimax(Phi, gamma=0.00001, q=1, tol=1e-6):
    p, k = Phi.shape
    R = np.eye(k)
    d = 0
    for i in xrange(q):
        d_old = d
        Lambda = np.dot(Phi, R)
        u, s, vh = np.linalg.svd(np.dot(Phi.T, np.asarray(Lambda) ** 3 - (gamma / p) * np.dot(Lambda, np.diag(
            np.diag(np.dot(Lambda.T, Lambda))))))
        R = np.dot(u, vh)
        d = sum(s)
        if d_old != 0 and d / d_old < 1 + tol: break
    return np.dot(Phi, R), R

def interpolate_eigenfunctions(data,num_pts):
    data_out = []
    for fxn in data:
        data_temp = np.zeros((np.shape(fxn)[0],num_pts))
        for ii in range(0,np.shape(fxn)[0]):
            data_temp[ii,:]=interpolation.zoom(fxn[ii,:], float(num_pts) /len(fxn[ii,:]))

        data_out.append(data_temp)
    return data_out

def FPCA(data_in, var_ranges,plotting=False):
    # data_in will be a list of d demonstrations, each of which is a mxn matrix
    # for FPCA in multiple dimensions, it will be done on the concatenation of all variables
    # into a single vector for each demonstration (dx(mn))

    # the output will ultimately be a list of mxn matrices, each of which corresponds to a functional principal component

    num_variables = np.shape(data_in[0])[0]  # number of rows in the demo
    num_demos = len(data_in)
    num_samples = np.shape(data_in[0])[1]

    # Step 1: remove the means (prerequsite for PCA/FPCA)
    # loop over each variable

    total = np.zeros((num_demos,0))

    std_dev_per_variable = []

    for var_ind in range(num_variables):
        temp = np.zeros((num_demos,num_samples))

        for demo_ind in range(num_demos):
            temp[demo_ind,:] = data_in[demo_ind][var_ind,:]

        temp = temp - np.mean(temp,axis=0)
        scale_temp = var_ranges[var_ind]
        if(scale_temp==0):
            scale_temp = 1 # catch for data with exactly no variance
        temp = temp/scale_temp
        std_dev_per_variable.append(scale_temp)

        total = np.concatenate((total,temp), axis=1)


    U,s,Vt = np.linalg.svd(total,full_matrices=False)
    s_total = 0.0
    for ii in range(0,len(s)):
        s_total +=s[ii]**2
    print("FPCA Result (% var 1st FPC):",(s[0]**2)/s_total)
    print("FPCA Result (% var 2st FPC):",(s[1]**2)/s_total)


    fpcs=[]
    for pc_ind in range(0,len(s)):
        temp = np.zeros((num_variables,num_samples))
        for ii in range(0,num_variables):
            temp[ii,:]=3.0*s[pc_ind]/np.sqrt(num_demos)*std_dev_per_variable[ii]*np.split(Vt[pc_ind],num_variables)[ii]
        fpcs.append(np.copy(temp))



    ######################################
    # From here on out is just plotting ##
    ######################################
    if plotting:
        fig, ax = plt.subplots(nrows=num_variables, ncols=1)

        for row_id in range(0,len(ax)):
            row = ax[row_id]

            temp2 = np.zeros((num_demos, num_samples))
            for demo_ind in range(num_demos):
                temp2[demo_ind, :] = data_in[demo_ind][row_id,:]

            temp2 = temp2 - np.mean(temp2, axis=0)
            for demo_ind in range(num_demos):
                row.plot(range(0,num_samples),temp2[demo_ind,:],color='gray')
            row.plot(fpcs[0][row_id,:],color='blue')
            row.plot(fpcs[1][row_id,:],color='green')
            row.plot(fpcs[2][row_id,:],color='red')

            if(np.max(np.abs(np.correlate(Vt[0], Vt[1],mode='full')))>0.7):
                corre = np.correlate(Vt[0], Vt[1], mode='full')


                rot_vec, R = varimax(Vt[0:2, :].T)
                val_before = np.diag([s[0], s[1]])
                val_after = np.dot(val_before, R)

                row.plot(np.split(rot_vec.T[0,:], num_variables)[row_id], color='yellow')
                row.plot(np.split(rot_vec.T[1,:], num_variables)[row_id], color='lime')


        plt.show()
        rot_vec, R = varimax(Vt[0:2, :].T)
        print(np.max(np.abs(np.correlate(rot_vec.T[0], rot_vec.T[1],mode='full'))))
        rot_vec, R = varimax(Vt[[0,2], :].T)
        print(np.max(np.abs(np.correlate(rot_vec.T[0], rot_vec.T[1], mode='full'))))
        rot_vec, R = varimax(Vt[[1, 2], :].T)
        print(np.max(np.abs(np.correlate(rot_vec.T[0], rot_vec.T[1], mode='full'))))



        #
        # # fig = plt.figure()
        # # plt.plot(np.abs(np.correlate(Vt[0], Vt[1],mode='full')),color='g')
        # # # plt.plot(np.abs(np.correlate(Vt[0], Vt[0],mode='full')),color='r')
        # # plt.show()
        # if (np.max(np.abs(np.correlate(Vt[0], Vt[1], mode='full'))) > 0.7):
        #     fig = plt.figure()
        #     plt.plot(np.split(rot_vec.T[0,:], num_variables)[2],color='g')
        #     plt.plot(np.split(rot_vec.T[1,:], num_variables)[2],color='b')
        #
        #     # plt.plot(np.abs(np.correlate(Vt[0], Vt[0],mode='full')),color='r')
        #     plt.show()


    return fpcs


def testFPCA():
    demos = []
    for ii in range(0,4):
        temp = np.copy(np.zeros((3,100)))
        t = np.linspace(0,1,100)
        temp[0,:] = 0.3+0.0*t+ii*0.1
        temp[1,:] = 0.3+0.1*np.sin(t/5.0)+ii*0.1
        temp[2,:] = 0.3+0.3*np.sin(t/10.0)+ii*0.1
        demos.append(temp)

    FPCA(demos)
if __name__ == "__main__":
    FPCA()





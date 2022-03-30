#!/usr/bin/env python

""" Performs FPCA using stacked, concatenated PCA
 Created: 01/21/2021
"""

__author__ = "Mike Hagenow"

import numpy as np
from scipy import signal
from scipy.ndimage import interpolation
import matplotlib
import matplotlib.pyplot as plt

def PerSamplePCA(data_in, var_ranges,plotting=False):
    # data_in will be a list of d demonstrations, each of which is a mxn matrix
    # for FPCA in multiple dimensions, it will be done on the concatenation of all variables
    # into a single vector for each demonstration (dx(mn))

    # the output will ultimately be a list of mxn matrices, each of which corresponds to a functional principal component

    num_variables = np.shape(data_in[0])[0]  # number of rows in the demo
    num_demos = len(data_in)
    num_samples = np.shape(data_in[0])[1]

    # Step 1: remove the means (prerequsite for PCA/FPCA)
    # loop over each variable

    fpcs = []
    for ii in range(0,3):
        fpcs.append(np.zeros((num_variables,num_samples)))

    for ii in range(0,num_samples):
        # if ii%10==0:
        #     print(str(ii)+" of "+str(num_samples))
        temp = np.zeros((num_demos,num_variables))
        for jj in range(0,num_demos):
            temp[jj,:]=data_in[jj][:,ii]/var_ranges
        temp = temp - np.mean(temp,axis=0)
        U, s, Vt = np.linalg.svd(temp, full_matrices=False)

        flip = [1,1,1]

        num_PCs = np.min([num_demos-1, 3])

        for xx in range(0,num_PCs):
            if ii>0:
                if np.dot(Vt_old[xx],Vt[xx])<0.0:
                    flip[xx] = -1

            fpcs[xx][:, ii] = flip[xx]*3.0 * s[xx] * (1.0/np.sqrt(num_demos)) * Vt[xx] * var_ranges

        Vt_old = []
        for xx in range(0,num_PCs):
            Vt_old.append(flip[xx]*Vt[xx])



    ######################################
    # From here on out is just plotting ##
    ######################################
    if plotting:
        matplotlib.rcParams['pdf.fonttype'] = 42
        # matplotlib.rcParams['ps.fonttype'] = 42
        # matplotlib.rcParams['text.usetex'] = True
        labelsy=['$u$','$v$','$f_{n}$','$v_{tool}$','$\Delta n$',r"$\theta_{u}$",r"$\theta_{v}$"]
        # import matplotlib.font_manager as font_manager
        # font_dirs = ['/home/mike/Desktop/minionproiit']
        # font_files = font_manager.findSystemFonts(fontpaths=font_dirs)
        # font_list = font_manager.createFontList(font_files)
        # font_manager.fontManager.ttflist.extend(font_list)
        # matplotlib.rcParams['font.family'] = 'Minion Pro'
        import matplotlib.font_manager as fm
        minionpro = fm.FontProperties(fname='/home/mike/Desktop/minionproiit/MinionPro-Regular.otf')
        fig, ax = plt.subplots(nrows=num_variables-2, ncols=1, figsize=((13/2.54),(7/2.54)))

        for row_id in range(0,len(ax)):
            row = ax[row_id]

            temp2 = np.zeros((num_demos, num_samples))
            for demo_ind in range(num_demos):
                temp2[demo_ind, :] = data_in[demo_ind][row_id,:]

            temp2 = temp2 - np.mean(temp2, axis=0)
            for demo_ind in range(num_demos):
                row.plot(range(0,num_samples),temp2[demo_ind,:],color='gray')
            row.plot(fpcs[0][row_id,:],color='blue',label='PC1')
            row.plot(fpcs[1][row_id, :], color='green',label='PC2')
            row.plot(fpcs[2][row_id, :], color='red',label='PC3')
            # plt.locator_params(axis="y", nbins=2)
            row.set_ylabel(labelsy[row_id], fontproperties=minionpro)
            row.margins(0, 0)
            row.tick_params(axis='y',labelsize='medium')

            if row_id==2:
                row.set_yticks([0, 10])
            if row_id == 1:
                row.set_yticks([0, 0.1])
            if row_id == 3:
                row.set_yticks([0, 2.5])
            if row_id<4:
                row.tick_params(
                    axis='x',  # changes apply to the x-axis
                    which='both',  # both major and minor ticks are affected
                    bottom=False,  # ticks along the bottom edge are off
                    top=False,  # ticks along the top edge are off
                    labelbottom=False)  # labels along the bottom edge are off
            if row_id==4:
                row.set_xlabel("Sample (n)", fontproperties=minionpro)
            if row_id==4:
                plt.legend(bbox_to_anchor=(0.0, 5.50), loc='upper left', ncol=3, fontsize='small')


        # plt.show()
        plt.subplots_adjust(top=1, bottom=0.0, right=1, left=0,
                            hspace=0.1, wspace=0)
        #
        fig.align_ylabels(ax)
        plt.savefig('pchybrid.pdf',bbox_inches='tight',pad_inches=0.025)
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

    PerSamplePCA(demos)
if __name__ == "__main__":
    PerSamplePCA()





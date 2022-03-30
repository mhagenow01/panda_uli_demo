"""Routine computes transformation between two rigid frames by use of optimization between
two sets of data in the frames.

 Last Updated: 02/06/2019
"""

__author__ = "Mike Hagenow"

import numpy as np
from scipy.optimize import minimize
from scipy.spatial import procrustes


def rigid_registration_from_svd(source_frame, destination_frame):
    """
        Implements the method for finding a transform given two different frames (frame {1} to {2} using
        the result described in "Least Squares Rigid Motion Using SVD" by  Sorkine-Hornung and Rabinovich
        :param source_frame: coordinate data in frame {1}
        :param destination_frame: coordinate data in {2}
        :type p_array: array of xyz coordinates
        :type q_array: array of xyz coordinates
        :returns: r_est, t_est
        :rtype: rotation matrix, translation vector
        """

    #Number of points - same # of points in each frame
    num_points=len(source_frame[:,0])

    #Weighted Centroid - All points are weighted equally
    source_average=[np.average(source_frame[:,0]), np.average(source_frame[:,1]), np.average(source_frame[:,2])]
    destination_average = [np.average(destination_frame[:, 0]), np.average(destination_frame[:, 1]), np.average(destination_frame[:, 2])]

    #Centered Vectors
    source_centered=np.zeros((len(source_frame[:,0]),3))
    source_centered[:,0]=[p-source_average[0] for p in source_frame[:,0]]
    source_centered[:,1] = [p - source_average[1] for p in source_frame[:, 1]]
    source_centered[:,2] = [p - source_average[2] for p in source_frame[:, 2]]



    destination_centered=np.zeros((len(destination_frame[:,0]),3))
    destination_centered[:,0]=[p-destination_average[0] for p in destination_frame[:,0]]
    destination_centered[:,1] = [p - destination_average[1] for p in destination_frame[:,1]]
    destination_centered[:,2] = [p - destination_average[2] for p in destination_frame[:,2]]


    #Covariance Matrix
    X=np.transpose(source_centered)
    Ytrans=destination_centered
    W=np.eye(num_points) # Equal weights -> W is the identity matrix
    S=np.matmul(np.matmul(X,W),Ytrans)

    #Singular Value Decomposition
    U,sigma,Vh=np.linalg.svd(S,full_matrices=False)
    V=np.transpose(Vh)

    #Rotation using SVD
    diag = np.eye(3)
    diag[2, 2] = np.linalg.det(np.matmul(U, np.transpose(V)))
    r_est = np.matmul(np.matmul(V, diag), np.transpose(U))

    #Translation from Rotation and Centroids
    t_est=destination_average-np.matmul(r_est,source_average)

    error = 0.0
    for ii in range(0,len(source_frame)):
        error+= np.linalg.norm(r_est @ source_frame[ii,:].reshape((3,1))+ t_est.reshape((3,1))-destination_frame[ii,:].reshape((3,1)))
    print("SVD Registration Error (avg): ",error/len(source_frame))

    return r_est, t_est


############### OLD METHOD - OPTIMIZATION ##############################
#
# def rigid_registration_from_points(source_frame, destination_frame):
#     # source_frame -> points in source frame
#     # destination_frame -> points in destination frame
#     def optim_objective(optim_in):
#         r = optim_in[:3]
#         A = np.reshape(optim_in[3:], (3, 3))
#         return np.sum((np.transpose([r]) + np.dot(A[:, :], np.transpose(source_frame))
#                        - np.transpose(destination_frame)) ** 2)
#
#     def optim_constraint(optim_in):
#         A = np.reshape(optim_in[3:], (3, 3))
#         return np.linalg.norm(np.dot(A, np.transpose(A)) - np.eye(3))
#
#     constraints = {}
#     constraints['type'] = 'eq'
#     constraints['fun'] = optim_constraint
#
#     sol = minimize(optim_objective, np.ones(12), constraints=constraints, method='SLSQP',
#                    options={'disp': True,'maxiter':1e4})
#     r_est = sol.x[:3]
#     A_est = np.reshape(sol.x[3:], (3, 3))
#     return A_est, r_est, sol.fun/len(source_frame)



def test_rigid():
    numpts=10000
    source=np.random.rand(numpts,3)
    r_mat=np.array([[0.866, -0.5, 0],[0.5,0.866, 0],[0, 0, 1]])
    print("det:",np.linalg.det(r_mat))
    dest=np.zeros((numpts,3))
    t=np.array([0.7, 0.2, 0.4])

    for ii in range(0,numpts):
        dest[ii,:]=np.matmul(r_mat,source[ii,:].reshape((3,1))).reshape(1,3)+t

    r_est, t_est = rigid_registration_from_svd(source,dest)
    # r_est, t_est, paramthree = rigid_registration_from_points(source,dest)
    print("R:Actual")
    print(r_mat)
    print("R:Estimate")
    print(r_est)
    print("T:Actual")
    print(t)
    print("T:Estimate")
    print(t_est)

    print("MOCAP: ", source[0])
    print("ROBOT: ", dest[0])
    est_robot = np.matmul(r_est, source[0].reshape((3, 1))) + t_est.reshape((3, 1))
    print("EST ROBOT: ", est_robot.reshape((1,3))[0])


if __name__ == '__main__':
    test_rigid()

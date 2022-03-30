import numpy as np
from scipy.linalg import logm,expm


def exp_map_np(w):
    w = np.array(w)
    w = w.reshape(3, 1)
    theta = (w[0] ** 2 + w[1] ** 2 + w[2] ** 2) ** 0.5 + 1e-30
    w = w / theta
    w_hat = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    return np.eye(3) + w_hat * np.sin(theta) + np.dot(w_hat, w_hat) * (1 - np.cos(theta))

def skew(w):
    w_hat = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    return w_hat

def random_rotation():
    w = np.random.rand(3)
    w = skew(w)
    A = expm(w)
    return A

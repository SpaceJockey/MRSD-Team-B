""" ___ """
from scipy.optimize import root
import numpy as np


LENGTH = 3


def process(x):
    return x[0, 0] + x[0, 1] * 5


def draw(process, length):
    """ """
    X = np.matrix(np.random.normal(0, 10, (length, 2)))
    y = np.matrix([process(x) for x in X])
    y += np.random.normal(3, 1, len(y))
    return y.T, X.T


def maximum_likelyhood(y, X):
    def objective(b):
        b = np.matrix(b).T
        return np.transpose(np.array((X.T * (y - X * b))))[0]
    x0 = (1, 1)
    res = root(objective, x0=x0)
    return res.x

y, X = draw(process, LENGTH)
X = X.transpose()
b = np.matrix([[0], [1]])
print maximum_likelyhood(y, X)
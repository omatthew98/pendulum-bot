#!/usr/bin/env python

import numpy as np

def system(t, y):
    A = np.array([0, 1],
                [0, 0])
    b = np.array([0, -g * np.sin(y[0]) / r])
    return np.dot(A, y) + b

def x_model():

def y_model():

def z_model():


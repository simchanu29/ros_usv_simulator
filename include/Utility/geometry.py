#!/usr/bin/env python

import numpy as np


def bezier(t, p0, p1, p2, p3):
    return p0 * (1 - t) ** 3 \
           + 3 * p1 * t * (1 - t) ** 2 \
           + 3 * p2 * t ** 2 * (1 - t) \
           + p3 * t ** 3


def angle_deg(a1, a2):
    return np.mod(a1 + a2 + 3*180.0, 360) - 180


def angle_rad(a1, a2):
    return np.mod(a1 + a2 + 3*np.pi, 2*np.pi) - np.pi


def homothetie_vec(vec, theta, x, y):
    # Rotation matrix
    R = np.array([[np.cos(theta), -np.sin(theta), x],
                  [np.sin(theta), np.cos(theta), y],
                  [0, 0, 1]])

    return np.dot(R, vec)


def rotation(vec, theta):
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta), np.cos(theta)]])

    return np.dot(R, vec)


def draw_kayak(theta, x, y):
    # Original
    hull = np.array([[-1.4,  1.0,  1.4, 1.4, 1.0, -1.4, -1.4, -1.4],
                     [-0.4, -0.4, -0.2, 0.2, 0.4,  0.4, -0.4, -0.4],
                     [   1,    1,    1,   1,   1,    1,    1,    1]])
    return homothetie_vec(hull, theta, x, y)

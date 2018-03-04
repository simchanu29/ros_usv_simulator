#!/usr/bin/env python
# coding=utf-8

import numpy as np
import tf.transformations


def extract_bezier_pts(param_dic, maneuvrability):
    """
    Fonction spécifique de cette ia
    :param param_dic:
    :param maneuvrability:
    :return:
    """
    pt0 = np.array([param_dic['origin']['point']['x'],
                    param_dic['origin']['point']['y']])
    x1 = param_dic['origin']['speed']['x'] * maneuvrability + pt0[0]
    y1 = param_dic['origin']['speed']['y'] * maneuvrability + pt0[1]
    pt1 = np.array([x1, y1])

    pt3 = np.array([param_dic['objective']['point']['x'],
                    param_dic['objective']['point']['y']])
    x2 = - param_dic['objective']['speed']['x'] * maneuvrability + pt3[1]
    y2 = - param_dic['objective']['speed']['y'] * maneuvrability + pt3[1]
    pt2 = np.array([x2, y2])

    return pt0, pt1, pt2, pt3


def extract_vel(param_dic):
    vel_1 = np.sqrt(param_dic['origin']['speed']['x']**2
                    + param_dic['origin']['speed']['y']**2)
    vel_2 = np.sqrt(param_dic['objective']['speed']['x'] ** 2
                    + param_dic['objective']['speed']['y'] ** 2)
    return vel_1, vel_2


def get_head_from_pose(pose):
    quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    euler = tf.transformations.euler_from_quaternion(quaternion)
    return euler[2]
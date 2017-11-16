#!/usr/bin/env python
# coding=utf-8

import rospy
import numpy as np
import Utility.geometry as geom
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int16
import pyqtgraph as plt
import tf


class Plotter():
    def __init__(self):
        rospy.init_node('simu_plot')

        # Subscriber
        self.subCmdThrAv = rospy.Subscriber('avant/cmd_thr', Int16, self.updateCmdThrAv)
        self.cmdThrAv = 0
        self.subCmdThrAr = rospy.Subscriber('arriere/cmd_thr', Int16, self.updateCmdThrAr)
        self.cmdThrAr = 0
        self.subPosThrAv = rospy.Subscriber('avant/cmd_pos', Int16, self.updatePosThrAv)
        self.posThrAv = 0
        self.subPosThrAr = rospy.Subscriber('arriere/cmd_pos', Int16, self.updatePosThrAr)
        self.posThrAr = 0
        self.subPose = rospy.Subscriber('pose_real', PoseStamped, self.updatePose)
        self.pose = PoseStamped()
        self.theta = 0.0
        self.x = 0.0
        self.y = 0.0

        # Init plot
        self.win = plt.GraphicsWindow()
        self.fig = self.win.addPlot(title="Display simu")
        self.fig.setXRange(-5, 5)
        self.fig.setYRange(-5, 5)
        self.plt_boat = self.fig.plot()
        # Init trace
        self.plt_trace = self.fig.plot()
        # Init cmd front
        self.plt_cmdfront = self.fig.plot()
        # Init cmd rear
        self.plt_cmdrear = self.fig.plot()


        self.trace = [[], [], []]

    def updateCmdThrAv(self, msg):
        self.cmdThrAv = msg.data

    def updateCmdThrAr(self, msg):
        self.cmdThrAr = msg.data

    def updatePosThrAv(self, msg):
        self.posThrAv = msg.data

    def updatePosThrAr(self, msg):
        self.posThrAr = msg.data

    def updatePose(self, msg):
        self.pose = msg
        self.x = self.pose.pose.position.x
        self.y = self.pose.pose.position.y
        self.theta = tf.transformations.euler_from_quaternion((self.pose.pose.orientation.x,
                                                               self.pose.pose.orientation.y,
                                                               self.pose.pose.orientation.z,
                                                               self.pose.pose.orientation.w))[2]

    def update_trace(self):
        MAX_SIZE = 500
        self.trace[0].append(self.x)
        self.trace[1].append(self.y)
        self.trace[2].append(self.theta)
        if len(self.trace[0]) > MAX_SIZE:
            del (self.trace[0][0])
        if len(self.trace[1]) > MAX_SIZE:
            del (self.trace[1][0])
        if len(self.trace[2]) > MAX_SIZE:
            del (self.trace[2][0])

    def process(self):

        # print "====== Plotting boat"
        hull = geom.draw_kayak(self.theta, self.x, self.y)
        self.plt_boat.setData(hull[0], hull[1], pen=plt.mkPen('l', width=2))
        self.update_trace()
        self.plt_trace.setData(self.trace[0], self.trace[1], pen=plt.mkPen('g'))

        # print "====== Plotting front cmd"
        fr_vec = np.array([[1.2, 1.2+self.cmdThrAv/10.0 * np.cos(self.posThrAv/180.0*np.pi)],
                           [0.0, self.cmdThrAv/10.0 * np.sin(self.posThrAv/180.0*np.pi)],
                           [1.0, 1.0]])
        fr_vec = geom.homothetie_vec(fr_vec, self.theta,
                                     self.x, self.y)
        self.plt_cmdfront.setData(fr_vec[0],
                                  fr_vec[1], pen=plt.mkPen('b'))

        # print "====== Plotting rear cmd"
        re_vec = np.array([[-1.2, -1.2+self.cmdThrAr/10.0 * np.cos(self.posThrAr/180.0*np.pi)],
                           [0.0, self.cmdThrAr/10.0 * np.sin(self.posThrAr/180.0*np.pi)],
                           [1.0, 1.0]])
        re_vec = geom.homothetie_vec(re_vec, self.theta,
                                     self.x, self.y)
        self.plt_cmdrear.setData(re_vec[0],
                                 re_vec[1], pen=plt.mkPen('b'))


if __name__ == '__main__':
    plot = Plotter()
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        plot.process()
        plt.QtGui.QApplication.processEvents()
        rate.sleep()

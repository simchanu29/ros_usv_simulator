#!/usr/bin/env python
# coding=utf-8

import rospy
import numpy as np
import Utility.geometry as geom
import Utility.geodesy as geod
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int16
import pyqtgraph as plt
import tf


class Plotter():
    """
    Affiche les commandes avec un offset afin que l'origine gps spécifiée dans le fichier de config soit à 0
    """
    def __init__(self):

        # Subscriber vars
        self.motors = {}
        self.pose = PoseStamped()
        self.twist = TwistStamped()
        self.theta = float(config['simulated_characteristics']['orientation']['yaw'])

        # Gestion de l'offset à cause de l'UTM : le zero de l'affichage sera le point gps indiqué
        x, y = geod.latlon2meters(float(config['simulated_characteristics']['position']['gps_lon']),
                                  float(config['simulated_characteristics']['position']['gps_lat']))
        self.x_offset = 0.0  # désactivé
        self.y_offset = 0.0  # désactivé

        self.x = float(config['simulated_characteristics']['position']['x']) - self.x_offset
        self.y = float(config['simulated_characteristics']['position']['y']) - self.y_offset
        self.vx = 0.0
        self.vy = 0.0

        # Init plot
        self.win = plt.GraphicsWindow()
        self.fig = self.win.addPlot(title="Display simu")
        self.fig.setXRange(self.x-5, self.x+5)
        self.fig.setYRange(self.y-5, self.y+5)
        self.plt_boat = self.fig.plot()
        self.plt_zone = self.fig.plot()

        # Init trace
        self.plt_trace = self.fig.plot()
        # Init twist
        self.plt_twist = self.fig.plot()

        self.trace = [[], [], []]

        self.authorized_zone_x = np.array(env['environnement']['authorized_zone']['x_offset']) + env['environnement']['authorized_zone']['x_origin']
        self.authorized_zone_y = np.array(env['environnement']['authorized_zone']['y_offset']) + env['environnement']['authorized_zone']['y_origin']
        self.plt_zone.setData(self.authorized_zone_x, self.authorized_zone_y, pen=plt.mkPen('g'))

    def update_motor(self, msg, motor):
        self.motors[motor]['thrust'] = msg.data

    def update_pose(self, msg):
        self.pose = msg
        self.x = self.pose.pose.position.x - self.x_offset
        self.y = self.pose.pose.position.y - self.y_offset
        self.theta = tf.transformations.euler_from_quaternion((self.pose.pose.orientation.x,
                                                               self.pose.pose.orientation.y,
                                                               self.pose.pose.orientation.z,
                                                               self.pose.pose.orientation.w))[2]

    def update_twist(self, msg):
        self.twist = msg
        self.vx = self.twist.twist.linear.x
        self.vy = self.twist.twist.linear.y

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

        # print "====== Plotting twist"
        vec = np.array([[ 0.0, self.vx],
                        [ 0.0, self.vy],
                        [ 1.0, 1.0]])
        vec = geom.homothetie_vec(vec, 0.0,
                                  self.x, self.y)
        self.plt_twist.setData(vec[0],
                               vec[1], pen=plt.mkPen('b', width=2))

        # print "====== Plotting motors"
        for motor in self.motors:
            # print "====== Plotting cmd", motor

            x_mot = float(self.motors[motor]['position']['x'])
            y_mot = - float(self.motors[motor]['position']['y'])
            angle = float(self.motors[motor]['orientation'])
            cmd = float(self.motors[motor]['thrust'])
            plot = self.motors[motor]['plot']

            print "plot cmd :", (cmd-1500)/500.0, "for", motor, "at [", x_mot,",",y_mot,"]"
            vec = np.array([[ x_mot, x_mot + (cmd-1500)/500.0 * np.cos(angle)],
                            [ y_mot, y_mot + (cmd-1500)/500.0 * np.sin(angle)],
                            [1.0, 1.0]])
            vec = geom.homothetie_vec(vec, self.theta,
                                      self.x, self.y)
            plot.setData(vec[0],
                         vec[1], pen=plt.mkPen('r', width=3))

if __name__ == '__main__':
    rospy.init_node('simu_plot')

    config = rospy.get_param('robot')
    env = rospy.get_param('simulation')
    device_types = rospy.get_param('device_types')

    # Remplissage des donnees du type
    for motor in config['actuators']:
        config['actuators'][motor]['type'] = device_types[config['actuators'][motor]['type']]

    plot = Plotter()

    rospy.Subscriber('pose_real', PoseStamped, plot.update_pose)
    rospy.Subscriber('twist_real', TwistStamped, plot.update_twist)
    for motor in config['actuators']:
        if config['actuators'][motor]['type'] != 'None':

            # init dict
            position = config['actuators'][motor]['position']
            orientation = config['actuators'][motor]['orientation']
            plot.motors[motor] = {}
            plot.motors[motor]['thrust'] = 0
            plot.motors[motor]['orientation'] = orientation
            plot.motors[motor]['position'] = position
            plot.motors[motor]['plot'] = plot.fig.plot()

            # sub motor
            pin = config['actuators'][motor]['command']['pwm']['pin']
            print "Subscribing to", pin, "for motor :", motor
            sub_motor = rospy.Subscriber('pwm_out_'+str(pin), Int16, plot.update_motor, motor)

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        plot.process()
        plt.QtGui.QApplication.processEvents()
        rate.sleep()

#!/usr/bin/env python
# coding=utf-8

# Node ROS simu_thruster

# S'abonne aux commande et position moteur pour traduire ça en 
# force et en moment sur le système


import rospy
import numpy as np
from std_msgs.msg import Int16
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Quaternion
import tf

# class T200BlueRobotics():
#
#     def __init__(self,x,y,tension):
#         self.tension = tension
#         self.x = x # coordonnee le long de l'USV
#         self.y = y # perpendiculairement au cap de l'USV
#         self.maxPower = 300
#         self.maxThrustInfo = {16:11.23, 12:7.29}
#         self.maxThrust = self.maxThrustInfo[tension]
#
#         print(self.x, self.y, self.tension, self.maxThrust)
#
#
# class ClassicMotor():
#
#     def __init__(self, x, y, tension):
#         self.tension = tension
#         self.x = x  # coordonnee le long du vehicule
#         self.y = y  # perpendiculairement au cap du vehicule
#         self.maxPower = 50
#         self.motoReduction = 1
#         self.wheelDiameter = 0.15  # m
#         # self.maxThrustInfo = {16: 11.23, 12: 7.29}
#         self.maxThrust = float(tension)/5.0*self.wheelDiameter*self.motoReduction*self.maxPower
#
#         print(self.x, self.y, self.tension, self.maxThrust)


class SimDynMot():

    def __init__(self, config):
        self.config = config

        self.cmd_thrust = 0

        self.orientation = Quaternion()
        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        self.orientation.x = quaternion[0]
        self.orientation.y = quaternion[1]
        self.orientation.z = quaternion[2]
        self.orientation.w = quaternion[3]

        self.wrenchThruster = Wrench()

    def update_orientation(self, msg):
        self.orientation = msg

    def update_cmd_thrust(self, msg):
        self.cmd_thrust = msg.data
        self.process()

    def process_force(self, commande):
        # La commande est de -1 à 1
        # Pour les moteurs on est habituellement sur une courbe quadratique de puissance
        thrust = self.config['type']['max_thrust']*commande*commande

        return thrust

    def process(self):
        cmd_thrust = (self.cmd_thrust-1500)/500.0
        yaw, pitch, roll = tf.transformations.euler_from_quaternion(self.orientation)
        thrust = self.process_force(cmd_thrust)

        self.wrenchThruster.force.x = thrust * np.cos(yaw)
        self.wrenchThruster.force.y = thrust * np.sin(yaw)
        self.wrenchThruster.force.z = 0

        # la force s'applique en (x,y) avec un angle a par rapport au cap du vehicule.
        # x est la coordonnee le long du vehicule
        self.wrenchThruster.torque.z = self.wrenchThruster.force.y*self.config['position']['x'] \
                                       + self.wrenchThruster.force.x*self.config['position']['y']
        self.wrenchThruster.torque.x = self.wrenchThruster.force.z*self.config['position']['y'] \
                                       + self.wrenchThruster.force.y*self.config['position']['z']
        self.wrenchThruster.torque.y = self.wrenchThruster.force.z*self.config['position']['x'] \
                                       + self.wrenchThruster.force.x*self.config['position']['z']

        pub_force.publish(self.wrenchThruster)


if __name__ == '__main__':
    rospy.init_node('simu_thruster')

    # === COMMON ===

    # La node doit se lancer en sachant où chercher sa config
    node_name = rospy.get_name()
    device_type_name = rospy.get_param(node_name+'_type_name')

    # Config
    config_node = rospy.get_param('robot/'+device_type_name+'/'+node_name[6:])
    print config_node

    # Pas sur qu'on en ai besoin mais au cas où
    device_type = config_node['type']
    config_device = rospy.get_param('device_types/'+device_type)
    print config_device

    # === SPECIFIC ===

    # Fusion
    config_node['type'] = config_device

    # Recuperation des parametres
    pin = config_node['command']['pwm']['pin']

    simu = SimDynMot(config_node)

    # sub pub
    sub_yaw = rospy.Subscriber('orientation', Quaternion, simu.update_orientation) # Eventuellement la node qui est l'actionneur placé avant le moteur a son propre temps. Ou sinon il y a une node qui donne le temps et qui synchronise les simulation (mieux)
    sub_pwm_cmd = rospy.Subscriber('pwm_out_'+pin, Int16, simu.update_cmd_thrust)
    pub_force = rospy.Publisher('force', Wrench, queue_size=1)

    rospy.spin()

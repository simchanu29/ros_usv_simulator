#!/usr/bin/env python
# coding=utf-8

# Node ROS simu_thruster

# S'abonne aux commande et position moteur pour traduire ça en
# force et en moment sur le système


import rospy
import numpy as np
from std_msgs.msg import Int16, Float32
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Quaternion
import tf

class SimDynMot():

    def __init__(self, config):
        self.config = config

        # initialisation des commandes
        self.cmd_thrust = 0

        self.orientation = Quaternion()
        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        self.orientation.x = quaternion[0]
        self.orientation.y = quaternion[1]
        self.orientation.z = quaternion[2]
        self.orientation.w = quaternion[3]

        self.wrenchThruster = WrenchStamped()
        self.power_load = 0.0  # W
        self.voltage = 0.0  # V

    def update_orientation(self, msg):
        self.orientation = msg

    def update_cmd_thrust(self, msg):
        self.cmd_thrust = msg.data
        self.process()

    def update_voltage(self, msg):
        self.voltage = msg.data

    def process_force(self, commande):
        # La commande est de -1 à 1
        # Pour les moteurs on est habituellement sur une courbe quadratique de puissance
        y = 0.3
        thrust = self.config['type']['max_force']*(y*np.sign(commande)*commande**2 + (1-y)*commande)

        return thrust

    def process_power_load(self, commande):

        # Mesures empiriques
        y = 0.1
        gain_power = self.config['type']['gain_power_tension']*self.voltage + self.config['type']['offset_power_tension']
        power = gain_power*(y*commande**2 + (1-y)*abs(commande))

        return power

    def process(self):
        # Traduction pwm->[-1;1]
        cmd_thrust = (self.cmd_thrust-1500)/500.0

        # Gestion des quaternions
        print 'self.orientation:', self.orientation, self.orientation.__class__
        quaternion = (self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[0]
        pitch = euler[1]
        roll = euler[2]

        # Calcul de la force
        thrust = self.process_force(cmd_thrust)
        self.wrenchThruster.wrench.force.x = thrust * np.cos(yaw)
        self.wrenchThruster.wrench.force.y = thrust * np.sin(yaw)
        self.wrenchThruster.wrench.force.z = 0

        # la force s'applique en (x,y) avec un angle a par rapport au cap du vehicule.
        # x est la coordonnee le long du vehicule
        self.wrenchThruster.wrench.torque.z = self.wrenchThruster.wrench.force.y*self.config['position']['x'] \
                                            + self.wrenchThruster.wrench.force.x*self.config['position']['y']

        # Calcul de la puissance demandee par les moteurs
        power_load = self.process_power_load(cmd_thrust)

        pub_force.publish(self.wrenchThruster)
        pub_power_load.publish(power_load)


if __name__ == '__main__':
    rospy.init_node('simu_thruster')

    # === COMMON ===

    # Necessite le formalisme de configuration de ros_usv_simulator

    # La node doit se lancer en sachant où chercher sa config
    node_name = rospy.get_name()
    device_type_name = rospy.get_param(node_name + '_type_name')

    # Config
    ns = rospy.get_namespace()
    nslen = len(ns)
    prefix_len = nslen + 5  # On enlève le namespace et simu_
    reduced_node_name = node_name[prefix_len:]
    config_node = rospy.get_param('robot/' + device_type_name + '/' + reduced_node_name)
    print 'CONFIG NODE '+reduced_node_name
    print config_node

    # Pas sur qu'on en ai besoin mais au cas où
    device_type = config_node['type']
    config_device = rospy.get_param('device_types/' + device_type)
    print 'CONFIG DEVICE '+reduced_node_name
    print config_device

    # === SPECIFIC ===

    # Fusion
    config_node['type'] = config_device

    # Recuperation des parametres
    pin = config_node['command']['pwm']['pin']

    simu = SimDynMot(config_node)

    # sub pub
    sub_yaw = rospy.Subscriber('orientation', Quaternion, simu.update_orientation) # Eventuellement la node qui est l'actionneur placé avant le moteur a son propre temps. Ou sinon il y a une node qui donne le temps et qui synchronise les simulation (mieux)
    sub_pwm_cmd = rospy.Subscriber('pwm_out_'+str(pin), Int16, simu.update_cmd_thrust)
    sub_voltage = rospy.Subscriber(reduced_node_name+'_voltage', Float32, simu.update_voltage)
    pub_force = rospy.Publisher('force_'+reduced_node_name, WrenchStamped, queue_size=1)
    pub_power_load = rospy.Publisher(reduced_node_name+'_power_load', Float32, queue_size=1)

    rospy.spin()

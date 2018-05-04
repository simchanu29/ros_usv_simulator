#!/usr/bin/env python
# coding=utf-8

import rospy
import numpy as np
from std_msgs.msg import Float32, Float64


# S'abonne au consommations des actuateurs
# Publie la consommation distribuée sur les batteries


class Input:  # Batterie
    def __init__(self, battery):
        self.sub = rospy.Subscriber(str(battery) + '_voltage_real', Float32, self.cb)
        self.pub = rospy.Publisher(str(battery) + '_power_load', Float32, queue_size=1)
        self.voltage = 0.0

    def cb(self, msg):
        self.voltage = msg.data


class Output:  # Motor
    def __init__(self, motor):
        self.sub = rospy.Subscriber(str(motor) + '_power_load', Float32, self.cb)
        self.pub = rospy.Publisher(str(motor) + '_voltage', Float32, queue_size=1)
        self.power = 0.0

    def cb(self, msg):
        self.power = msg.data


class PowerDistributionSystem:
    def __init__(self, name, motor_list, battery_list):
        # self.motor_list = motor_list
        self.name = name
        self.resistance = config_node['resistance']
        self.max_current = config_node['max_current']

        # s'abonne à tout les actuateurs et à toutes les batteries
        self.motors = []
        for motor in motor_list:
            self.motors.append(Output(motor))
        self.batteries = []
        for battery in battery_list:
            self.batteries.append(Input(battery))

    def update_dt(self, msg):

        # calcul de la puissance consommee
        tot_power = 0
        for motor in self.motors:
            tot_power += motor.power

        # calcul de la tension délivrée par les batteries
        moy_voltage = 0
        for battery in self.batteries:
            moy_voltage += battery.voltage
        moy_voltage /= len(self.batteries)

        # calcul du courant demandé par les moteurs
        if moy_voltage == 0:
            tot_current = 0
        else:
            tot_current = tot_power / moy_voltage

        # calcul de la puissance thermique degagee
        tot_power += self.resistance * tot_current ** 2

        # publie la tension moyenne à tout les équipements
        for motor in self.motors:
            motor.pub.publish(moy_voltage)

        # publie le power load à toute les batteries
        for battery in self.batteries:
            battery.pub.publish(tot_power / len(self.batteries))


if __name__ == '__main__':
    rospy.init_node('simu_attopilot')

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
    print 'CONFIG NODE ' + reduced_node_name
    print config_node

    # Pas sur qu'on en ai besoin mais au cas où
    device_type = config_node['type']
    config_device = rospy.get_param('device_types/' + device_type)
    print 'CONFIG DEVICE ' + reduced_node_name
    print config_device

    # === SPECIFIC ===

    motor_list = rospy.get_param('robot/actuators')
    battery_list = rospy.get_param('robot/power_source')

    system = PowerDistributionSystem(reduced_node_name, motor_list, battery_list)
    sub_dt = rospy.Subscriber('dt', Float64, system.update_dt)

    rospy.spin()

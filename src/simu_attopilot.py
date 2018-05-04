#!/usr/bin/env python
# coding=utf-8

import rospy
import numpy as np
from std_msgs.msg import Float32, Int16

# S'abonne au voltage des batteries
# Publie deux valeurs analogiques de 0 a 5V


def cb_voltage(msg):
    pub_voltage.publish(int(msg.data*voltage_gain))


def cb_current(msg):
    pub_current.publish(int(msg.data*voltage_gain))


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
    print 'CONFIG NODE '+reduced_node_name
    print config_node

    # Pas sur qu'on en ai besoin mais au cas où
    device_type = config_node['type']
    config_device = rospy.get_param('device_types/' + device_type)
    print 'CONFIG DEVICE '+reduced_node_name
    print config_device

    # === SPECIFIC ===

    voltage_gain = config_device['voltage_gain']
    current_gain = config_device['current_gain']
    battery_name = config_node['battery']

    sub_name_voltage = rospy.get_param('~sub_voltage', default=str(battery_name)+'_voltage_real')
    sub_name_current = rospy.get_param('~sub_current', default=str(battery_name)+'_current_real')
    pub_name_voltage = rospy.get_param('~pub_voltage', default=str(battery_name)+'_voltage_raw')
    pub_name_current = rospy.get_param('~pub_current', default=str(battery_name)+'_current_raw')

    sub_voltage = rospy.Subscriber(sub_name_voltage, Float32, cb_voltage)
    sub_current = rospy.Subscriber(sub_name_current, Float32, cb_current)
    pub_voltage = rospy.Publisher(pub_name_voltage, Int16, queue_size=1)
    pub_current = rospy.Publisher(pub_name_current, Int16, queue_size=1)

    rospy.spin()

#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import Float32, Float64

# s'abonne au courant des moteurs
# publie un voltage
# publie un courant


class CurrentOutput:
    def __init__(self, name):
        self.sub = rospy.Subscriber(str(name)+'_power_load', Float32, self.cb)
        self.power = 0

    def cb(self, msg):
        self.power = msg.data


class Battery:
    def __init__(self, name, init_voltage):
        self.name = name

        self.power_load = CurrentOutput(name)

        self.voltage = init_voltage
        self.energy = (init_voltage-min_voltage)/(max_voltage-min_voltage)*max_energy
        self.pub_voltage = rospy.Publisher(self.name+'_voltage_real', Float32, queue_size=1)
        self.pub_current = rospy.Publisher(self.name+'_current_real', Float32, queue_size=1)

    def update_dt(self, msg):
        print '___'
        print 'dt='+str(msg.data)
        tot_power = self.power_load.power
        print 'tot_power='+str(tot_power)
        tot_current = tot_power/self.voltage  # en A
        print 'tot_current='+str(tot_current)
        delta_energy = tot_current*msg.data/3600.0  # en Ah
        print 'delta_energy='+str(delta_energy)
        self.energy -= delta_energy
        print 'energy='+str(self.energy)
        self.voltage = self.compute_voltage(self.energy)
        print 'self.voltage='+str(self.voltage)

        self.pub_current.publish(tot_current)
        self.pub_voltage.publish(self.voltage)

    def compute_voltage(self, energy):
        return energy/max_energy*(max_voltage-min_voltage)+min_voltage


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

    # s'abonne à tout les moteurs
    max_energy = float(config_device['max_energy'])
    max_voltage = float(config_device['max_voltage'])
    min_voltage = float(config_device['min_voltage'])

    init_voltage = config_node['start_voltage']
    batterie = Battery(reduced_node_name, init_voltage)

    sub_dt = rospy.Subscriber('dt', Float64, batterie.update_dt)

    rospy.spin()


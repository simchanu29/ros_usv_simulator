#!/usr/bin/env python
# coding=utf-8

# Node ROS simu_pwm board

"""
S'abonne aux commandes de PWM
Renvoie sur le topic associé au moteur la commande (-100 à 100)

En fait cette node sert juste de hub de redistribution au niveau de la simulation.
En effet dans le simulateur chaque moteur va écouter sur son canal propre.

Cette node ne peut être utilisée que si la node de hardware ros_maestro est installée.
De manière générale ce sera souvent le cas qu'un driver simulé ne puisse fonctionner que si sa contrepartie réelle est
installée.
"""

import rospy
import numpy as np
#from ros_adafruit_pwm_driver.msg import PwmCmd
from ros_maestro.msg import PwmCmd
from std_msgs.msg import Int16

class PwmOutput():
    def __init__(self, pin):
        # Define publisher
        self.pub = rospy.Publisher('pwm_out_'+str(pin), Int16, queue_size=1)

    def publish(self, pwm):
        self.pub.publish(pwm)

class SimPWMBoard():
    def __init__(self, nb_pin):
        self.out_tab = []

        # Output
        for pin in range(nb_pin):
            self.out_tab.append(PwmOutput(pin))

        # Inputs
        self.sub = rospy.Subscriber('/pwm_cmd', PwmCmd, self.update_cmd_pwm)

    def update_cmd_pwm(self, msg):
        pin = int(msg.pin)
        # Gère les messages entre -100 et 100
        if np.abs(msg.command)>100.0:
            msg.command = 0.0
        self.out_tab[pin].publish(msg.command/2.0*10+1500)


if __name__ == '__main__':
    rospy.init_node('simu_maestro')

    # === COMMON ===

    # La node doit se lancer en sachant où chercher sa config
    node_name = rospy.get_name()
    device_type_name = rospy.get_param(node_name+'_type_name')

    # Config
    ns = rospy.get_namespace()
    nslen = len(ns)
    prefix_len = nslen + 5 # On enlève le namespace et simu_
    config_node = rospy.get_param('robot/'+device_type_name+'/'+node_name[prefix_len:])
    print config_node

    # Pas sur qu'on en ai besoin mais au cas où
    device_type = config_node['type']
    config_device = rospy.get_param('device_types/'+device_type)
    print config_device

    # === SPECIFIC ===

    # Launch node
    nb_pin = config_device['output_number']
    simu = SimPWMBoard(nb_pin)

    rospy.spin()

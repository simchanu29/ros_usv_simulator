#!/usr/bin/env python
# coding=utf-8

import roslaunch
import rospy
import rosparam

rospy.init_node('sim_launcher')

# import robot structure
robot_def = rospy.get_param('robot')
device_types = rospy.get_param('device_types')

# Init launch
launch = roslaunch.scriptapi.ROSLaunch()
launch.start()
param_server = rosparam.get_param_server()
namespace_simu = rospy.get_param("~namespace_simu", "/")
output_node_name = "simu_battery1"  # name of the node logged on screen
output = None  # init global var
process_dic = {}
device_type_names = ['sensors', 'actuators', 'pre_actuators', 'power_source', 'other']
# device_type_names = ['pre_actuators', 'actuators']

# launch nodes
# TODO gerer plusieurs robots avec des namespaces ?
for device_type_name in device_type_names:
    if robot_def[device_type_name] is not False:  # Si une catégorie est à faux alors elle est passée
        # TODO eliminer ce systeme de categories
        for device in robot_def[device_type_name]:
            device_type = robot_def[device_type_name][device]['type']
            if device_type != "not_simulated":

                node_type = device_types[device_type]['node_type']
                node_package = device_types[device_type]['node_package']
                node_name = 'simu_' + device

                # Output handling
                if node_name == output_node_name:
                    output = 'screen'
                else:
                    output = None

                # Launch
                param_server.setParam(node_name + '_type_name', device_type_name)
                print 'Launching', node_name, '(' + node_type + ')'
                print 'package   : ', node_package
                print 'node_type : ', node_type
                print 'name      : ', node_name
                print 'namespace : ', namespace_simu
                print 'output    : ', output
                node = roslaunch.core.Node(package=node_package,
                                           node_type=node_type,
                                           name=node_name,
                                           namespace=namespace_simu,
                                           output=output)
                process = launch.launch(node)
                process_dic[node_name] = process

rospy.spin()

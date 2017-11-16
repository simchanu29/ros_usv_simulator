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
output_node_name = 'simu_adafruit_pwm_hat'
output = None
process_dic = {}
# device_type_names = ['sensors', 'actuators', 'pre_actuators']
device_type_names = ['pre_actuators']

# launch nodes
for device_type_name in device_type_names:
    for device in robot_def[device_type_name]:

        device_type = robot_def[device_type_name][device]['type']
        if device_type != "not_simulated":

            node_type = device_types[device_type]['node_type']
            node_name = 'simu_'+device
            print 'Launching', node_name, '('+node_type+')'

            # Output handling
            if node_name==output_node_name:
                output='screen'

            # Launch
            param_server.setParam(node_name+'_type_name', device_type_name)
            node = roslaunch.core.Node(package='ros_bprime_drivers',
                                       node_type=node_type,
                                       name=node_name,
                                       output=output)
            process = launch.launch(node)
            process_dic[node_name] = process

rospy.spin()

#!/usr/bin/env python
# coding=utf-8

# Node ROS simu_time

# Gère le temps interne à la simulation afin de synchroniser
# les nodes de simulation
# S'abonne à rien, publie les étapes de temps

import rospy
from std_msgs.msg import Float64

simu_config = rospy.get_param('simulation')

class Time_handler:

    def __init__(self, dt, duration):
        self.time = 0
        self.dt = dt
        self.duration = duration

    def new_step(self):
        if self.time < self.duration:
            self.time += self.dt
            pub.publish(self.time)


if __name__ == '__main__':
    rospy.init_node('simu_kayak')
    r = rospy.Rate(int(1.0/simu_config['dt_reel']))

    time_handler = Time_handler(simu_config['dt'], simu_config['duration'])

    pub = rospy.Publisher('time', Float64, queue_size=1)

    while not rospy.is_shutdown():
        time_handler.new_step()
        r.sleep()


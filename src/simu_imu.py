#!/usr/bin/env python
# coding=utf-8

# Node ROS simu_gps

# S'abonne a la pose reelle
# Renvoie un message imu


import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import AccelStamped
from sensor_msgs.msg import Imu


class SimIMU():
    def __init__(self):
        # Subscriber
        self.pose_sub = rospy.Subscriber('simu/pose_real', PoseStamped, self.update_pose)
        self.twist_sub = rospy.Subscriber('simu/twist_real', TwistStamped, self.update_twist)
        self.wrench_sub = rospy.Subscriber('simu/acc_real', AccelStamped, self.update_acc)

        # Publisher
        self.imu_pub = rospy.Publisher('imu', Imu, queue_size=1)
        self.imu = Imu()

        # Configuration initiale de l'imu
        self.imu.orientation_covariance = [1, 1, 1,
                                           1, 1, 1,
                                           1, 1, 1]
        self.imu.angular_velocity_covariance = [1, 1, 1,
                                                1, 1, 1,
                                                1, 1, 1]
        self.imu.linear_acceleration_covariance = [1, 1, 1,
                                                   1, 1, 1,
                                                   1, 1, 1]

    def update_pose(self, msg):
        self.imu.header = msg.header
        self.imu.orientation = msg.pose.orientation

    def update_twist(self, msg):
        self.imu.angular_velocity = msg.twist.angular
        self.imu.linear_acceleration = msg.twist.angular

    def update_acc(self, msg):
        self.imu.linear_acceleration = msg.accel.linear

    def process(self):
        self.imu_pub.publish(self.imu)


if __name__ == '__main__':
    rospy.init_node('simu_imu')
    r = rospy.Rate(50)

    simu = SimIMU()
    # rospy.spin()
    while not rospy.is_shutdown():
        simu.process()
        r.sleep()

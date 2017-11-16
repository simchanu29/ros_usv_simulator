#!/usr/bin/env python
# coding=utf-8

# Node ROS simu_gps

# S'abonne a la pose reelle
# Renvoie un message gps


import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus


class SimGPS():
    def __init__(self):
        # Subscriber
        self.pose_sub = rospy.Subscriber('simu/pose_real', PoseStamped, self.update_pose)
        self.twist_sub = rospy.Subscriber('simu/twist_real', TwistStamped, self.update_twist)

        # Publisher
        self.gpsfix_pub = rospy.Publisher('fix', NavSatFix, queue_size=1)
        self.gpsfix = NavSatFix()
        self.gpsvel_pub = rospy.Publisher('vel', TwistStamped, queue_size=1)
        self.gpsvel = TwistStamped()

        # Configuration initiale du gps
        self.CONVERSION_FACTOR_GPS = 1852  # m/min d'angle
        self.lat_origin = 60.0
        self.lon_origin = 0.0

        self.gpsfix.status = NavSatStatus(status=NavSatStatus.STATUS_FIX,
                                          service=NavSatStatus.SERVICE_GPS)  # gps simple
        self.gpsfix.position_covariance = [1, 1, 1,
                                           1, 1, 1,
                                           1, 1, 1]
        self.gpsfix.position_covariance_type = self.gpsfix.COVARIANCE_TYPE_UNKNOWN

    def update_pose(self, msg):
        self.gpsfix.header = msg.header
        self.gpsfix.latitude = msg.pose.position.x / 60.0 / self.CONVERSION_FACTOR_GPS \
                               + self.lat_origin
        self.gpsfix.longitude = msg.pose.position.y / 60.0 / self.CONVERSION_FACTOR_GPS \
                                / np.cos(self.lat_origin / 180 * np.pi) + self.lon_origin
        self.gpsfix.altitude = 0.0

    def update_twist(self, msg):
        self.gpsvel = msg

    def process(self):
        self.gpsfix_pub.publish(self.gpsfix)
        self.gpsvel_pub.publish(self.gpsvel)


if __name__ == '__main__':
    rospy.init_node('simu_gps')
    r = rospy.Rate(50)

    simu = SimGPS()
    # rospy.spin()
    while not rospy.is_shutdown():
        simu.process()
        r.sleep()

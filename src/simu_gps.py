#!/usr/bin/env python
# coding=utf-8

# Node ROS simu_gps

# S'abonne a la pose reelle
# Renvoie un message gps


import rospy
import numpy as np
import Utility.geodesy as geod
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus


class SimGPS():
    def __init__(self):
        # Subscriber
        self.pose_sub = rospy.Subscriber('pose_real', PoseStamped, self.update_pose)
        self.twist_sub = rospy.Subscriber('twist_real', TwistStamped, self.update_twist)

        # Publisher
        self.gpsfix_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=1)
        self.gpsfix = NavSatFix()
        self.gpsvel_pub = rospy.Publisher('/gps/vel', TwistStamped, queue_size=1)
        self.gpsvel = TwistStamped()

        # Configuration initiale du gps
        self.gpsfix.status = NavSatStatus(status=NavSatStatus.STATUS_FIX,
                                          service=NavSatStatus.SERVICE_GPS)  # gps simple
        self.gpsfix.position_covariance = [1, 1, 1,
                                           1, 1, 1,
                                           1, 1, 1]
        self.gpsfix.position_covariance_type = self.gpsfix.COVARIANCE_TYPE_UNKNOWN
        self.lon_offset = -4
        self.lat_offset = 48.5

    def update_pose(self, msg):
        self.gpsfix.header = msg.header
        self.gpsfix.latitude, self.gpsfix.longitude = geod.meters2latlon(msg.pose.position.x, msg.pose.position.y)
        # self.gpsfix.latitude += self.lat_offset
        # self.gpsfix.longitude += self.lon_offset
        # self.gpsfix.latitude *= 180.0/np.pi # Passage en degrés
        # self.gpsfix.longitude *= 180.0/np.pi # Passage en degrés
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

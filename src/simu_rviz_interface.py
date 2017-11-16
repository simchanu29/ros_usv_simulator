#!/usr/bin/env python
# coding=utf-8

import rospy
import numpy as np
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from visualization_msgs.msg import Marker

class RvizInterface():


    def __init__(self):
        rospy.init_node('rviz_interface')

        # Subscriber
        self.subCmdThrAv = rospy.Subscriber('avant/commandThr', Int16, self.updateCmdThrAv)
        self.cmdThrAv = 0
        self.subCmdThrAr = rospy.Subscriber('arriere/commandThr', Int16, self.updateCmdThrAr)
        self.cmdThrAr = 0
        self.subPosThrAv = rospy.Subscriber('avant/commandPos', Int16, self.updatePosThrAv)
        self.posThrAv = 0
        self.subPosThrAr = rospy.Subscriber('arriere/commandPos', Int16, self.updatePosThrAr)
        self.posThrAr = 0
        self.subPose2D = rospy.Subscriber('pose_real', Pose2D, self.updatePose)
        self.pose2D = Pose2D(0,0,0)

        # Publisher
        self.pubMarker = rospy.Publisher('visualization_marker', Marker, queue_size=1)

        # Init rviz
        self.marker = Marker()
        self.marker.header.frame_id = "/map"
        self.marker.header.stamp = rospy.Time.now()

        self.marker.ns = "basic_shapes"
        self.marker.id = 0
        self.marker.type = Marker.ARROW

        self.marker.action = Marker.ADD

        self.marker.color.r = 0.0;
        self.marker.color.g = 1.0;
        self.marker.color.b = 0.0;
        self.marker.color.a = 1.0;

        self.marker.lifetime = rospy.Duration();

    def updateCmdThrAv(self, msg):
        self.cmdThrAv = msg.data


    def updateCmdThrAr(self, msg):
        self.cmdThrAr = msg.data


    def updatePosThrAv(self, msg):
        self.posThrAv = msg.data


    def updatePosThrAr(self, msg):
        self.posThrAr = msg.data

    def updatePose(self, msg):
        self.pose2D.x = msg.x
        self.pose2D.y = msg.y
        self.pose2D.theta = msg.theta

    def process(self):
        self.marker.pose.position.x = self.pose2D.x
        self.marker.pose.position.y = self.pose2D.y
        self.marker.pose.position.z = 0.0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = self.pose2D.theta
        self.marker.pose.orientation.w = 1.0

        self.pubMarker.publish(self.marker)

if __name__ == '__main__':
    interface = RvizInterface()
    while not rospy.is_shutdown():
        interface.process()

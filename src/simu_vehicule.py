#!/usr/bin/env python
# coding=utf-8

# Node ROS simu_world

# S'abonne aux forces et moments sur les objet et les simule
# Renvoie une pose et l'etat du monde


import rospy
import numpy as np
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import AccelStamped
from geometry_msgs.msg import Quaternion
import tf


def Msg(parent):
    """
    Dynamic msg class to implement function fill_header
    :param parent:
    :return: message with fill_header
    """

    class Msg(parent):
        def __init__(self):
            super(Msg, self).__init__()
            # print self

        def fill_header(msg, secs=rospy.get_rostime().secs,
                        nsecs=rospy.get_rostime().nsecs, frame_id=''):
            msg.header.stamp.secs = secs
            msg.header.stamp.nsecs = nsecs
            msg.header.frame_id = frame_id

    return Msg()


class Kayak():
    """
    Surface marine vehicule
    """
    def __init__(self, mass=40.0, length=2.8, width=0.75, height=0.4,
                 x=0.0, y=0.0, yaw=0.0, vx=0.0, vy=0.0):
        massDensityOfFluid = 1000.0
        dragCoeff = 0.5
        dragAngCoeff = 0.3

        self.mass = mass  # kg
        self.length = length  # m
        self.width = width  # m
        self.angularMass = (mass * (length * width) ** 2) / 12.0
        self.drag = 0.5 * massDensityOfFluid * height / 2.0 * width * dragCoeff
        self.dragAng = 0.5 * massDensityOfFluid * height / 2.0 * length * dragAngCoeff * length / 2.0

        self.pos = np.array([x, y, 0.0])  # m
        self.v = np.array([vx, vy, 0.0])  # m/s
        self.vAng = 0.0  # rad/s
        self.yaw = yaw  # rad

class Buggy():
    """
    Ground vehicule
    """
    def __init__(self, mass=2.0, length=0.3, width=0.2, height=0.2,
                 x=0.0, y=0.0, yaw=0.0, vx=0.0, vy=0.0):
        frictionCoeff = 1.0
        minForceToMove = 2.0
        electromechanicBrakingTorque = 1.0

        self.mass = mass  # kg
        self.length = length  # m
        self.width = width  # m
        self.angularMass = (mass * (length * width) ** 2) / 12.0

        # On a une commande char avec 4 roues, donc une force de friction par roue
        self.drag = 4*frictionCoeff*self.vx 0.5 * height / 2.0 * width
        self.dragAng = 0.5 * height / 2.0 * length * length / 2.0

        self.pos = np.array([x, y, 0.0])  # m
        self.v = np.array([vx, vy, 0.0])  # m/s
        self.vAng = 0.0  # rad/s
        self.yaw = yaw  # rad

class SimVehicule():
    def __init__(self):

        self.simuTime = 0
        self.dt = 0.1

        self.vehicule = Vehicule()

        # Definitions des messages
        self.msgPose = Msg(PoseStamped)
        self.msgPose.pose.position.z = 0.0
        self.msgTwist = Msg(TwistStamped)
        self.msgTwist.twist.linear.z = 0.0
        self.msgTwist.twist.angular.x = 0.0
        self.msgTwist.twist.angular.y = 0.0
        self.msgAccel = Msg(AccelStamped)

        self.constraints = [Wrench(), Wrench()]

        self.pubPose = rospy.Publisher('pose_real', PoseStamped, queue_size=1)
        self.pubTwist = rospy.Publisher('twist_real', TwistStamped, queue_size=1)
        self.pubAcc = rospy.Publisher('acc_real', AccelStamped, queue_size=1)

        self.subConstraint0 = rospy.Subscriber('avant/simu_force', Wrench, self.update_constraint_0)
        self.subConstraint1 = rospy.Subscriber('arriere/simu_force', Wrench, self.update_constraint_1)

    def update_constraint_0(self, msg):
        self.constraints[0].force = msg.force
        self.constraints[0].torque = msg.torque

    def update_constraint_1(self, msg):
        self.constraints[1].force = msg.force
        self.constraints[1].torque = msg.torque

    def process(self):
        # Mise à jour du temps
        self.simuTime += self.dt

        # Somme des contraintes
        sumForce = np.array([0, 0, 0])
        sumMoment = np.array([0, 0, 0])
        for constraint in self.constraints:
            # Contraintes dans le repère de l'USV
            sumForce[0] += constraint.force.x
            sumForce[1] += constraint.force.y
            sumForce[2] += constraint.force.z
            sumMoment[0] += constraint.torque.x
            sumMoment[1] += constraint.torque.y
            sumMoment[2] += constraint.torque.z

        # Rotation des contraintes dans le repère global
        rotAngle = self.vehicule.yaw
        Rot = np.array([[np.cos(rotAngle), -np.sin(rotAngle), 0],
                        [np.sin(rotAngle), np.cos(rotAngle), 0],
                        [0, 0, 1]])
        sumForce = Rot.dot(sumForce)

        dragForce = self.vehicule.drag * self.vehicule.v ** 2 * np.sign(self.vehicule.v)
        dragForceAng = 2.0 * self.vehicule.dragAng * self.vehicule.vAng ** 2 * np.sign(self.vehicule.vAng)

        # PFD : Accéleration
        linAcc = (sumForce - dragForce) / self.vehicule.mass
        angAcc = (sumMoment[2] - dragForceAng) / self.vehicule.angularMass  # On restreint la matrice d'inertie à l'axe Z

        rospy.loginfo("_____")
        rospy.loginfo("vAng         = %s", self.vehicule.vAng)
        rospy.loginfo("dragForceAng = %s", dragForceAng)
        rospy.loginfo("dragAng       = %s", self.vehicule.dragAng)
        rospy.loginfo("angAcc       = %s", angAcc)

        # Euler : Vitesse
        self.vehicule.v = self.vehicule.v + self.dt * linAcc
        self.vehicule.vAng = self.vehicule.vAng + self.dt * angAcc

        # test
        dragForce = self.vehicule.drag * self.vehicule.v ** 2 * np.sign(self.vehicule.v)
        dragForceAng = 2.0 * self.vehicule.dragAng * self.vehicule.vAng ** 2 * np.sign(self.vehicule.vAng)
        linAcc = (sumForce - dragForce) / self.vehicule.mass
        angAcc = (sumMoment[2] - dragForceAng) / self.vehicule.angularMass  # On se restreint à l'axe Z
        self.vehicule.v = self.vehicule.v + self.dt * linAcc
        self.vehicule.vAng = self.vehicule.vAng + self.dt * angAcc

        # Euler : Position
        self.vehicule.pos = self.vehicule.pos + self.dt * self.vehicule.v
        self.vehicule.yaw = self.vehicule.yaw + self.dt * self.vehicule.vAng

        # Remplissage des messages
        self.msgPose.pose.position.x = self.vehicule.pos[0]  # m
        self.msgPose.pose.position.y = self.vehicule.pos[1]  # m
        self.msgPose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0.0, 0.0, self.vehicule.yaw))
        self.msgTwist.twist.linear.x = self.vehicule.v[0]
        self.msgTwist.twist.linear.y = self.vehicule.v[1]
        self.msgTwist.twist.angular.z = self.vehicule.vAng  # rad/s
        self.msgAccel.accel.linear.x = linAcc[0]
        self.msgAccel.accel.linear.y = linAcc[1]
        self.msgAccel.accel.angular.z = angAcc

        rospy.loginfo("vAng         = %s", self.vehicule.vAng)
        rospy.loginfo("yaw          = %s", self.vehicule.yaw)
        rospy.loginfo("dragForce    = %s", dragForce)
        rospy.loginfo("linAcc       = %s", linAcc)
        rospy.loginfo("v            = %s", self.vehicule.v)
        rospy.loginfo("pos          = %s", self.vehicule.pos)

        # Remplissage des header pour la simulation
        secs = np.fix(self.simuTime)
        nsecs = self.simuTime - np.fix(self.simuTime)
        self.msgPose.fill_header(secs, nsecs)
        self.msgTwist.fill_header(secs, nsecs)
        self.msgAccel.fill_header(secs, nsecs)

        # Publication
        self.pubPose.publish(self.msgPose)
        self.pubTwist.publish(self.msgTwist)
        self.pubAcc.publish(self.msgAccel)


if __name__ == '__main__':
    rospy.init_node('simu_kayak')
    r = rospy.Rate(2)

    simu = SimVehicule()

    # pub sub
    pub_pose =
    pub_twist =
    pub_acc =


    rospy.spin()
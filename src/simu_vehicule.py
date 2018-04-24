#!/usr/bin/env python
# coding=utf-8

# Node ROS simu_world

# S'abonne aux forces et moments sur les objet et les simule
# Renvoie une pose et l'etat du monde
# IMPORTANT, les stamped sont important pour Rviz

import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import WrenchStamped
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


class USV_char():
    """
    Surface marine vehicule
    """
    def __init__(self, config_vehicule, config_simu, actuators, environnement):
        print 'config_vehicule :', config_vehicule
        print 'config_simu :', config_simu
        print 'actuators :', actuators
        print 'environnement :', environnement

        massDensityOfFluid = environnement['fluid_mass_density']
        # dragCoeff = 0.5
        dragCoeffX = 0.05
        dragCoeffY = 4

        # Init dans le repere global
        self.x = config_simu['position']['x']
        self.y = config_simu['position']['y']
        self.z = config_simu['position']['z']
        self.roll = config_simu['orientation']['roll']
        self.pitch = config_simu['orientation']['pitch']
        self.yaw = config_simu['orientation']['yaw']
        self.v_x = config_simu['speed']['v_x']
        self.v_y = config_simu['speed']['v_y']
        self.v_z = config_simu['speed']['v_z']
        self.v_roll = config_simu['speed']['v_roll']
        self.v_pitch = config_simu['speed']['v_pitch']
        self.v_yaw = config_simu['speed']['v_yaw']

        self.mass = config_vehicule['mass']  # kg
        self.length = config_vehicule['length']  # m
        self.width = config_vehicule['width']  # m
        self.height = config_vehicule['height']  # m

        # Matric d'inertie simplifiée
        self.angularMass = (self.mass * (self.length * self.width) ** 2) / 12.0 # Parallepipède rectangle

        # On a une commande char avec plusieurs moteurs, donc une force de friction par moteur
        # drag = résistance de l'eau
        # On considère que le vehicule est à 100% de sa coque dans l'eau ce qui veut dire que ses dimensions sont les dimensions immergées
        # v_yaw est une vitesse angulaire, donc on transforme aussi avec ce coeff la vitesse angulaire en vitesse en bout
        # TODO generaliser les equations avec des matrices
        self.dragX = 0.5 * massDensityOfFluid * self.height * self.width * dragCoeffX
        self.dragY = 0.5 * massDensityOfFluid * self.height * self.length * dragCoeffY
        self.dragAng = 0.5 * massDensityOfFluid * self.height * self.length / 2.0 * dragCoeffY * (self.length / 2.0)**2
        for motor in actuators:
            if actuators[motor]['type'] != 'None':
                print 'motor :', motor
                dist2center = (actuators[motor]['position']['x']**2
                               + actuators[motor]['position']['y']**2
                               + actuators[motor]['position']['z']**2)**0.5

        self.pos = np.array([self.x, self.y, 0.0])  # m
        # Vitesse dans le repère global
        self.v = np.array([self.v_x, self.v_y, 0.0])  # m/s

# class Buggy():
    # """
    # Ground vehicule
    # """
    # def __init__(self, config_vehicule, config_simu, actuators, environnement):
    #     print 'config_vehicule :', config_vehicule
    #     print 'config_simu :', config_simu
    #     print 'actuators :', actuators
    #     print 'environnement :', environnement
    #     massDensityOfFluid = environnement['fluid_mass_density']
    #     dragCoeff = 0.5
    #     dragAngCoeff = 0.3
    #     minForceToMove = 2.0
    #     electromechanicBrakingTorque = 1.0
    #
    #     # Init dans le repere global
    #     self.x = config_simu['position']['x']
    #     self.y = config_simu['position']['y']
    #     self.z = config_simu['position']['z']
    #     self.roll = config_simu['position']['roll']
    #     self.pitch = config_simu['position']['pitch']
    #     self.yaw = config_simu['position']['yaw']
    #     self.v_x = config_simu['speed']['v_x']
    #     self.v_y = config_simu['speed']['v_y']
    #     self.v_z = config_simu['speed']['v_z']
    #     self.v_roll = config_simu['speed']['v_roll']
    #     self.v_pitch = config_simu['speed']['v_pitch']
    #     self.v_yaw = config_simu['speed']['v_yaw']
    #
    #     self.mass = config_vehicule['mass']  # kg
    #     self.length = config_vehicule['length']  # m
    #     self.width = config_vehicule['width']  # m
    #     self.height = config_vehicule['height']  # m
    #
    #     # Matric d'inertie simplifiée
    #     self.angularMass = (self.mass * (self.length * self.width) ** 2) / 12.0 # Parallepipède rectangle
    #
    #     # On a une commande char avec plusieurs moteurs, donc une force de friction par moteur
    #     # drag = résistance de l'air + frottement des roues
    #     # TODO generaliser les equations avec des matrices
    #     self.dragX = 0.5 * massDensityOfFluid * self.height * self.width * dragCoeff
    #     self.dragY = 0.5 * massDensityOfFluid * self.height * self.length * dragCoeff
    #     self.dragAng = 0.5 * massDensityOfFluid * self.height * self.length / 2.0 * dragAngCoeff * self.length / 4.0
    #     for motor in actuators:
    #         if actuators[motor]['type'] != 'None':
    #             print 'motor :', motor
    #             dist2center = (actuators[motor]['position']['x']**2
    #                            + actuators[motor]['position']['y']**2
    #                            + actuators[motor]['position']['z']**2)**0.5
    #
    #             # Force résistante à l'avancement engendré par l'applatissement des roues sur le sol.
    #             self.drag += self.mass*actuators[motor]['type']['kinetic_friction_coeff']*9.81
    #             self.dragAng += self.mass*actuators[motor]['type']['kinetic_friction_coeff']*9.81 * dist2center
    #
    #     self.pos = np.array([self.x, self.y, 0.0])  # m
    #     self.v = np.array([self.v_x, self.v_y, 0.0])  # m/s

class SimVehicule():
    def __init__(self):

        self.vehicule = USV_char(config['characteristics'],
                                config['simulated_characteristics'],
                                config['actuators'],
                                environnement['environnement'])

        # Definitions des messages
        self.msgPose = Msg(PoseStamped)
        self.msgTwist = Msg(TwistStamped)
        self.msgAccel = Msg(AccelStamped)

        self.constraints = {}

    def update_wrench(self, msg, motor):
        self.constraints[motor] = msg

    def update_dt(self, msg):
        simu.process(msg.data)

    def process(self, dt):

        rospy.loginfo("_____")

        # Somme des contraintes
        sumForce = np.array([0.0, 0.0, 0.0])
        sumMoment = np.array([0.0, 0.0, 0.0])
        for constraint in self.constraints:
            # Contraintes dans le repère du vehicule
            sumForce[0] += self.constraints[constraint].wrench.force.x
            sumForce[1] += self.constraints[constraint].wrench.force.y
            sumForce[2] += self.constraints[constraint].wrench.force.z
            sumMoment[0] += self.constraints[constraint].wrench.torque.x
            sumMoment[1] += self.constraints[constraint].wrench.torque.y
            sumMoment[2] += self.constraints[constraint].wrench.torque.z

        rospy.loginfo("V self.constraints = %s", self.constraints)
        rospy.loginfo("V sumForce     = %s", sumForce)
        rospy.loginfo("V sumMoment    = %s", sumMoment)

        # Rotation des contraintes dans le repère global
        rotAngle = self.vehicule.yaw
        Rot = np.array([[np.cos(rotAngle), -np.sin(rotAngle), 0.0],
                        [np.sin(rotAngle), np.cos(rotAngle), 0.0],
                        [0.0, 0.0, 1.0]])
        sumForce = Rot.dot(sumForce)

        # Rotation des vitesse vers le repère vehicule
        rotAngle = -self.vehicule.yaw
        Rot = np.array([[np.cos(rotAngle), -np.sin(rotAngle), 0.0],
                        [np.sin(rotAngle), np.cos(rotAngle), 0.0],
                        [0.0, 0.0, 1.0]])
        veh_v = Rot.dot(self.vehicule.v)

        rospy.loginfo("V vitesse = %s", veh_v)

        # Calcul avec prise en compte de la dérive dans le repère global
        dragForceX = self.vehicule.dragX * veh_v[0]**2 * np.sign(veh_v[0])
        dragForceY = self.vehicule.dragY * veh_v[1]**2 * np.sign(veh_v[1])
        dragForce = np.array([dragForceX,dragForceY,0.0])

        # rospy.loginfo("V dragY        = %s", self.vehicule.dragY)
        # rospy.loginfo("V dragForceY   = %s", dragForceY)
        # rospy.loginfo("V dragForce    = %s", dragForce)

        # Rotation des contraintes vers le repère global
        rotAngle = self.vehicule.yaw
        Rot = np.array([[np.cos(rotAngle), -np.sin(rotAngle), 0],
                        [np.sin(rotAngle), np.cos(rotAngle), 0],
                        [0, 0, 1]])
        dragForce = Rot.dot(dragForce)

        # Calcul trainée pour les rotations
        dragForceAng = self.vehicule.dragAng * self.vehicule.v_yaw ** 2 * np.sign(self.vehicule.v_yaw)

        # Ajout de l'effet de la trainée linéaire sur la trainée angulaire
        # v_yaw_angle = np.arctan2(self.vehicule.v_yaw[1],self.vehicule.v_yaw[0])
        # diff_cap = v_yaw_angle - self.vehicule.yaw


        # PFD : Accéleration
        linAcc = (sumForce - dragForce) / self.vehicule.mass
        angAcc = (sumMoment[2] - dragForceAng) / self.vehicule.angularMass  # On restreint la matrice d'inertie à l'axe Z

        # rospy.loginfo("G v_yaw        = %s", self.vehicule.v_yaw)
        rospy.loginfo("G dragForceAng = %s", dragForceAng)
        # rospy.loginfo("G dragAng      = %s", self.vehicule.dragAng)
        rospy.loginfo("G angAcc       = %s", angAcc)


        # Euler : Vitesse
        self.vehicule.v = self.vehicule.v + dt * linAcc
        self.vehicule.v_yaw = self.vehicule.v_yaw + dt * angAcc

        # test
        # dragForce = self.vehicule.drag * self.vehicule.v ** 2 * np.sign(self.vehicule.v)
        # dragForceAng = self.vehicule.dragAng * self.vehicule.v_yaw**2 * np.sign(self.vehicule.v_yaw)
        # linAcc = (sumForce - dragForce) / self.vehicule.mass
        # angAcc = (sumMoment[2] - dragForceAng) / self.vehicule.angularMass  # On se restreint à l'axe Z
        # self.vehicule.v = self.vehicule.v + dt * linAcc
        # self.vehicule.v_yaw = self.vehicule.v_yaw + dt * angAcc

        # Euler : Position
        self.vehicule.pos = self.vehicule.pos + dt * self.vehicule.v
        self.vehicule.yaw = self.vehicule.yaw + dt * self.vehicule.v_yaw

        # Remplissage des messages
        self.msgPose.pose.position.x = self.vehicule.pos[0]  # m
        self.msgPose.pose.position.y = self.vehicule.pos[1]  # m
        self.msgPose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0.0, 0.0, self.vehicule.yaw))
        self.msgTwist.twist.linear.x = self.vehicule.v[0]
        self.msgTwist.twist.linear.y = self.vehicule.v[1]
        self.msgTwist.twist.angular.z = self.vehicule.v_yaw  # rad/s
        self.msgAccel.accel.linear.x = linAcc[0]
        self.msgAccel.accel.linear.y = linAcc[1]
        self.msgAccel.accel.angular.z = angAcc

        rospy.loginfo("G v_yaw         = %s", self.vehicule.v_yaw)
        rospy.loginfo("G yaw          = %s", self.vehicule.yaw)
        # rospy.loginfo("G dragForce    = %s", dragForce)
        rospy.loginfo("G linAcc       = %s", linAcc)
        # rospy.loginfo("G v            = %s", self.vehicule.v)
        rospy.loginfo("G pos          = %s", self.vehicule.pos)

        # Remplissage des header pour la simulation
        self.msgPose.fill_header(frame_id='map')
        self.msgTwist.fill_header(frame_id='map')
        self.msgAccel.fill_header(frame_id='map')

        # Publication
        pub_pose.publish(self.msgPose)
        pub_twist.publish(self.msgTwist)
        pub_acc.publish(self.msgAccel)


if __name__ == '__main__':
    rospy.init_node('simu_vehicule')

    config = rospy.get_param('robot')
    environnement = rospy.get_param('simulation')
    device_types = rospy.get_param('device_types')

    # Remplissage des donnees du type
    for motor in config['actuators']:
        config['actuators'][motor]['type'] = device_types[config['actuators'][motor]['type']]
    simu = SimVehicule()

    # pub sub
    pub_pose = rospy.Publisher('pose_real', PoseStamped, queue_size=1)
    pub_twist = rospy.Publisher('twist_real', TwistStamped, queue_size=1)
    pub_acc = rospy.Publisher('accel_real', AccelStamped, queue_size=1)
    for motor in config['actuators']:
        if config['actuators'][motor]['type'] != 'None':

            # sub motor
            sub_motor = rospy.Subscriber('force_'+motor, WrenchStamped, simu.update_wrench, motor)

    sub_dt = rospy.Subscriber('dt', Float64, simu.update_dt)

    rospy.spin()

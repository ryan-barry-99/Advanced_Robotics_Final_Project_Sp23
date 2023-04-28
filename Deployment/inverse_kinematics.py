#!/usr/bin/env python

"""
Omnidirectional Inverse Kinematics ROS Node

This node takes in a Twist message containing the target velocities of the robot and uses inverse kinematics to calculate the
target velocities of each wheel (m/s). The resulting wheel velocities are published to separate topics for each wheel.

The robot model is assumed to have a circular base with N omnidirectional wheels. The wheels are evenly spaced around the base,
and the orientation of each wheel is shifted by an angle of 2*pi/N radians relative to the previous wheel. The robot can move in any
direction on a plane.

This code is modular, allowing for different numbers of wheels to be used, and the wheel radius and base width to be easily changed.

Subscribes to:
    velocity/target_velocity (geometry_msgs/Twist): The target velocities of the robot

Publishes to:
    velocity/wheel_velocities/wheelX_target_vel (std_msgs/Float64): The target velocity of wheel X, where X is the wheel number
    
Author: Ryan Barry
Date created: April 23, 2023
"""

import rospy
from robot_constants import constants
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import numpy as np
from math import cos, sin, atan2, pi

# Default values for the class attributes
WHEEL_RADIUS = constants['wheel_radius'] # The radius of the wheels in meters
L = constants['base_width'] # The difference between the wheel and the center of the base in meters
WHEEL_NAMES = constants['wheel_names'] # A list of names for the wheels
BETA = constants['beta']

class OmnidirectionalInverseKinematics:
    def __init__(self, beta=BETA, wheel_radius=WHEEL_RADIUS, L=L, wheel_names=WHEEL_NAMES):
        # Initialize ROS node
        rospy.init_node('inverse_kinematice_node')

        # Define constants and variables
        self.wheel_radius = wheel_radius
        self.L = L
        self.num_wheels = len(wheel_names)
        self.alpha = [pi/self.num_wheels + 2*i*pi/self.num_wheels for i in range(self.num_wheels)]
        self.beta = beta
        self.wheel_names = wheel_names

        # Initialize target velocities and wheel velocities
        self.zeta_dot = np.zeros((3,1))
        self.wheel_vel = np.zeros((self.num_wheels, 1))

        # Initialized Matrices
        self.r_theta = np.eye(self.num_wheels)
        self.J1_list = []
        self.C1_list = []
        self.J2 = np.eye(len(self.num_wheels)) * self.wheel_radius
        for i, _ in enumerate(self.wheel_names):
            self.J1_list.append(np.array([sin(self.alpha[i] + self.beta[i]), cos(self.alpha[i] + self.beta[i]), self.L*cos(self.beta[i])]))
            self.C1_list.append(np.array([cos(self.alpha[i] + self.beta[i]), sin(self.alpha[i] + self.beta[i]), self.L*sin(self.beta[i])]))
        self.J1 = np.array(self.J1_list)
        self.C1 = np.array(self.C1_list)

        # Create subscribers for the target velocities
        rospy.Subscriber('velocity/target_velocity', Twist, callback=self.target_velocity_callback)

        # Create publishers for the wheel velocities
        self.pub = [rospy.Publisher('velocity/wheel_velocities/'+name+'_target_vel', Float64, queue_size=10) for name in wheel_names]


    def target_velocity_callback(self, target_vel):
        self.zeta_dot[0][0] = target_vel.linear.x
        self.zeta_dot[0][1] = target_vel.linear.y
        self.zeta_dot[0][2] = target_vel.angular.z

    def compute_wheel_velocities(self):
        phi = np.inv(self.J2) * self.J1 * self.r_theta * self.zeta_dot

        for i in range(self.num_wheels):
            self.pub[i].publish(phi[i])

if __name__ == '__main__':
    inverse_kinematic_model = OmnidirectionalInverseKinematics()
    while not rospy.is_shutdown():
        inverse_kinematic_model.compute_wheel_velocities()
        rospy.sleep(0.01)

#!/usr/bin/env python

'''
This module provides the `OmnidirectionalKinematics` class which calculates the linear and angular velocities of a three-wheeled omnidirectional robot.
The class uses the `QuadEncoder` class to obtain and publish the instantaneous velocity of the wheels.

Default values for the `OmnidirectionalForwardKinematics` class are defined in the robot_constants module

The module initializes a ROS node and creates publishers for the wheel and overall robot velocities.
The `run()` method of the `OmnidirectionalKinematics` class runs in a loop and uses the `QuadEncoder` class to obtain the instantaneous velocity 
of the wheels every 0.01 seconds, publishes them to ROS topics, and calculates the linear and angular velocities of the robot using the `calculate_robot_velocity()` method.

Author: Ryan Barry
Date created: April 22, 2023
'''


import rospy
from pinout import pinout
from robot_constants import constants
from QuadEncoder import QuadEncoder
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import numpy as np
from math import cos, sin, pi


# Default values for the class attributes
GEAR_RATIO = constants['gear_ratio']  # 4.4:1 Gear ratio motor with 11 tooth drive gear and 33 tooth driven gear
ENC_CPR = constants['enc_cpr']  # 48 CPR encoders
WHEEL_RADIUS = constants['wheel_radius']  # 2 inch radius wheels converted to meters
L = constants['base_width']  # Length from center of the robot to the center of the wheels converted to meters
WHEEL_NAMES = constants['wheel_names']
BETA = constants['beta']


class OmnidirectionalForwardKinematics :
    def __init__(self, BETA=BETA, WHEEL_NAMES=WHEEL_NAMES, GEAR_RATIO=GEAR_RATIO, ENC_CPR=ENC_CPR, WHEEL_RADIUS=WHEEL_RADIUS, L=L):
        """
        Initializes the `OmnidirectionalKinematics` instance.
        :param wheel_names: The names of wheels in the robot.
        :param gear_ratio: The gear ratio of the motor.
        :param enc_cpr: The encoder count per revolution.
        :param wheel_radius: The radius of the wheel in meters.
        :param L: The distance from the center of the robot to the center of the wheels in meters.
        """
        # Initialize ROS node
        rospy.init_node('velocity_node')

        # Define variables
        self.beta = BETA
        self.gear_ratio = GEAR_RATIO
        self.enc_cpr = ENC_CPR
        self.wheel_radius = WHEEL_RADIUS
        self.L = L
        self.wheel_names = WHEEL_NAMES
        self.num_wheels = len(self.wheel_names)
        self.alpha = [pi/self.num_wheels + 2*i*pi/self.num_wheels for i in range(self.num_wheels)]
        self.r_theta = np.eye(self.num_wheels)
        self.J1_list = []
        self.C1_list = []
        self.J2 = np.eye(len(self.num_wheels)) * self.wheel_radius
        for i, wheel_name in enumerate(self.wheel_names):
            self.J1_list.append(np.array([sin(self.alpha[i] + self.beta[i]), cos(self.alpha[i] + self.beta[i]), self.L*cos(self.beta[i])]))
            self.C1_list.append(np.array([cos(self.alpha[i] + self.beta[i]), sin(self.alpha[i] + self.beta[i]), self.L*sin(self.beta[i])]))
        self.J1 = np.array(self.J1_list)
        self.C1 = np.array(self.C1_list)


        # Create publishers for the velocities
        self.wheel_vel_pubs = []
        for i in range(self.num_wheels):
            self.wheel_vel_pubs[i] = rospy.Publisher('velocity/wheel_velocities/'+self.WHEEL_NAME[i]+'_vel', Float64, queue_size=10)
        self.robot_vel_pub = rospy.Publisher('velocity/robot_velocity/current_velocity', Twist, queue_size=10)

        # Create EncoderReader instance
        for i in range(self.num_wheels):
            self.enc[i] = QuadEncoder(encoder_A_pin=pinout[f'encoder_{i+1}_a'], encoder_B_pin=pinout[f'encoder_{i+1}_b'], gear_ratio=self.gear_ratio, wheel_radius=self.wheel_radius, encoder_cpr=self.enc_cpr)

    # Calculates the current linear and angular velocities of the robot
    def calculate_robot_velocity(self, velocities):
        zeta_dot = np.inv(self.r_theta) * self.J1 * self.J2 * np.array(velocities)

        robot_vel = Twist()
        robot_vel.linear.x = zeta_dot[0]
        robot_vel.linear.y = zeta_dot[1]
        robot_vel.angular.z = zeta_dot[2]
        return robot_vel    

    # Uses QuadEncoder class to obtain and publish the instantaneous velocity of the wheels
    def run(self):
        while not rospy.is_shutdown():
            # Measure velocity every 0.01 seconds
            vel = np.empty(0)
            for i in range(self.wheel_vel_pubs):
                vel.append(self.enc[i].get_instantaneous_velocity())
                self.wheel_vel_pubs[i].publish(vel.T)

            self.robot_vel_pub.publish(self.calculate_robot_velocity(vel))
            
            time.sleep(0.01)
            
        # Clean up GPIO pins
        self.enc1.cleanup()
        self.enc2.cleanup()
        self.enc3.cleanup()


if __name__ == '__main__':
    try:
        forward_kinematic_model = OmnidirectionalForwardKinematics()
        forward_kinematic_model.run()
    except rospy.ROSInterruptException:
        forward_kinematic_model.enc1.cleanup()
        forward_kinematic_model.enc2.cleanup()
        forward_kinematic_model.enc3.cleanup()

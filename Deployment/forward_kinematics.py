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
from math import cos, sin


# Default values for the class attributes
GEAR_RATIO = constants['gear_ratio']  # 4.4:1 Gear ratio motor with 11 tooth drive gear and 33 tooth driven gear
ENC_CPR = constants['enc_cpr']  # 48 CPR encoders
WHEEL_RADIUS = constants['wheel_radius']  # 2 inch radius wheels converted to meters
L = constants['base_width']  # Length from center of the robot to the center of the wheels converted to meters


class OmnidirectionalForwardKinematics :
    def __init__(self, GEAR_RATIO=GEAR_RATIO, ENC_CPR=ENC_CPR, WHEEL_RADIUS=WHEEL_RADIUS, L=L):
        """
        Initializes the `OmnidirectionalKinematics` instance.
        :param GEAR_RATIO: The gear ratio of the motor.
        :param ENC_CPR: The encoder count per revolution.
        :param WHEEL_RADIUS: The radius of the wheel in meters.
        :param L: The distance from the center of the robot to the center of the wheels in meters.
        """
        # Initialize ROS node
        rospy.init_node('velocity_node')

        # Define variables
        self.GEAR_RATIO = GEAR_RATIO
        self.ENC_CPR = ENC_CPR
        self.WHEEL_RADIUS = WHEEL_RADIUS
        self.L = L

        # Create publishers for the velocities
        self.pub1 = rospy.Publisher('velocity/wheel_velocities/wheel1_vel', Float64, queue_size=10)
        self.pub2 = rospy.Publisher('velocity/wheel_velocities/wheel2_vel', Float64, queue_size=10)
        self.pub3 = rospy.Publisher('velocity/wheel_velocities/wheel3_vel', Float64, queue_size=10)
        self.pub4 = rospy.Publisher('velocity/robot_velocity/current_velocity', Twist, queue_size=10)

        # Create EncoderReader instance
        self.enc1 = QuadEncoder(encoder_A_pin=pinout['encoder_1_a'], encoder_B_pin=pinout['encoder_1_b'], gear_ratio=self.GEAR_RATIO, wheel_radius=self.WHEEL_RADIUS, encoder_cpr=self.ENC_CPR)
        self.enc2 = QuadEncoder(encoder_A_pin=pinout['encoder_2_a'], encoder_B_pin=pinout['encoder_2_b'], gear_ratio=self.GEAR_RATIO, wheel_radius=self.WHEEL_RADIUS, encoder_cpr=self.ENC_CPR)
        self.enc3 = QuadEncoder(encoder_A_pin=pinout['encoder_3_a'], encoder_B_pin=pinout['encoder_3_b'], gear_ratio=self.GEAR_RATIO, wheel_radius=self.WHEEL_RADIUS, encoder_cpr=self.ENC_CPR)

    # Calculates the current linear and angular velocities of the robot
    def calculate_robot_velocity(self, v1, v2, v3):
        robot_vel = Twist()
        robot_vel.linear.x = cos(30)*v1 + cos(30)*v2 + cos(0)*v3
        robot_vel.linear.y = sin(30)*v1 - sin(30)*v2 + sin(0)*v3
        robot_vel.angular.z = (cos(30)*v1/self.L + cos(30)*v2/self.L - cos(150)*v3/self.L)
        return robot_vel

    # Uses QuadEncoder class to obtain and publish the instantaneous velocity of the wheels
    def run(self):
        while not rospy.is_shutdown():
            # Measure velocity every 0.01 seconds
            vel1 = self.enc1.get_instantaneous_velocity()
            vel2 = self.enc2.get_instantaneous_velocity()
            vel3 = self.enc3.get_instantaneous_velocity()

            time.sleep(0.01)

            # Publish velocity to a ROS topic
            self.pub1.publish(vel1)
            self.pub2.publish(vel2)
            self.pub3.publish(vel3)
            self.pub4.publish(self.calculate_robot_velocity(vel1, vel2, vel3))

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

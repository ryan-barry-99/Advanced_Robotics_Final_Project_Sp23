#!/usr/bin/env python

'''
This module provides the `OmnidirectionalKinematics` class which calculates the linear and angular velocities of a three-wheeled omnidirectional robot.
The class uses the `QuadEncoder` class to obtain and publish the instantaneous velocity of the wheels.

Default values for the `OmnidirectionalKinematics` class are:
    - GEAR_RATIO = 4.4*33/11: 4.4:1 Gear ratio motor with 11 tooth drive gear and 33 tooth driven gear
    - ENC_CPR = 48: 48 CPR encoders
    - WHEEL_RADIUS = 2*.0254: 2 inch radius wheels converted to meters
    - L = 8*.0254: Length from center of the robot to the center of the wheels converted to meters

The module initializes a ROS node and creates publishers for the wheel and overall robot velocities.
The `run()` method of the `OmnidirectionalKinematics` class runs in a loop and uses the `QuadEncoder` class to obtain the instantaneous velocity 
of the wheels every 0.01 seconds, publishes them to ROS topics, and calculates the linear and angular velocities of the robot using the `calculate_robot_velocity()` method.

Author: Ryan Barry
Date created: April 22, 2023
'''


import rospy
from QuadEncoder import QuadEncoder
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from math import cos, sin


class OmnidirectionalKinematics:
    '''
    Default Values for this class (Customize this to your application):
        GEAR_RATIO = 4.4*33/11: 4.4:1 Gear ratio motor with 11 tooth drive gear and 33 tooth driven gear
        ENC_CPR = 48: 48 CPR encoders
        WHEEL_RADIUS = 2*.0254: 2 inch radius wheels converted to meters
        L = 8*.0254: Length from center of the robot to the center of the wheels converted to meters
    '''
    def __init__(self, GEAR_RATIO=4.4*33/11, ENC_CPR=48, WHEEL_RADIUS=2*.0254, L=8*.0254):
        # Initialize ROS node
        rospy.init_node('omnidirectional_robot_node')

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
        self.enc1 = QuadEncoder(encoder_A_pin=11, encoder_B_pin=13, gear_ratio=self.GEAR_RATIO, wheel_radius=self.WHEEL_RADIUS, encoder_cpr=self.ENC_CPR)
        self.enc2 = QuadEncoder(encoder_A_pin=15, encoder_B_pin=16, gear_ratio=self.GEAR_RATIO, wheel_radius=self.WHEEL_RADIUS, encoder_cpr=self.ENC_CPR)
        self.enc3 = QuadEncoder(encoder_A_pin=18, encoder_B_pin=22, gear_ratio=self.GEAR_RATIO, wheel_radius=self.WHEEL_RADIUS, encoder_cpr=self.ENC_CPR)

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

            rospy.sleep(0.01)

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
        omni_kin = OmnidirectionalKinematics()
        omni_kin.run()
    except rospy.ROSInterruptException:
        omni_kin.enc1.cleanup()
        omni_kin.enc2.cleanup()
        omni_kin.enc3.cleanup()

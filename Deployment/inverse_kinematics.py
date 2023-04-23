  
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
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from math import cos, sin, atan2, pi

# Default values for the class attributes
WHEEL_RADIUS = 2*.0254 # The radius of the wheels in meters
L = 8*.0254 # The difference between the wheel and the center of the base in meters
WHEEL_NAMES = ['wheel1', 'wheel2', 'wheel3'] # A list of names for the wheels

class OmnidirectionalInverseKinematics:

    def __init__(self, wheel_radius=WHEEL_RADIUS, L=L, wheel_names=WHEEL_NAMES):
        # Initialize ROS node
        rospy.init_node('omni_inverse_kinematics')

        # Define constants and variables
        self.r = wheel_radius
        self.L = L
        self.N = len(wheel_names)
        self.beta = 2*pi/self.N
        self.wheel_names = wheel_names

        # Initialize target velocities and wheel velocities
        self.vx = 0.0
        self.vy = 0.0
        self.w = 0.0
        self.v = [0.0] * self.N

        # Create subscribers for the target velocities
        rospy.Subscriber('velocity/target_velocity', Twist, self.target_velocity_callback)

        # Create publishers for the wheel velocities
        self.pub = [rospy.Publisher('velocity/wheel_velocities/'+name+'_target_vel', Float64, queue_size=10) for name in wheel_names]


    def target_velocity_callback(self, target_vel):
        self.vx = target_vel.linear.x
        self.vy = target_vel.linear.y
        self.w = target_vel.angular.z

    def compute_wheel_velocities(self):
        if self.vx == 0.0 and self.vy == 0.0:
            return
        
        theta = atan2(self.vy, self.vx)

        for i in range(self.N):
            self.v[i] = (1/self.r) * (self.vx * cos(theta + self.beta*i) + self.vy * sin(theta + self.beta*i) - self.L * self.w)
            self.pub[i].publish(self.v[i])

if __name__ == '__main__':
    inverse_kinematic_model = OmnidirectionalInverseKinematics()
    while not rospy.is_shutdown():
        inverse_kinematic_model.compute_wheel_velocities()
        rospy.sleep(0.01)

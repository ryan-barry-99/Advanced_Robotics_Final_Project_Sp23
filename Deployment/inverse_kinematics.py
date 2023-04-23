#!/usr/bin/env python


import rospy
from QuadEncoder import QuadEncoder
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from math import cos, sin

class OmnidirectionalInverseKinematics:

    def __init__(self, WHEEL_RADIUS=2, L=8*.0254):
        # Initialize ROS node
        rospy.init_node('inverse_kinematics_node')

        # Define variables
        self.WHEEL_RADIUS = WHEEL_RADIUS
        self.L = L

        self.target_velocity = Twist()
        self.target_velocity.linear.x = 0.0
        self.target_velocity.linear.y = 0.0
        self.target_velocity.angular.z = 0.0

        rospy.Subscriber('velocity/target_velocity', Twist, self.target_velocity_callback)
        
        # Create publishers for the velocities
        self.pub1 = rospy.Publisher('velocity/wheel_velocities/wheel1_target_vel', Float64, queue_size=10)
        self.pub2 = rospy.Publisher('velocity/wheel_velocities/wheel2_target_vel', Float64, queue_size=10)
        self.pub3 = rospy.Publisher('velocity/wheel_velocities/wheel3_target_vel', Float64, queue_size=10)

    def target_velocity_callback(self, target_vel):
        self.target_velocity = Twist()
        self.target_velocity.linear.x = target_vel.linear.x
        self.target_velocity.linear.y = target_vel.linear.y
        self.target_velocity.angular.z = target_vel.angular.z



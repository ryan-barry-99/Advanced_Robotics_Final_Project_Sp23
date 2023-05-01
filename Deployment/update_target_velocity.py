#!/usr/bin/env python

"""
UpdateTargetVelocity class
    This module adds the percieved relative velocity of the balloon to the targeted velocity of the robot so the inverse kinematics required to match the trajectory of the balloon can be calculated.

Subscribes to:
    velocity/target_velocity (geometry_msgs/Twist): The target velocities of the robot
    balloon_velocity (geometry_msgs/Vector3): The current velocity of the balloon

Publishes to:
    velocity/wheel_velocities/wheelX_target_vel (std_msgs/Float64): The target velocity of wheel X, where X is the wheel number
    
Author: Ryan Barry
Date created: May 1, 2023
"""

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import numpy as np

class UpdateTargetVelocity:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('update_target_velocity_node')

        # Create a publisher to target velocity
        self.pub = rospy.Publisher('velocity/target_velocity', Twist, queue_size=10)

        # Create subscribers for the target velocitiy and balloon velocity
        rospy.Subscriber('velocity/target_velocity', Twist, callback=self.target_velocity_callback)
        rospy.Subscriber('balloon_velocity', Vector3, self.balloon_velocity_callback)

        # Initia;oze c;ass variables
        self.target_velocity = Twist()
        self.target_velocity.linear.x = 0.0
        self.target_velocity.linear.y = 0.0
        self.target_velocity.angular.z = 0.0

        self.balloon_velocity = Vector3()
        self.balloon_velocity.x = 0.0
        self.balloon_velocity.y = 0.0
        self.balloon_velocity.z = 0.0


    def target_velocity_callback(self, target_vel):
        self.target_velocity.linear.x = target_vel.linear.x
        self.target_velocity.linear.y = target_vel.linear.y
        self.target_velocity.angular.z = target_vel.angular.z


    def balloon_velocity_callback(self, balloon_vel):
        self.balloon_velocity.x = balloon_vel.x
        self.balloon_velocity.y = balloon_vel.y
        self.balloon_velocity.z = balloon_vel.z


    def run(self):
        self.target_velocity.linear.x += self.balloon_velocity.x
        self.target_velocity.linear.y += self.balloon_velocity.y
        self.pub.publish(self.target_velocity)

            
if __name__ == '__main__':
    update_velo = UpdateTargetVelocity()
    while not rospy.is_shutdown():
        update_velo.run()
        rospy.sleep(0.01)

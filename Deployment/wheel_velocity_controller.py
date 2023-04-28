#!/usr/bin/env python

'''
This module provides the `Wheel` class which uses a `PIDController` object to update the direction and speed of each wheel on an omnidirectional robot.
The class subscribes to two ROS topics for the targeted and current velocities of each wheel and uses the `Motor` class to control the direction and speed of each wheel.

Default values for the `Wheel` class are defined in the robot_constants module.

The module initializes a ROS node and creates a list of `Wheel` objects for each wheel on the robot.
The main loop of the script runs until the ROS node is shut down and calls the `update` method of each `Wheel` object to update the direction and speed of each wheel based on the difference between its targeted and current velocities.

Author: Ryan Barry
Date created: April 28, 2023
'''

import rospy
import time
from std_msgs.msg import Float64
import numpy as np
from pinout import pinout
from robot_constants import constants
from pid_controller import PIDController
from motor import Motor

wheel_names = constants['wheel_names']
num_wheels = len(wheel_names)
kp = constants['kp']
ki = constants['ki']
kd = constants['kd']

class Wheel:
    def __init__(self, name, kp, ki, kd, wheel_number):
        self.pid = PIDController(kp, ki, kd)
        self.targeted_velocity = 0
        self.current_velocity = 0
        self.pwm = 0
        self.n = wheel_number
        self.motor = Motor(in1_pin=pinout[f'motor_{self.n+1}_dir_a'], in2_pin=pinout[f'motor_{self.n+1}_dir_b'], pwm_pin=pinout[f'motor_{self.n+1}_pwm'])

        rospy.Subscriber('velocity/wheel_velocities/'+name+'_target_vel', Float64, self.targeted_velocity_callback)
        rospy.Subscriber('velocity/wheel_velocities/'+name+'_vel', Float64, self.current_velocity_callback)


    def targeted_velocity_callback(self, target_vel):
        self.targeted_velocity = target_vel.data


    def current_velocity_callback(self, current_vel):
        self.current_velocity = current_vel.data


    def update(self):
        self.pwm += self.pid.update(self.targeted_velocity, self.current_velocity)
        if output >= 0:
            self.motor.set_direction('forward')
            self.motor.set_speed(min(self.pwm, 100))
        else:
            self.motor.set_direction('backward')
            self.motor.set_speed(min(-self.pwm, 100))

        self.motor.update_pwm()



if __name__ == '__main__':
    rospy.init_node("wheel_velocity_controller")
    wheels = []

    for i in range(num_wheels):
        wheels.append(Wheel(name=wheel_names[i], kp=kp[i], ki=ki[i], kd=kd[i], wheel_number=i+1))
 
    while not rospy.is_shutdown():
        for wheel in wheels:
            wheel.update()
        time.sleep(0.01)

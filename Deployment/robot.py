#!/usr/bin/env python
import Jetson.GPIO as GPIO
import rospy
from robot_constants import WHEEL_NAMES, NUM_WHEELS, KP, KI, KD
from forward_kinematics import OmnidirectionalForwardKinematics
from inverse_kinematics import OmnidirectionalInverseKinematics
from update_target_velocity import UpdateTargetVelocity
from wheel_velocity_controller import Wheel

class Robot:
    def __init__(self):
        self.forward_kinematic_model = OmnidirectionalForwardKinematics()
        self.inverse_kinematic_model = OmnidirectionalInverseKinematics()
        self.update_velo = UpdateTargetVelocity()
        self.wheels = []
        for i in range(NUM_WHEELS):
            self.wheels.append(Wheel(name=WHEEL_NAMES[i], kp=KP[i], ki=KI[i], kd=KD[i], wheel_number=i+1))

    def wheel_velocity_controller(self):
        for wheel in self.wheels:
            wheel.update()

    def run(self):
        self.update_velo.run()
        self.inverse_kinematic_model.compute_wheel_velocities()
        self.forward_kinematic_model.publish_wheel_velocities()
        self.wheel_velocity_controller()

if __name__ == '__main__':
    robot = Robot()
    try:
        while not rospy.is_shutdown():
            robot.run()
            rospy.sleep(0.01)
    finally:
        GPIO.cleanup()
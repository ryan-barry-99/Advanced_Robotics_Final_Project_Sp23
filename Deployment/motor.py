'''
This module provides the Motor class for controlling the speed and direction of a DC motor using PWM signals.
It uses the Jetson.GPIO library to interface with the GPIO pins of a Jetson Nano.

To use this class, the user must specify the GPIO pin numbers for the motor's IN1, IN2, and PWM pins.

Author: Ryan Barry
Date created: April 27, 2023
'''

import Jetson.GPIO as GPIO
from robot_constants import constants
import time

class Motor:
    def __init__(self, in1_pin, in2_pin, pwm_pin):
        self.in1_pin = in1_pin
        self.in2_pin = in2_pin
        self.pwm_pin = pwm_pin
        self.duty_cycle = 0
        self.frequency = constants['motor_frequency']
        self.pwm_running = False
        self.next_toggle_time = 0
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.in1_pin, GPIO.OUT)
        GPIO.setup(self.in2_pin, GPIO.OUT)
        GPIO.setup(self.pwm_pin, GPIO.OUT)

    def set_direction(self, direction):
        if direction == 'forward':
            GPIO.output(self.in1_pin, GPIO.HIGH)
            GPIO.output(self.in2_pin, GPIO.LOW)
        elif direction == 'backward':
            GPIO.output(self.in1_pin, GPIO.LOW)
            GPIO.output(self.in2_pin, GPIO.HIGH)
        elif direction == 'active_stop':
            GPIO.output(self.in1_pin, GPIO.HIGH)
            GPIO.output(self.in2_pin, GPIO.HIGH)
        elif direction == 'passive_stop':
            GPIO.output(self.in1_pin, GPIO.LOW)
            GPIO.output(self.in2_pin, GPIO.LOW)

    def set_speed(self, duty_cycle):
        self.duty_cycle = duty_cycle

    def update_pwm(self):
        current_time = time.monotonic()
        if current_time >= self.next_toggle_time:
            if GPIO.input(self.pwm_pin) == GPIO.HIGH:
                off_time = 1 / self.frequency * (1 - (self.duty_cycle / 100))
                self.next_toggle_time = current_time + off_time
                GPIO.output(self.pwm_pin, GPIO.LOW)
            else:
                on_time = 1 / self.frequency * (self.duty_cycle / 100)
                self.next_toggle_time = current_time + on_time
                GPIO.output(self.pwm_pin, GPIO.HIGH)

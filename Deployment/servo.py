'''
This module provides the Servo class for controlling the position of a servo using PWM signals.
It uses the Jetson.GPIO library to interface with the GPIO pins of a Jetson Nano.

To use this class, the user must specify the GPIO pin number for signal pin of the servo.

Author: Ryan Barry
Date created: May 1, 2023
'''


import Jetson.GPIO as GPIO
from robot_constants import constants
import time

class Servo:
    def __init__(self, signal_pin)
        self.signal_pin = signal_pin
        self.duty_cycle = 0
        self.frequency = constants['servo_frequency']
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.signal_pin, GPIO.OUT)
        

    def set_position(self, position):
        self.duty_cycle = position      
        current_time = time.monotonic()
        if current_time >= self.next_toggle_time:
            if GPIO.input(self.signal_pin) == GPIO.HIGH:
                off_time = 1 / self.frequency * (1 - (self.duty_cycle / 180))
                self.next_toggle_time = current_time + off_time
                GPIO.output(self.signal_pin, GPIO.LOW)
            else:
                on_time = 1 / self.frequency * (self.duty_cycle / 180)
                self.next_toggle_time = current_time + on_time
                GPIO.output(self.signal_pin, GPIO.HIGH)

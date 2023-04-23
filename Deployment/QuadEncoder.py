'''
This module provides the QuadEncoder class for obtaining position and velocity from an incremental encoder.
It uses the Jetson.GPIO library to interface with the GPIO pins of a Jetson Nano.

To use this class, the user must provide the GPIO pin numbers of the encoder A and B channels, 
the gear ratio of the system, the wheel radius, and the encoder counts per revolution (CPR).

Author: Ryan Barry
Date created: April 22, 2023
'''

import time
import Jetson.GPIO as GPIO
import math.pi as pi

# Quadrature Encoder class to obtain position and velocity from a quadrature encoder
class QuadEncoder:
    def __init__(self, encoder_A_pin, encoder_B_pin, gear_ratio, wheel_radius, encoder_cpr):
        self.encoder_A_pin = encoder_A_pin
        self.encoder_B_pin = encoder_B_pin
        self.gear_ratio = gear_ratio
        self.wheel_radius = wheel_radius
        self.encoder_cpr = encoder_cpr
        self.encoder_pos = 0
        self.last_pos = 0
        self.last_time = 0
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(encoder_A_pin, GPIO.IN)
        GPIO.setup(encoder_B_pin, GPIO.IN)
        GPIO.add_event_detect(encoder_A_pin, GPIO.BOTH, callback=self.encoder_callback)

    # Interrupt to update the encoder count
    def encoder_callback(self, channel):
        if GPIO.input(self.encoder_A_pin) == GPIO.input(self.encoder_B_pin):
            self.encoder_pos += 1
        else:
            self.encoder_pos -= 1

    # Calculates net distance traveled by the wheel between calls of this function
    def get_distance_since_last_call(self):
        current_pos = self.encoder_pos
        distance_per_count = 2 * pi * self.wheel_radius / (self.encoder_cpr * 4 * self.gear_ratio)
        distance_traveled = (current_pos - self.last_pos) * distance_per_count
        self.last_pos = current_pos
        return distance_traveled

    # Calculate average velocity of a wheel over time window size parameter, duration
    def get_wheel_velocity_duration(self, duration):
        start_pos = self.encoder_pos
        start_time = time.monotonic()
        time.sleep(duration)
        end_pos = self.encoder_pos
        end_time = time.monotonic()
        counts_per_second = (end_pos - start_pos) / (end_time - start_time)
        counts_per_revolution = self.encoder_cpr * 4
        wheel_circumference = 2 * pi * self.wheel_radius
        wheel_velocity = counts_per_second * wheel_circumference * self.gear_ratio / counts_per_revolution
        return wheel_velocity

    # Calculate the instantaneous velocity of a wheel between the last two encoder position updates
    def get_instantaneous_velocity(self):
        current_pos = self.encoder_pos
        current_time = time.monotonic()
        counts_per_second = (current_pos - self.last_pos) / (current_time - self.last_time)
        counts_per_revolution = self.encoder_cpr * 4
        wheel_circumference = 2 * pi * self.wheel_radius
        wheel_velocity = counts_per_second * wheel_circumference * self.gear_ratio / counts_per_revolution
        self.last_pos = current_pos
        self.last_time = current_time
        return wheel_velocity

    # Function to reset GPIO pin modes
    def cleanup(self):
        GPIO.cleanup(self.encoder_A_pin)
        GPIO.cleanup(self.encoder_B_pin)

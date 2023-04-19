import time
import Jetson.GPIO as GPIO

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

    def encoder_callback(self, channel):
        if GPIO.input(self.encoder_A_pin) == GPIO.input(self.encoder_B_pin):
            self.encoder_pos += 1
        else:
            self.encoder_pos -= 1

    def get_wheel_velocity_duration(self, duration):
        start_pos = self.encoder_pos
        start_time = time.time()
        time.sleep(duration)
        end_pos = self.encoder_pos
        end_time = time.time()
        counts_per_second = (end_pos - start_pos) / (end_time - start_time)
        counts_per_revolution = self.encoder_cpr * 4
        wheel_circumference = 2 * 3.141592 * self.wheel_radius
        gear_ratio_eff = self.gear_ratio * 11/33
        wheel_velocity = counts_per_second * wheel_circumference * gear_ratio_eff / counts_per_revolution
        return wheel_velocity

    def get_instantaneous_velocity(self):
        current_pos = self.encoder_pos
        current_time = time.time()
        counts_per_second = (current_pos - self.last_pos) / (current_time - self.last_time)
        counts_per_revolution = self.encoder_cpr * 4
        wheel_circumference = 2 * 3.141592 * self.wheel_radius
        gear_ratio_eff = self.gear_ratio * 11/33
        wheel_velocity = counts_per_second * wheel_circumference * gear_ratio_eff / counts_per_revolution
        self.last_pos = current_pos
        self.last_time = current_time
        return wheel_velocity

    def cleanup(self):
        GPIO.cleanup()

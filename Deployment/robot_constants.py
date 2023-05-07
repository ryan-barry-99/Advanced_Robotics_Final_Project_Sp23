# robot_constants.py
# Contains metrics of an individual robot. Updating this file updates constants globally.
# If you change the number of wheels you must change the wheel_names and beta to reflect this change.
# The default values of this dictionary define a 3 wheeled omnidirectional robot with Swedish 90 wheels 

constants = {
    'wheel_radius': 2*0.0254, # Wheel radius in meters
    'base_width': 8*.0254, # Distance from wheel to center of wheels in meters
    'wheel_names': ['wheel1', 'wheel2', 'wheel3'], # A list of names for the wheels
    'gear_ratio': 4.4*33/11, # 4.4:1 Gear ratio motor with 11 tooth drive gear and 33 tooth driven gear
    'enc_cpr': 48, # 48 CPR encoders
    'motor_frequency': 1000, # Set motor PWM frequency to 1 kHz
    'servo_frequency': 1000, # Set servo PWM frequency to 1 kHz
    'beta': [0, 0, 0], # Angle between center of robot and wheel rotation axis for each wheel
    'gamma': [0, 0, 0], # 90 - angle between wheel rotation axis and roller rotation axis for each wheel
    'kp': [1.0, 1.0, 1.0], # Proportional gain for each wheel
    'ki': [0.5, 0.5, 0.5], # Integral gain for each wheel
    'kd': [0.2, 0.2, 0.2] # Derivative gain for each wheel
}

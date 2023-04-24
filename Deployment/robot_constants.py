# robot_constants.py
# Contains metrics of an individual robot. Updating this file updates constants globally.

constants = {
    'wheel_radius': 2*0.0254, # Wheel radius in meters
    'base_width': 8*.0254, # Distance from wheel to center of wheels in meters
    'wheel_names': ['wheel1', 'wheel2', 'wheel3'], # A list of names for the wheels
    'gear_ratio': 4.4*33/11, # 4.4:1 Gear ratio motor with 11 tooth drive gear and 33 tooth driven gear
    'enc_cpr': 48 # 48 CPR encoders
}
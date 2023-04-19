#!/usr/bin/env python

import rospy
from QuadEncoder import QuadEncoder
from std_msgs.msg import Float64

GEAR_RATIO = 4.4*33/11  # 4.4:1 Gear ratio motor with 11 tooth drive gear and 33 tooth driven gear
ENC_CPR = 48    # 48 CPR encoders
WHEEL_RADIUS = 2*.0254  # Convert 2 inch radius wheels to meters

def main():
    # Initialize ROS node
    rospy.init_node('wheel_velocity_node')

    # Create publishers for the velocities
    pub1 = rospy.Publisher('wheel_velocities/wheel1_vel', Float64, queue_size=10)
    pub2 = rospy.Publisher('wheel_velocities/wheel2_vel', Float64, queue_size=10)
    pub3 = rospy.Publisher('wheel_velocities/wheel3_vel', Float64, queue_size=10)


    # Create EncoderReader instance
    enc1 = QuadEncoder(encoder_A_pin=11, encoder_B_pin=13, gear_ratio=GEAR_RATIO, wheel_radius=WHEEL_RADIUS, encoder_cpr=ENC_CPR)
    enc2 = QuadEncoder(encoder_A_pin=15, encoder_B_pin=16, gear_ratio=GEAR_RATIO, wheel_radius=WHEEL_RADIUS, encoder_cpr=ENC_CPR)
    enc3 = QuadEncoder(encoder_A_pin=18, encoder_B_pin=22, gear_ratio=GEAR_RATIO, wheel_radius=WHEEL_RADIUS, encoder_cpr=ENC_CPR)



    while not rospy.is_shutdown():
        # Measure velocity over 0.1 seconds
        vel1 = enc1.get_instantaneous_velocity()
        vel2 = enc2.get_instantaneous_velocity()
        vel3 = enc3.get_instantaneous_velocity()

        # Publish velocity to a ROS topic
        pub1.publish(vel1)
        pub2.publish(vel2)
        pub3.publish(vel3)

    # Clean up GPIO pins
    enc1.cleanup()
    enc2.cleanup()
    enc3.cleanup()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        enc1.cleanup()
        enc2.cleanup()
        enc3.cleanup()

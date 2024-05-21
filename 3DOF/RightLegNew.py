#!/usr/bin/env python3

import time
from math import *
import busio
import RPi.GPIO as GPIO
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import curses

# Servo angle limits
servo_min = 10  # Minimum servo angle (degrees)
servo_max = 170  # Maximum servo angle (degrees)

# Servo offsets Right Front (for your specific configuration)
foot_offset_RF = 30
leg_offset_RF = 0
shoulder_offset_RF = 90

# Servo offsets Right Back (for your specific configuration)
foot_offset_RB = 30
leg_offset_RB = 0
shoulder_offset_RB = 90

# Inverse Kinematics for Right Legs function
def RightLegIK(x, y, z):
    try:

        # Constant lengths in mm
        l3 = 120
        l4 = 120
        l2 = 0
        shoulder_leg = 60

        F=sqrt(x**2+y**2-shoulder_leg**2)
        G=F-l2  
        H=sqrt(G**2+z**2)
        theta1=-atan2(y,x)-atan2(F,-l2)

        D=(H**2-l3**2-l4**2)/(2*l3*l4)
        theta3=acos(D) 

        theta2=atan2(z,G)-atan2(l4*sin(theta3),l3+l4*cos(theta3))

        shoulder_leg = theta1
        leg = -theta2
        foot = theta3

        # Convert radians to degrees and add servo offsets
        foot_RF =  (foot/pi * 180) + foot_offset_RF
        leg_RF = 180 - (leg/pi * 180) + leg_offset_RF
        shoulder_RF = (shoulder_leg/pi * 180) + shoulder_offset_RF

        # Left Back
        foot_RB =  (foot/pi * 180) + foot_offset_RB
        leg_RB = 180 - (leg/pi * 180) + leg_offset_RB
        shoulder_RB =  (shoulder_leg/pi * 180) + shoulder_offset_RB

        # Check if angles are within servo limits
        if (servo_min <= foot_RF <= servo_max) and (servo_min <= leg_RF <= servo_max) and (servo_min <= shoulder_RF <= servo_max) and (servo_min <= foot_RB <= servo_max) and (servo_min <= leg_RB <= servo_max) and (servo_min <= shoulder_RB <= servo_max):
            return foot_RF, leg_RF, shoulder_RF, foot_RB, leg_RB, shoulder_RB
        else:
            print("Error: Servo angles are out of bounds")
            print("Moving to ({}, {}, {})".format(x, y, z))
            print("FootLF angle:", foot_RF)
            print("LegLF angle:", leg_RF)
            print("ShoulderLF angle:", shoulder_RF)

            # print("FootLB angle:", foot_RB)
            # print("LegLB angle:", leg_RB)
            # print("ShoulderLB angle:", shoulder_RB)
            return None, None, None, None, None, None

    except ValueError as e:
        print("Error:", e)
        return None, None, None, None, None, None


# Initialize GPIO
GPIO.setmode(GPIO.BCM)
gpio_port = 18  # Change this to your desired GPIO port
GPIO.setup(gpio_port, GPIO.OUT)
GPIO.output(gpio_port, False)

# Initialize PCA9685
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50

# Set up curses
stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(True)

if __name__ == "__main__":
    try:    
        # Set up PCA9685 servo controller Right Front
        shoulder_servo_RF = servo.Servo(pca.channels[8])
        leg_servo_RF = servo.Servo(pca.channels[9])
        foot_servo_RF = servo.Servo(pca.channels[10])

        # Set up PCA9685 servo controller Right Back
        shoulder_servo_RB = servo.Servo(pca.channels[4])
        leg_servo_RB = servo.Servo(pca.channels[5])
        foot_servo_RB = servo.Servo(pca.channels[6])

        # Set pulse width range for Right servos
        shoulder_servo_RF.set_pulse_width_range(min_pulse=500, max_pulse=2500)
        leg_servo_RF.set_pulse_width_range(min_pulse=500, max_pulse=2500)
        foot_servo_RF.set_pulse_width_range(min_pulse=500, max_pulse=2500)
        shoulder_servo_RB.set_pulse_width_range(min_pulse=500, max_pulse=2500)
        leg_servo_RB.set_pulse_width_range(min_pulse=500, max_pulse=2500)
        foot_servo_RB.set_pulse_width_range(min_pulse=500, max_pulse=2500)

        x = 0
        y = -150
        z = 0

        
        foot_RF, leg_RF, shoulder_RF, foot_RB, leg_RB, shoulder_RB= RightLegIK(x, y, z)
    
        if foot_RF is not None and leg_RF is not None and shoulder_RF is not None and foot_RB is not None and leg_RB is not None and shoulder_RB is not None:
            print("FootRF angle:", foot_RF)
            print("LegRF angle:", leg_RF)
            print("ShoulderRF angle:", shoulder_RF)
            print("FootRB angle:", foot_RB)
            print("LegRB angle:", leg_RB)
            print("ShoulderRB angle:", shoulder_RB)
            # Move servos to calculated angles
            shoulder_servo_RF.angle = shoulder_RF 
            leg_servo_RF.angle = leg_RF 
            foot_servo_RF.angle = foot_RF 
            # Move servos to calculated angles
            shoulder_servo_RB.angle = shoulder_RB
            leg_servo_RB.angle = leg_RB
            foot_servo_RB.angle = foot_RB

    except KeyboardInterrupt:
        pass
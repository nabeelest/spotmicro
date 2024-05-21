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

# Servo offsets Left Front (for your specific configuration)
foot_offset_LF = -30
leg_offset_LF = 0
shoulder_offset_LF = -80

# Servo offsets Left Back (for your specific configuration)
foot_offset_LB = -30
leg_offset_LB = 0
shoulder_offset_LB = -90


def LeftLegIK(x, y, z):
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
        foot_LF =  180 - (foot/pi * 180) + foot_offset_LF
        leg_LF = (leg/pi * 180) + leg_offset_LF
        shoulder_LF = 180 - (shoulder_leg/pi * 180) + shoulder_offset_LF

        # Left Back
        foot_LB = 180 - (foot/pi * 180) + foot_offset_LB
        leg_LB = (leg/pi * 180) + leg_offset_LB
        shoulder_LB = 180 - (shoulder_leg/pi * 180) + shoulder_offset_LB

        # Check if angles are within servo limits
        if (servo_min <= foot_LF <= servo_max) and (servo_min <= leg_LF <= servo_max) and (servo_min <= shoulder_LF <= servo_max) and (servo_min <= foot_LB <= servo_max) and (servo_min <= leg_LB <= servo_max) and (servo_min <= shoulder_LB <= servo_max):
            return foot_LF, leg_LF, shoulder_LF, foot_LB, leg_LB, shoulder_LB
        else:
            print("Error: Servo angles are out of bounds")
            print("Moving to ({}, {}, {})".format(x, y, z))
            print("FootLF angle:", foot_LF)
            print("LegLF angle:", leg_LF)
            print("ShoulderLF angle:", shoulder_LF)

            # print("FootLB angle:", foot_LB)
            # print("LegLB angle:", leg_LB)
            # print("ShoulderLB angle:", shoulder_LB)
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
       # Set up PCA9685 servo controller Left Front
        shoulder_servo_LF = servo.Servo(pca.channels[12])
        leg_servo_LF = servo.Servo(pca.channels[13])
        foot_servo_LF = servo.Servo(pca.channels[14])
        # Set up PCA9685 servo controller Left Back
        shoulder_servo_LB = servo.Servo(pca.channels[0])
        leg_servo_LB = servo.Servo(pca.channels[1])
        foot_servo_LB = servo.Servo(pca.channels[2])

        # Set pulse width range for Left servos
        shoulder_servo_LF.set_pulse_width_range(min_pulse=500, max_pulse=2500)
        leg_servo_LF.set_pulse_width_range(min_pulse=500, max_pulse=2500)
        foot_servo_LF.set_pulse_width_range(min_pulse=500, max_pulse=2500)
        shoulder_servo_LB.set_pulse_width_range(min_pulse=500, max_pulse=2500)
        leg_servo_LB.set_pulse_width_range(min_pulse=500, max_pulse=2500)
        foot_servo_LB.set_pulse_width_range(min_pulse=500, max_pulse=2500)

        x = 0
        y = -150
        z = 0

        
        foot_LF, leg_LF, shoulder_LF, foot_LB, leg_LB, shoulder_LB= LeftLegIK(x, y, z)
    
        if foot_LF is not None and leg_LF is not None and shoulder_LF is not None and foot_LB is not None and leg_LB is not None and shoulder_LB is not None:
            # print("FootRF angle:", foot_RF)
            # print("LegRF angle:", leg_RF)
            # print("ShoulderRF angle:", shoulder_RF)
            # print("FootRB angle:", foot_RB)
            # print("LegRB angle:", leg_RB)
            # print("ShoulderRB angle:", shoulder_RB)
              # Move Left Front servos to calculated angles
            shoulder_servo_LF.angle = shoulder_LF
            leg_servo_LF.angle = leg_LF 
            foot_servo_LF.angle = foot_LF
            # # Move Left Back servos to calculated angles
            shoulder_servo_LB.angle = shoulder_LB 
            leg_servo_LB.angle = leg_LB
            foot_servo_LB.angle = foot_LB

    except KeyboardInterrupt:
        pass
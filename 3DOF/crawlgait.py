#!/usr/bin/env python3

import time
from math import *
import busio
import RPi.GPIO as GPIO
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import numpy as np
import curses

# Servo angle limits
servo_min = 0 # Minimum servo angle (degrees)
servo_max = 180  # Maximum servo angle (degrees)

# Servo offsets Left Front (for your specific configuration)
foot_offset_LF = -30  
leg_offset_LF = 10
shoulder_offset_LF = 5

# Servo offsets Left Back (for your specific configuration)
foot_offset_LB = -30  
leg_offset_LB = 10
shoulder_offset_LB = -5

# Servo offsets Right Front (for your specific configuration)
foot_offset_RF = 30  
leg_offset_RF = -10
shoulder_offset_RF = 10

# Servo offsets Right Back (for your specific configuration)
foot_offset_RB = 30  
leg_offset_RB = -10
shoulder_offset_RB = 10

# Inverse Kinematics for Left Front function
def LeftFrontIK(point):
    try:
        (x, y, z) = (point[0], point[1], point[2])

        # Constant lengths in mm
        upper_leg = 120
        lower_leg = 120
        shoulder_leg = 60

        y1 = sqrt(y*y + z*z - shoulder_leg*shoulder_leg)
        
        distance = sqrt(x*x + y1*y1)

        foot = acos((distance*distance - upper_leg*upper_leg - lower_leg*lower_leg)/(-2*upper_leg*lower_leg))
        
        leg = asin((lower_leg*sin(foot))/distance) - (atan(x/y) if y!=0 else 0)

        shoulder_leg = atan(distance/shoulder_leg) + atan(z/y)
    

        # Convert radians to degrees and add servo offsets
        foot_LF =  (foot/pi * 180) + foot_offset_LF
        leg_LF = (leg/pi * 180) + leg_offset_LF
        shoulder_LF = 180 - (shoulder_leg/pi * 180) + shoulder_offset_LF

        # Check if angles are within servo limits
        if (servo_min <= foot_LF <= servo_max) and (servo_min <= leg_LF <= servo_max) and (servo_min <= shoulder_LF <= servo_max):
            return foot_LF, leg_LF, shoulder_LF
        else:
            print("Error: Servo angles are out of bounds")
            print("Moving to ({}, {}, {})".format(x, y, z))
            print("FootLF angle:", foot_LF)
            print("LegLF angle:", leg_LF)
            print("ShoulderLF angle:", shoulder_LF)
            return None, None, None

    except ValueError as e:
        print("Error:", e)
        return None, None, None

# Inverse Kinematics for Left Back function
def LeftBackIK(point):
    try:
        (x, y, z) = (point[0], point[1], point[2])

        # Constant lengths in mm
        upper_leg = 120
        lower_leg = 120
        shoulder_leg = 60

        y1 = sqrt(y*y + z*z - shoulder_leg*shoulder_leg)
        
        distance = sqrt(x*x + y1*y1)

        foot = acos((distance*distance - upper_leg*upper_leg - lower_leg*lower_leg)/(-2*upper_leg*lower_leg))
        
        leg = asin((lower_leg*sin(foot))/distance) - (atan(x/y) if y!=0 else 0)

        shoulder_leg = atan(distance/shoulder_leg) + atan(z/y)
    

        # Convert radians to degrees and add servo offsets
        foot_LB =  (foot/pi * 180) + foot_offset_LB
        leg_LB = (leg/pi * 180) + leg_offset_LB
        shoulder_LB = 180 - (shoulder_leg/pi * 180) + shoulder_offset_LB

        # Check if angles are within servo limits
        if (servo_min <= foot_LB <= servo_max) and (servo_min <= leg_LB <= servo_max) and (servo_min <= shoulder_LB <= servo_max):
            return foot_LB, leg_LB, shoulder_LB
        else:
            print("Error: Servo angles are out of bounds")
            print("Moving to ({}, {}, {})".format(x, y, z))
            print("FootLB angle:", foot_LB)
            print("LegLB angle:", leg_LB)
            print("ShoulderLB angle:", shoulder_LB)
            return None, None, None

    except ValueError as e:
        print("Error:", e)
        return None, None, None
    

# Inverse Kinematics for Right Back function
def RightBackIK(point):
    try:    
        (x, y, z) = (point[0], point[1], point[2])

        # Constant lengths in mm
        upper_leg = 120
        lower_leg = 120
        shoulder_leg = 60

        y1 = sqrt(y*y + z*z - shoulder_leg*shoulder_leg)
        
        distance = sqrt(x*x + y1*y1)

        foot = acos((distance*distance - upper_leg*upper_leg - lower_leg*lower_leg)/(-2*upper_leg*lower_leg))
        
        leg = asin((lower_leg*sin(foot))/distance) - (atan(x/y) if y!=0 else 0)

        shoulder_leg = atan(distance/shoulder_leg) + atan(z/y)
    

        # Convert radians to degrees and add servo offsets
        foot_RB = 180 - (foot/pi * 180) + foot_offset_RB
        leg_RB = 180 - (leg/pi * 180) + leg_offset_RB
        shoulder_RB = (shoulder_leg/pi * 180) + shoulder_offset_RB

        # Check if angles are within servo limits
        if (servo_min <= foot_RB <= servo_max) and (servo_min <= leg_RB <= servo_max) and (servo_min <= shoulder_RB <= servo_max):
            return foot_RB, leg_RB, shoulder_RB
        else:
            print("Error: Servo angles are out of bounds")
            print("Moving to ({}, {}, {})".format(x, y, z))
            print("FootRB angle:", foot_RB)
            print("LegRB angle:", leg_RB)
            print("ShoulderRB angle:", shoulder_RB)
            return None, None, None
    except ValueError as e:
        print("Error:", e)
        return None, None, None
    
# Inverse Kinematics for Right Front function
def RightFrontIK(point):
    try:
        (x, y, z) = (point[0], point[1], point[2])

        # Constant lengths in mm
        upper_leg = 120
        lower_leg = 120
        shoulder_leg = 60

        y1 = sqrt(y*y + z*z - shoulder_leg*shoulder_leg)
        
        distance = sqrt(x*x + y1*y1)

        foot = acos((distance*distance - upper_leg*upper_leg - lower_leg*lower_leg)/(-2*upper_leg*lower_leg))
        
        leg = asin((lower_leg*sin(foot))/distance) - (atan(x/y) if y!=0 else 0)

        shoulder_leg = atan(distance/shoulder_leg) + atan(z/y)
    

        # Convert radians to degrees and add servo offsets
        foot_RF = 180 - (foot/pi * 180) + foot_offset_RF
        leg_RF = 180 - (leg/pi * 180) + leg_offset_RF
        shoulder_RF = (shoulder_leg/pi * 180) + shoulder_offset_RF

        # Check if angles are within servo limits
        if (servo_min <= foot_RF <= servo_max) and (servo_min <= leg_RF <= servo_max) and (servo_min <= shoulder_RF <= servo_max):
            return foot_RF, leg_RF, shoulder_RF
        else:
            print("Error: Servo angles are out of bounds")
            print("Moving to ({}, {}, {})".format(x, y, z))
            print("FootRF angle:", foot_RF)
            print("LegRF angle:", leg_RF)
            print("ShoulderRF angle:", shoulder_RF)
            return None, None, None
    except ValueError as e:
        print("Error:", e)
        return None, None, None
    

# Bezier curve calculation function
def bezier(t, points):
    n = len(points) - 1
    x = y = 0
    for i, (px, py) in enumerate(points):
        bernstein = comb(n, i) * (1 - t)**(n - i) * t**i
        x += px * bernstein
        y += py * bernstein
    return x, y




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
        leg_servo_LF = servo.Servo(pca.channels[0])
        foot_servo_LF = servo.Servo(pca.channels[1])
        # Set up PCA9685 servo controller Left Back
        shoulder_servo_LB = servo.Servo(pca.channels[8])
        leg_servo_LB = servo.Servo(pca.channels[2])
        foot_servo_LB = servo.Servo(pca.channels[3])

        # Set up PCA9685 servo controller Right Front
        shoulder_servo_RF = servo.Servo(pca.channels[13])
        leg_servo_RF = servo.Servo(pca.channels[4])
        foot_servo_RF = servo.Servo(pca.channels[5])

        # Set up PCA9685 servo controller Right Back
        shoulder_servo_RB = servo.Servo(pca.channels[9])
        leg_servo_RB = servo.Servo(pca.channels[6])
        foot_servo_RB = servo.Servo(pca.channels[7])

        # Set pulse width range for Left servos
        shoulder_servo_LF.set_pulse_width_range(min_pulse=500, max_pulse=2500)
        leg_servo_LF.set_pulse_width_range(min_pulse=500, max_pulse=2500)
        foot_servo_LF.set_pulse_width_range(min_pulse=500, max_pulse=2500)
        shoulder_servo_LB.set_pulse_width_range(min_pulse=500, max_pulse=2500)
        leg_servo_LB.set_pulse_width_range(min_pulse=500, max_pulse=2500)
        foot_servo_LB.set_pulse_width_range(min_pulse=500, max_pulse=2500)

        # Set pulse width range for Right servos
        shoulder_servo_RF.set_pulse_width_range(min_pulse=500, max_pulse=2500)
        leg_servo_RF.set_pulse_width_range(min_pulse=500, max_pulse=2500)
        foot_servo_RF.set_pulse_width_range(min_pulse=500, max_pulse=2500)
        shoulder_servo_RB.set_pulse_width_range(min_pulse=500, max_pulse=2500)
        leg_servo_RB.set_pulse_width_range(min_pulse=500, max_pulse=2500)
        foot_servo_RB.set_pulse_width_range(min_pulse=500, max_pulse=2500)



        x = 0
        y = 150
        z = 60

        
        # Define the Bezier curve control points
        points = [(-100,200), (20, 150), (60, 150), (0, 200)]
        while True:
            #Left Front 
            for t in range(0, 101, 1):   
                t /= 100  # Normalize t to [0, 1]
                (x, y) = bezier(t, points)
                point = [x,y,z]
                foot_LF, leg_LF, shoulder_LF = LeftFrontIK(point)
                foot_LB, leg_LB, shoulder_LB = LeftBackIK(point)
                foot_RF, leg_RF, shoulder_RF = RightFrontIK(point)
                foot_RB, leg_RB, shoulder_RB = RightBackIK(point)

                # if foot_LF is not None and leg_LF is not None and shoulder_LF is not None and foot_LB is not None and leg_LB is not None and shoulder_LB is not None and foot_RF is not None and leg_RF is not None and shoulder_RF is not None and foot_RB is not None and leg_RB is not None and shoulder_RB is not None:
                    
                    # print("Moving to ({}, {}, {})".format(x, y, z))

                    # print("FootLF angle:", foot_LF)
                    # print("LegLF angle:", leg_LF)
                    # print("ShoulderLF angle:", shoulder_LF)

                    # print("FootLB angle:", foot_LB)
                    # print("LegLB angle:", leg_LB)
                    # print("ShoulderLB angle:", shoulder_LB)

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
                time.sleep(0.01)
                    
                    # # Move Right Back servos to calculated angles
                    # shoulder_servo_RB.angle = shoulder_RB
                    # leg_servo_RB.angle = leg_RB
                    # foot_servo_RB.angle = foot_RB
                    # time.sleep(0.01)

                    #  # Move Right Front servos to calculated angles
                    # shoulder_servo_RF.angle = shoulder_RF 
                    # leg_servo_RF.angle = leg_RF 
                    # foot_servo_RF.angle = foot_RF 
                    # time.sleep(0.01)

                    # # Move Left Back servos to calculated angles
                    # shoulder_servo_LB.angle = shoulder_LB
                    # leg_servo_LB.angle = leg_LB
                    # foot_servo_LB.angle = foot_LB
                    # time.sleep(0.01)

            #Right Back
            for t in range(0, 101, 1):   
                t /= 100  # Normalize t to [0, 1]
                (x, y) = bezier(t, points)
                point = [x,y,z]
                foot_RB, leg_RB, shoulder_RB = RightBackIK(point)

                # Move Right Back servos to calculated angles
                shoulder_servo_RB.angle = shoulder_RB
                leg_servo_RB.angle = leg_RB
                foot_servo_RB.angle = foot_RB
                time.sleep(0.01)

            #Right Front
            for t in range(0, 101, 1):   
                t /= 100  # Normalize t to [0, 1]
                (x, y) = bezier(t, points)
                point = [x,y,z]
                foot_RF, leg_RF, shoulder_RF = RightFrontIK(point)
    
                
                # Move Right Front servos to calculated angles
                shoulder_servo_RF.angle = shoulder_RF 
                leg_servo_RF.angle = leg_RF 
                foot_servo_RF.angle = foot_RF 
                time.sleep(0.01)

            #Left Back
            for t in range(0, 101, 1):   
                t /= 100  # Normalize t to [0, 1]
                (x, y) = bezier(t, points)
                point = [x,y,z]
                foot_LB, leg_LB, shoulder_LB = LeftBackIK(point)

                # Move Left Back servos to calculated angles
                shoulder_servo_LB.angle = shoulder_LB
                leg_servo_LB.angle = leg_LB
                foot_servo_LB.angle = foot_LB
                time.sleep(0.01)


    except KeyboardInterrupt:
        pass

#!/usr/bin/env python3

import time
import math
import busio
import RPi.GPIO as GPIO
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import os
import sys

# Length of each link in the manipulator
l1 = 120  # Length of link 1
l2 = 150  # Length of link 2

# Servo angle limits
servo_min = 0  # Minimum servo angle (degrees)
servo_max = 180  # Maximum servo angle (degrees)

# Servo offsets (for your specific configuration)
servo1_offset = 0  # Offset angle for servo 1 (degrees)
servo2_offset = 30  # Offset angle for servo 2 (degrees)

# Convert angles from degrees to radians
def deg_to_rad(deg):
    return math.radians(deg)

# Convert angles from radians to degrees
def rad_to_deg(rad):
    return math.degrees(rad)

# Inverse Kinematics function
def inverse_kinematics(x, y):
    try:
        # Calculate distance from the origin to the end effector
        distance = math.sqrt(x**2 + y**2)

        # Check if the desired point is within reach
        if distance > l1 + l2 or distance < abs(l1 - l2):
            raise ValueError("Point out of reach")

        # Calculate theta2 using the law of cosines
        D = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
        theta2 = math.acos(D)

        # Calculate theta1 using the atan2 function
        theta1 = math.atan2(y, x) - math.atan2((l2 * math.sin(theta2)), (l1 + l2 * math.cos(theta2)))

        # Convert radians to degrees and add servo offsets
        servo1_angle = rad_to_deg(theta1) + servo1_offset
        servo2_angle = rad_to_deg(theta2) + servo2_offset
        # print(servo1_angle)
        # print(servo2_angle)

        # Check if angles are within servo limits
        if (servo_min <= servo1_angle <= servo_max) and (servo_min <= servo2_angle <= servo_max):
            return servo1_angle, servo2_angle
        else:
            print("Error: Servo angles are out of bounds")
            return None, None

    except ValueError as e:
        print("Error:", e)
        return None, None

# Initialize PCA9685
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 60

# Function to move servos to specified angles
def move_servos(angle1, angle2):
    pca.channels[0].duty_cycle = int(servo.Servo.convert_angle_to_duty_cycle(angle1))
    pca.channels[1].duty_cycle = int(servo.Servo.convert_angle_to_duty_cycle(angle2))

# Bezier curve calculation function
def bezier(t, points):
    n = len(points) - 1
    x = y = 0
    for i, (px, py) in enumerate(points):
        bernstein = math.comb(n, i) * (1 - t)**(n - i) * t**i
        x += px * bernstein
        y += py * bernstein
    return x, y

if __name__ == "__main__":
    try:
        # Define the Bezier curve control points
        points = [(100, 250), (76, 167), (162, 106), (204, 174)]

        # Iterate over time (t) to interpolate positions along the curve
        for t in range(0, 101, 1):
            t /= 100  # Normalize t to [0, 1]
            x, y = bezier(t, points)

            # Calculate servo angles using inverse kinematics
            servo1_angle, servo2_angle = inverse_kinematics(x, y)

            if servo1_angle is not None and servo2_angle is not None:
                print("Moving to ({}, {})".format(x, y))
                print("Servo 1 angle:", servo1_angle)
                print("Servo 2 angle:", servo2_angle)

                # Set up PCA9685 servo controller
                active_servo1 = servo.Servo(pca.channels[0])
                active_servo2 = servo.Servo(pca.channels[1])

                # Set pulse width range for servos
                active_servo1.set_pulse_width_range(min_pulse=500, max_pulse=2500)
                active_servo2.set_pulse_width_range(min_pulse=500, max_pulse=2600)

                # Move servos to calculated angles
                active_servo1.angle = servo1_angle
                time.sleep(0.1)
                active_servo2.angle = servo2_angle
                time.sleep(0.1)

    except KeyboardInterrupt:
        pass

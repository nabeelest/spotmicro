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
servo_min = 0  # Minimum servo angle (degrees)
servo_max = 180  # Maximum servo angle (degrees)

# Servo offsets Right Front (for your specific configuration)
foot_offset_RF = 30
leg_offset_RF = -10
shoulder_offset_RF = 10

# Servo offsets Right Back (for your specific configuration)
foot_offset_RB = 30
leg_offset_RB = -10
shoulder_offset_RB = 10


# Inverse Kinematics for Right Legs function
def RightLegIK(x, y, z):
    try:

        # Constant lengths in mm
        upper_leg = 120
        lower_leg = 120
        shoulder_leg = 60

        y1 = sqrt(y * y + z * z - shoulder_leg * shoulder_leg)

        distance = sqrt(x * x + y1 * y1)

        foot = acos(
            (distance * distance - upper_leg * upper_leg - lower_leg * lower_leg)
            / (-2 * upper_leg * lower_leg)
        )

        leg = asin((lower_leg * sin(foot)) / distance) - (atan(x / y) if y != 0 else 0)

        shoulder_leg = atan(distance / shoulder_leg) + atan(z / y)

        # Convert radians to degrees and add servo offsets
        foot_RF = 180 - (foot / pi * 180) + foot_offset_RF
        leg_RF = 180 - (leg / pi * 180) + leg_offset_RF
        shoulder_RF = (shoulder_leg / pi * 180) + shoulder_offset_RF

        # Left Back
        foot_RB = 180 - (foot / pi * 180) + foot_offset_RB
        leg_RB = 180 - (leg / pi * 180) + leg_offset_RB
        shoulder_RB = (shoulder_leg / pi * 180) + shoulder_offset_RB

        # Check if angles are within servo limits
        if (
            (servo_min <= foot_RF <= servo_max)
            and (servo_min <= leg_RF <= servo_max)
            and (servo_min <= shoulder_RF <= servo_max)
            and (servo_min <= foot_RB <= servo_max)
            and (servo_min <= leg_RB <= servo_max)
            and (servo_min <= shoulder_RB <= servo_max)
        ):
            return foot_RF, leg_RF, shoulder_RF, foot_RB, leg_RB, shoulder_RB
        else:
            print("Error: Servo angles are out of bounds")
            print("Moving to ({}, {}, {})".format(x, y, z))
            print("FootLF angle:", foot_RF)
            print("LegLF angle:", leg_RF)
            print("ShoulderLF angle:", shoulder_RF)

            print("FootLB angle:", foot_RB)
            print("LegLB angle:", leg_RB)
            print("ShoulderLB angle:", shoulder_RB)
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
        shoulder_servo_RF = servo.Servo(pca.channels[13])
        leg_servo_RF = servo.Servo(pca.channels[4])
        foot_servo_RF = servo.Servo(pca.channels[5])

        # Set up PCA9685 servo controller Right Back
        shoulder_servo_RB = servo.Servo(pca.channels[9])
        leg_servo_RB = servo.Servo(pca.channels[6])
        foot_servo_RB = servo.Servo(pca.channels[7])

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

        while True:
            stdscr.clear()
            stdscr.addstr(0, 0, "Use arrow keys to move. Press q to quit.")
            print("here")
            key = stdscr.getch()
            if key == curses.KEY_RIGHT:
                if y > 80:
                    x -= 5
                    y -= 5
                    z -= 5
            elif key == curses.KEY_LEFT:
                if y < 200:
                    x += 5
                    y += 5
                    z += 5
            elif key == ord("q"):
                break

            foot_RF, leg_RF, shoulder_RF, foot_RB, leg_RB, shoulder_RB = RightLegIK(
                x, y, z
            )

            if (
                foot_RF is not None
                and leg_RF is not None
                and shoulder_RF is not None
                and foot_RB is not None
                and leg_RB is not None
                and shoulder_RB is not None
            ):
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

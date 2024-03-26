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

# Servo offsets Left Front (for your specific configuration)
foot_offset_LF = -30
leg_offset_LF = 10
shoulder_offset_LF = 5

# Servo offsets Left Back (for your specific configuration)
foot_offset_LB = -30
leg_offset_LB = 10
shoulder_offset_LB = -5


# Inverse Kinematics for Left Legs function
def LeftLegIK(x, y, z):
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
        foot_LF = (foot / pi * 180) + foot_offset_LF
        leg_LF = (leg / pi * 180) + leg_offset_LF
        shoulder_LF = 180 - (shoulder_leg / pi * 180) + shoulder_offset_LF

        # Left Back
        foot_LB = (foot / pi * 180) + foot_offset_LB
        leg_LB = (leg / pi * 180) + leg_offset_LB
        shoulder_LB = 180 - (shoulder_leg / pi * 180) + shoulder_offset_LB

        # Check if angles are within servo limits
        if (
            (servo_min <= foot_LF <= servo_max)
            and (servo_min <= leg_LF <= servo_max)
            and (servo_min <= shoulder_LF <= servo_max)
            and (servo_min <= foot_LB <= servo_max)
            and (servo_min <= leg_LB <= servo_max)
            and (servo_min <= shoulder_LB <= servo_max)
        ):
            return foot_LF, leg_LF, shoulder_LF, foot_LB, leg_LB, shoulder_LB
        else:
            print("Error: Servo angles are out of bounds")
            print("Moving to ({}, {}, {})".format(x, y, z))
            print("FootLF angle:", foot_LF)
            print("LegLF angle:", leg_LF)
            print("ShoulderLF angle:", shoulder_LF)

            print("FootLB angle:", foot_LB)
            print("LegLB angle:", leg_LB)
            print("ShoulderLB angle:", shoulder_LB)
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
        leg_servo_LF = servo.Servo(pca.channels[0])
        foot_servo_LF = servo.Servo(pca.channels[1])
        # Set up PCA9685 servo controller Left Back
        shoulder_servo_LB = servo.Servo(pca.channels[8])
        leg_servo_LB = servo.Servo(pca.channels[2])
        foot_servo_LB = servo.Servo(pca.channels[3])

        # Set pulse width range for Left servos
        shoulder_servo_LF.set_pulse_width_range(min_pulse=500, max_pulse=2500)
        leg_servo_LF.set_pulse_width_range(min_pulse=500, max_pulse=2500)
        foot_servo_LF.set_pulse_width_range(min_pulse=500, max_pulse=2500)
        shoulder_servo_LB.set_pulse_width_range(min_pulse=500, max_pulse=2500)
        leg_servo_LB.set_pulse_width_range(min_pulse=500, max_pulse=2500)
        foot_servo_LB.set_pulse_width_range(min_pulse=500, max_pulse=2500)

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
                # time.sleep(1)
                # y -= 1
                if y < 200:
                    x += 5
                    y += 5
                    z += 5
            elif key == ord("q"):
                break

            foot_LF, leg_LF, shoulder_LF, foot_LB, leg_LB, shoulder_LB = LeftLegIK(
                x, y, z
            )

            if (
                foot_LF is not None
                and leg_LF is not None
                and shoulder_LF is not None
                and foot_LB is not None
                and leg_LB is not None
                and shoulder_LB is not None
            ):
                print("FootLF angle:", foot_LF)
                print("LegLF angle:", leg_LF)
                print("ShoulderLF angle:", shoulder_LF)

                print("FootLB angle:", foot_LB)
                print("LegLB angle:", leg_LB)
                print("ShoulderLB angle:", shoulder_LB)

                # Move servos to calculated angles
                shoulder_servo_LF.angle = shoulder_LF
                leg_servo_LF.angle = leg_LF
                foot_servo_LF.angle = foot_LF
                # Move servos to calculated angles
                shoulder_servo_LB.angle = shoulder_LB
                leg_servo_LB.angle = leg_LB
                foot_servo_LB.angle = foot_LB

    except KeyboardInterrupt:
        pass

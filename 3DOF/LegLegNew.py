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

sHp=np.sin(pi/2)
cHp=np.cos(pi/2)

# Servo angle limits
servo_min = 0 # Minimum servo angle (degrees)
servo_max = 180  # Maximum servo angle (degrees)

# Servo angle limits
servo_min = 0  # Minimum servo angle (degrees)
servo_max = 180  # Maximum servo angle (degrees)

# Servo offsets Left Front (for your specific configuration)
foot_offset_LF = -30
leg_offset_LF = 0
shoulder_offset_LF = -80

# Servo offsets Left Back (for your specific configuration)
foot_offset_LB = -30
leg_offset_LB = 0
shoulder_offset_LB = -90

# Servo offsets Right Front (for your specific configuration)
foot_offset_RF = 30
leg_offset_RF = 0
shoulder_offset_RF = 90

# Servo offsets Right Back (for your specific configuration)
foot_offset_RB = 30
leg_offset_RB = 0
shoulder_offset_RB = 90


def bodyIK(omega,phi,psi,xm,ym,zm):

    L = 220
    W = 110

    Rx = np.array([[1,0,0,0],
                   [0,np.cos(omega),-np.sin(omega),0],
                   [0,np.sin(omega),np.cos(omega),0],[0,0,0,1]])
    Ry = np.array([[np.cos(phi),0,np.sin(phi),0],
                   [0,1,0,0],
                   [-np.sin(phi),0,np.cos(phi),0],[0,0,0,1]])
    Rz = np.array([[np.cos(psi),-np.sin(psi),0,0],
                   [np.sin(psi),np.cos(psi),0,0],[0,0,1,0],[0,0,0,1]])

    Rxyz=Rx@Ry@Rz

    T = np.array([[0,0,0,xm],[0,0,0,ym],[0,0,0,zm],[0,0,0,0]])

    Tm = T+Rxyz

    Trb = Tm @ np.array([[cHp,0,sHp,-L/2],[0,1,0,0],[-sHp,0,cHp,-W/2],[0,0,0,1]])

    Trf = Tm @ np.array([[cHp,0,sHp,L/2],[0,1,0,0],[-sHp,0,cHp,-W/2],[0,0,0,1]])

    Tlf = Tm @ np.array([[cHp,0,sHp,L/2],[0,1,0,0],[-sHp,0,cHp,W/2],[0,0,0,1]])

    Tlb = Tm @ np.array([[cHp,0,sHp,-L/2],[0,1,0,0],[-sHp,0,cHp,W/2],[0,0,0,1]])

    return (Tlf,Trf,Tlb,Trb,Tm)

# Inverse Kinematics for Left Legs function
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

       # Set up PCA9685 servo controller Left Front
        shoulder_servo_LF = servo.Servo(pca.channels[12])
        leg_servo_LF = servo.Servo(pca.channels[13])
        foot_servo_LF = servo.Servo(pca.channels[14])
        # Set up PCA9685 servo controller Left Back
        shoulder_servo_LB = servo.Servo(pca.channels[0])
        leg_servo_LB = servo.Servo(pca.channels[1])
        foot_servo_LB = servo.Servo(pca.channels[2])

        # Set up PCA9685 servo controller Right Front
        shoulder_servo_RF = servo.Servo(pca.channels[8])
        leg_servo_RF = servo.Servo(pca.channels[9])
        foot_servo_RF = servo.Servo(pca.channels[10])

        # Set up PCA9685 servo controller Right Back
        shoulder_servo_RB = servo.Servo(pca.channels[4])
        leg_servo_RB = servo.Servo(pca.channels[5])
        foot_servo_RB = servo.Servo(pca.channels[6])


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

        
        omega = 0 #yaw
        phi = 0 #roll
        psi = 0 #pitch

        xm = 0
        ym = 0
        zm = 0

        FP=[0,0,0,1]


        # Invert local X
        Ix=np.array([[-1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

        Lp=np.array([[110,-150,55,1],[-110, -150,55,1],[110,-150, -55,1],[-110, -150,-55,1]])
        (Tlf,Trf,Tlb,Trb,Tm) = bodyIK(omega,phi,psi,xm,ym,zm)
        CP=[x@FP for x in [Tlf,Trf,Tlb,Trb]]

        pr = [0,-150,0,1]
        pl = [0,-150,0,1]

        prf = Trf@pr
        prb = Trb@pr
        plf = Tlf@pl
        plb = Tlb@pl

        plf = np.linalg.inv(Tlf) @ Lp[0]
        plb = np.linalg.inv(Tlb) @ Lp[1]
        prf = Ix @ np.linalg.inv(Trf) @ Lp[2]
        prb = Ix @ np.linalg.inv(Trb) @ Lp[3]

        foot_LF, leg_LF, shoulder_LF,a,b,c = LeftLegIK(plf[0],plf[1],plf[2])
        a,b,c,foot_LB, leg_LB, shoulder_LB = LeftLegIK(plb[0],plb[1],plb[2])
        foot_RF, leg_RF, shoulder_RF,a,b,c = RightLegIK(prf[0],prf[1],prf[2])
        a,b,c,foot_RB, leg_RB, shoulder_RB = RightLegIK(prb[0],prb[1],prb[2])       

        
        
        if foot_LF is not None and leg_LF is not None and shoulder_LF is not None and foot_LB is not None and leg_LB is not None and shoulder_LB is not None and foot_RF is not None and leg_RF is not None and shoulder_RF is not None and foot_RB is not None and leg_RB is not None and shoulder_RB is not None:
            # print("FootLF angle:{}, LegLF angle:{},ShoulderLF angle:{}".format(foot_LF,leg_LF,shoulder_LF))
            # print("FootLB angle:{}, LegLB angle:{},ShoulderLB angle:{}".format(foot_LB,leg_LB,shoulder_LB))
            # print("FootRF angle:{}, LegRF angle:{},ShoulderRF angle:{}".format(foot_RF,leg_RF,shoulder_RF))
            # print("FootRB angle:{}, LegRB angle:{},ShoulderRB angle:{}".format(foot_RB,leg_RB,shoulder_RB)
            # Move Left Front servos to calculated angles
            shoulder_servo_LF.angle = shoulder_LF
            leg_servo_LF.angle = leg_LF 
            foot_servo_LF.angle = foot_LF
            # # Move Left Back servos to calculated angles
            shoulder_servo_LB.angle = shoulder_LB 
            leg_servo_LB.angle = leg_LB
            foot_servo_LB.angle = foot_LB
                    
            # Move Right Front servos to calculated angles
            shoulder_servo_RF.angle = shoulder_RF
            leg_servo_RF.angle = leg_RF 
            foot_servo_RF.angle = foot_RF 
                
            # Move Right Back servos to calculated angles
            shoulder_servo_RB.angle = shoulder_RB
            leg_servo_RB.angle = leg_RB
            foot_servo_RB.angle = foot_RB
    except KeyboardInterrupt:
        pass
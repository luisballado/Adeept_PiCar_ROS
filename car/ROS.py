#!/usr/bin/env python

# This script does both auto and manned controls
import os
import rospy
from sensor_msgs.msg import Joy
import RPi.GPIO as GPIO
import time
import numpy as np
import Adafruit_PCA9685
from geometry_msgs.msg import Twist

#os.system('/home/pi/Desktop/GST_Vid.sh &')

GPIO.setmode(GPIO.BCM)

# Motor_A is not used but left for future

#Motor_A_EN    = 4
Motor_B_EN    = 17

#Motor_A_Pin1  = 14
#Motor_A_Pin2  = 15
Motor_B_Pin1  = 27
Motor_B_Pin2  = 18

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)

pwm.set_pwm(2,0,320)

#def setup(): #Motor initialization
global pwm_A, pwm_B

# Set initial PWM states
pwm_A = 0
pwm_B = 0

#Set up GPIO PWM outputs for the H-bridge
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
#GPIO.setup(Motor_A_EN, GPIO.OUT)
GPIO.setup(Motor_B_EN, GPIO.OUT)
#GPIO.setup(Motor_A_Pin1, GPIO.OUT)
#GPIO.setup(Motor_A_Pin2, GPIO.OUT)
GPIO.setup(Motor_B_Pin1, GPIO.OUT)
GPIO.setup(Motor_B_Pin2, GPIO.OUT)
#pwm_A = GPIO.PWM(Motor_A_EN, 500)
pwm_B = GPIO.PWM(Motor_B_EN, 500)

# Collect joystick messages form ROS Master
def Joy_callback(data):

    A = data.buttons[0]
    B = data.buttons[1]
    X = data.buttons[2]
    Y = data.buttons[3]
    Lbmp = data.buttons[4]
    Rbmp = data.buttons[5]
    Sel = data.buttons[6]
    Xbt = data.buttons[7]
    Rjs = data.buttons[8]
    Ljs = data.buttons[9]

    LjoyH = data.axes[0]
    LjoyV = data.axes[1]
    Ltrig = data.axes[2]
    RjoyH = data.axes[3]
    RjoyV = data.axes[4]
    Rtrig = data.axes[5]
    DpadH = data.axes[6]
    DpadV = data.axes[7]

# Some motor mapping

    Rtrig = (Rtrig + 1) * 50
    Ltrig = (Ltrig + 1) * 50

    fb = (Ltrig - Rtrig)

    spd_joy = abs(fb)

    # Motor Controlling

    if fb > 50:
        GPIO.output(Motor_B_Pin1, GPIO.HIGH)
        GPIO.output(Motor_B_Pin2, GPIO.LOW)
        pwm_B.start(spd_joy)
        pwm_B.ChangeDutyCycle(spd_joy)
    elif fb < -50:
        GPIO.output(Motor_B_Pin1, GPIO.LOW)
        GPIO.output(Motor_B_Pin2, GPIO.HIGH)
        pwm_B.start(spd_joy)
        pwm_B.ChangeDutyCycle(spd_joy)
    else:
        pwm_B.ChangeDutyCycle(0)
        pass

# Turn handling
    ang = int(np.interp(LjoyH, (-1, 1), (240, 400)))

    pwm.set_pwm(2,0,ang)

# Collect movment data from autonomous script on ROS Master
def Twist_callback(data):

    ang = 0

    forward = data.linear.x
    turn = data.linear.y

# Motor Controlling

    if forward == 1:
        GPIO.output(Motor_B_Pin1, GPIO.HIGH)
        GPIO.output(Motor_B_Pin2, GPIO.LOW)
        pwm_B.start(50)
        pwm_B.ChangeDutyCycle(50)
    elif forward == 0:
        GPIO.output(Motor_B_Pin1, GPIO.LOW)
        GPIO.output(Motor_B_Pin2, GPIO.LOW)
        pwm_B.start(0)
        pwm_B.ChangeDutyCycle(0)
    else:
        pass

    ang = int(np.interp(turn, (0, 600), (380, 260)))

    # Turn handling
    pwm.set_pwm(2,0,ang)

# Intializes everything

def start():
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, Joy_callback)
    rospy.Subscriber("Twist", Twist, Twist_callback)
    # starts the node
    rospy.init_node('ROS_Car')
    rospy.spin()

def motorStop(): # Motor stops
#    GPIO.output(Motor_A_Pin1, GPIO.LOW)
#    GPIO.output(Motor_A_Pin2, GPIO.LOW)
#    GPIO.output(Motor_B_Pin1, GPIO.LOW)
#    GPIO.output(Motor_B_Pin2, GPIO.LOW)
#    GPIO.output(Motor_A_EN, GPIO.LOW)
#    GPIO.output(Motor_B_EN, GPIO.LOW)
     pass

def destroy():
    motorStop()
    GPIO.cleanup()             # Release resource


if __name__ == '__main__':
#    setup()
    start()

try:
    pass
except KeyboardInterrupt:
    destroy()

global MAX_SPEED
MAX_SPEED = 50 #placeholder

global turn90
turn90 = 500 #placeholder

import time

def fullSpeed():
    motor_left = MAX_SPEED
    motor_right = MAX_SPEED

def bodySpin():
    motor_left = MAX_SPEED
    motor_right = -MAX_SPEED
    time.sleep(turn90)

def collectObjective(): #placeholder
    servo = MAX_SPEED
    time.sleep(10)
    motor_left = MAX_SPEED
    motor_right = MAX_SPEED
    time.sleep(20)
    servo = -MAX_SPEED
    bodySpin()

def turnLeft(strength):
    motor_left = motor_left - strength

def turnRight(strength):
    motor_right = motor_right - strength



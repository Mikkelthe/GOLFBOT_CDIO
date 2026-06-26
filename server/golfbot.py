#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import math

class GolfBot:
    def __init__(self, brick: EV3Brick, leftMotor: Motor, rightMotor: Motor, rampMotor: Motor):
        self.brick = brick
        self.leftMotor = leftMotor
        self.rightMotor = rightMotor
        self.rampMotor = rampMotor

        # to be calibrated
        self.moveSpeed = 500
        self.anglePerMM = 10.0
        self.turnSpeed = 500
        self.anglePerDeg = 10.0

        self.rampSpeed = 100
    
    def move_dir(self, x, y):
        a = [x, y]
        b = [0, 1]
        magnitude = math.sqrt(x*x + y*y)
        x_norm = x/magnitude
        y_norm = y/magnitude
        signedAngle = math.atan2(
            a[0]*b[1] - a[1]*b[0],
            a[0]*b[0] + a[1]*b[1])
    
        dot_left = -x_norm
        dot_right = x_norm
        dot_up = y_norm

        if y > 0:
            self.leftMotor.run((dot_up + dot_left) * self.moveSpeed)
            self.rightMotor.run((dot_up + dot_right) * self.moveSpeed)        
        else:
            self.leftMotor.run((dot_up + dot_right) * self.moveSpeed)
            self.rightMotor.run((dot_up + dot_left) * self.moveSpeed)        

    def move_fwd(self):
        self.leftMotor.run(self.moveSpeed)
        self.rightMotor.run(self.moveSpeed)

    def move_fwd_left(self):
        self.leftMotor.run(self.moveSpeed/3)
        self.rightMotor.run(self.moveSpeed)

    def move_fwd_right(self):
        self.leftMotor.run(self.moveSpeed)
        self.rightMotor.run(self.moveSpeed/3)
    
    def move_back(self):
        self.leftMotor.run(-self.moveSpeed)
        self.rightMotor.run(-self.moveSpeed)

    def turn_left(self):
        self.leftMotor.run(-self.turnSpeed)
        self.rightMotor.run(self.turnSpeed)

    def turn_right(self):
        self.leftMotor.run(self.turnSpeed)
        self.rightMotor.run(-self.turnSpeed)

    def open_ramp(self):
        self.rampMotor.run(self.rampSpeed)

    def close_ramp(self):
        self.rampMotor.run(-self.rampSpeed)
    
    def stop(self):
        self.leftMotor.stop()
        self.rightMotor.stop()
        self.rampMotor.stop()

    def move(self, length_mm: int):
        self.leftMotor.run_angle(self.moveSpeed, -self.anglePerMM*length_mm, wait=False)
        self.rightMotor.run_angle(self.moveSpeed, -self.anglePerMM*length_mm)
    
    def turn(self, angleDeg: int):
        self.leftMotor.run_angle(self.turnSpeed, self.anglePerDeg*angleDeg, wait=False)
        self.rightMotor.run_angle(self.turnSpeed, -self.anglePerDeg*angleDeg)
    
    def openRamp(self):
        self.rampMotor.run_until_stalled(self.rampSpeed, then=Stop.HOLD, duty_limit=0.2)
    
    def closeRamp(self):
        self.rampMotor.run_until_stalled(-self.rampSpeed, then=Stop.HOLD, duty_limit=0.2)
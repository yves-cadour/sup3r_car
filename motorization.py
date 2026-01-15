#!/usr/bin/env micropython
# coding: utf-8

from ev3dev2.motor import LargeMotor, SpeedDPS # DPS stands for Degree Per Second
from math import sqrt, tan, pi
from time import sleep

class Motorization:
    """
    Une classe représentant la motorisation de la sup3r car
    """
    # Constructor
    def __init__(self, car):
        # motors
        self._lm = LargeMotor('outB');   # left motor
        self._rm = LargeMotor('outC');   # right motor
        self._wheelbase = 19             # empattement (distance entre essieux)
        self._track_width = 15           # voie (distance entre pneux)
        self._car = car

    #############################################################################
    #   GETTERS
    #############################################################################

    def get_car(self):
        """Return the car instance the motorization is attached to"""
        return self._car

    def get_left_motor(self):
        """Return the left motor instance"""
        return self._lm

    def get_right_motor(self):
        """Return the right motor instance"""
        return self._rm

    def get_wheelbase(self):
        """Return distance between the front and the back wheels (no unit needed)"""
        return self._wheelbase

    def get_track_width(self):
        """Return distance between the wheels (no unit needed)"""
        return self._track_width

    def run(self, speed, differential = True):
        """Run the two rear motors

        Arguments:
            speed (int): un nombre compris entre -100 et 100
            differential (boolean) : If set to False the motors will always rotate at the same speed
        """
        speed = -speed # otherwise get backwards
        if speed > 100 : speed = 100
        elif speed < -100 : speed = -100
        speed_left = speed_right = speed
        divider = 1
        if differential:
            steering = self.get_car().get_steering()
            motor_angle = steering.get_angle()
            wheel_angle = motor_angle / steering._steering_divider
            # Optimisation : précalculer les constantes et utiliser approximation pour tan si angle petit
            track_width = self._track_width
            wheelbase = self._wheelbase
            x_rad = wheel_angle * 0.017453292519943295  # pi / 180
            if abs(x_rad) < 0.1:  # Approximation pour petits angles
                tan_approx = x_rad
            else:
                tan_approx = tan(x_rad)
            compute = track_width * tan_approx / (2 * wheelbase)
            speed_left = (1 - compute) * speed
            speed_right = (1 + compute) * speed
            # The speed cant be more than possible
            if speed_right > 100 or speed_right < -100:
                divider = abs(speed_right / 100)
            elif speed_left > 100 or speed_left < -100:
                divider = abs(speed_left / 100)
        #print("left : "+str((speed_left/100)*(1040/divider)))
        #print("right : "+str((speed_right/100)*(1040/divider)))
        self.get_left_motor().on(speed=SpeedDPS((speed_left/100)*(1040/divider)), block=False) # 1050 is the max RPM for large motor
        self.get_right_motor().on(speed=SpeedDPS((speed_right/100)*(1040/divider)), block=False)
        
    def stop(self):
        """Stop the motorization
        """
        self.get_left_motor().off()
        self.get_right_motor().off()
        self.get_car().get_steering().turn(0)

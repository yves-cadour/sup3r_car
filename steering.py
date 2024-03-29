#!/usr/bin/env micropython
# coding: utf-8

from ev3dev2.motor import MediumMotor

class Steering:
    """
    A class for the sup3r_car steering
    """
    # Constructor
    def __init__(self, car):
        # motor
        self._motor = MediumMotor('outA'); # medium motor
        self._car = car
        self._angle = 0
        self._max_angle = 0
        self._steering_divider = 1

    def get_angle(self):
        """Return the steering angle."""
        return self._angle

    def get_max_angle(self):
        """Return the maximum steering angle."""
        return self._max_angle

    def set_max_angle(self, angle):
        """Set the max angle the steering motor can handle.

        Args:
            angle (int): the maximum angle of the steering motor
        """
        self._max_angle = angle

    def get_steering_divider(self):
        """The divider is the division between the angle of the steering motor and the angle of the wheel
        """
        return self._steering_divider

    def get_car(self):
        """Return the car instance the steering is attached to."""
        return self._car
        
    def turn(self, angle, speed=100):
        """
        Tourne le moteur de direction à la position 'angle'. Angle positif à droite, négatif à gauche
        La valeur par défaut de la vitesse de rotation est 50.
        """
        if angle > 0:
            angle = min(angle, self._max_angle)
        elif angle < 0:
            angle = max(angle, - self._max_angle)
        self._angle =  - angle
        self._motor.on_to_position(speed, - angle)

#! /usr/bin/env python3

import RPi.GPIO as GPIO


class DCMotor:
    """DC Motor driver class driven by PWM"""

    def __init__(self, forPin, bacPin):
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(forPin, GPIO.OUT)
        GPIO.setup(bacPin, GPIO.OUT)

        frequency = 20
        maxSpeed = 100
        minSpeed = 0

        self._frequency = frequency
        self._maxSpeed = maxSpeed
        self._minSpeed = minSpeed
        self._forPWM = GPIO.PWM(forPin, frequency)
        self._bacPWM = GPIO.PWM(bacPin, frequency)
        self.stop()

    def forward(self, speed=None):
        """Run motor forward"""
        if speed is None:
            speed = self._minSpeed
        self.run(speed)

    def reverse(self, speed=None):
        """Run motor Backward"""
        if speed is None:
            speed = self._minSpeed
        self.run(speed)

    def stop(self):
        # Stop the motor
        self.run(0)

    def run(self, speed=None):
        """Start the motor. If no speed given
        start at max speed."""
        if speed is None:
            speed = self._minSpeed

        # Ensure speed is within limits + or - max speed
        speed = min(self._maxSpeed, speed)
        speed = max(-self._maxSpeed, speed)

        # Run the motors
        if speed < 0:
            self._forPWM.start(0)
            self._bacPWM.start(-1.0 * speed)
        else:
            self._forPWM.start(speed)
            self._bacPWM.start(0)


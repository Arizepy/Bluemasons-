# #!/usr/bin/env python3

"""
    The drivetrain moves the chassis of the bot.
    Handles low-level motor primitives safely across multiple threads.
"""
import threading
import time
from ev3dev2.motor import SpeedPercent, MediumMotor

class DriveBase:

    def __init__(self, left_motor, right_motor):
        self.left_motor = left_motor
        self.right_motor = right_motor

        self._left_speed = 60
        self._right_speed = 60

        self.left_motor.off()
        self.right_motor.off()

        self.left_motor.stop_action = "brake"
        self.right_motor.stop_action = "brake"

        self._turn_speed = 50
        self._drive_lock = threading.Lock()

    def drive(self, left_speed, right_speed):
        """Applies clamped speed parameters across safe motor locking grids."""
        with self._drive_lock:
            # Bound input limits safely to hardware thresholds
            left_speed = max(min(int(left_speed), 100), -100)
            right_speed = max(min(int(right_speed), 100), -100)

            self.left_motor.on(left_speed)
            self.right_motor.on(right_speed)

    def forward(self, speed):
        self.drive(speed, speed)

    def backward(self, speed):
        self.drive(-speed, -speed)

    def turn_left(self, speed=50):
        # Force absolute velocity magnitudes to preserve rotation direction rules
        actual_speed = abs(int(speed))
        self.drive(-actual_speed, actual_speed)

    def turn_right(self, speed=50):
        actual_speed = abs(int(speed))
        self.drive(actual_speed, -actual_speed)

    def stop(self):
        with self._drive_lock:
            self.left_motor.off()
            self.right_motor.off()

    def wobble(self, degrees=60, speed=70, count=2):
        """
        Executes a sharp, simultaneous twist back and forth.
        
        :param degrees: How far each wheel rotates per wiggle step
        :param speed: Power level (faster speed = sharper shake)
        :param count: Number of wobble cycles to execute
        """
        with self._drive_lock:
            for _ in range(count):
                # 1. Twist Left (Left backward, Right forward)
                self.left_motor.on_for_degrees(SpeedPercent(-speed), degrees, block=False)
                self.right_motor.on_for_degrees(SpeedPercent(speed), degrees, block=True)
                
                # 2. Twist Right (Left forward, Right backward)
                self.left_motor.on_for_degrees(SpeedPercent(speed), degrees, block=False)
                self.right_motor.on_for_degrees(SpeedPercent(-speed), degrees, block=True)

            # 3. Secure drive base at the end
            self.left_motor.stop(stop_action="hold")
            self.right_motor.stop(stop_action="hold")
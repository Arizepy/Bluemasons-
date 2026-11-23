#!/usr/bin/env python3

"""
    Navigation is a code design to control movement
"""

import math
from drivetrain import DriveBase
from linefollow1 import LineFollower
from pose import Pose
import time


class Navigator:

    def __init__(self,
                pose:Pose,
                drivetrain:DriveBase,
                line: LineFollower
                ):
        self.pose  = pose
        self.drivetrain = drivetrain
        self.line = line
    
    # def drive_distance(self,
    #                 distance:float,
    #                 speed: int = 70) -> None:
    #     forward = distance >= 0      

    #     start_x, start_y = self.pose.get_position()
    #     traveled = 0
    #     if abs(distance) < 0.1:
    #         return
    #     motor_speed = speed if forward else -speed
    #     if forward:
    #         self.drivetrain.forward(motor_speed)
    #     else:
    #         self.drivetrain.backward(motor_speed)
        
    #     while traveled < abs(distance):

    #         x,y = self.pose.get_position()
    #         dx = x-start_x
    #         dy = y- start_y
    #         traveled = math.hypot(dx,dy)
    #         if traveled >= abs(distance):
    #             break
    #         time.sleep(0.01)
    #     self.drivetrain.stop()
    # def drive_distance(self, distance: float, speed: int = 70) -> None:
    #     """Drives straight using your running background Odometry thread coordinates to stop."""
    #     if abs(distance) < 1:
    #         return

    #     forward = distance >= 0      
    #     start_x, start_y = self.pose.get_position()
        
    #     # Determine driving power polarity
    #     motor_speed = abs(int(speed)) if forward else -abs(int(speed))
        
    #     # Start the chassis moving using your safe drivetrain wrapper
    #     if forward:
    #         self.drivetrain.forward(motor_speed)
    #     else:
    #         self.drivetrain.backward(abs(motor_speed))
        
    #     # Run loop strictly checking the live position calculated by Odometry
    #     while True:
    #         x, y = self.pose.get_position()
    #         dx = x - start_x
    #         dy = y - start_y
            
    #         # Use hypotenuse to find the straight-line displacement distance
    #         traveled = math.hypot(dx, dy)
            
    #         if traveled >= abs(distance):
    #             break
                
    #         time.sleep(0.005)  # Fast sampling to prevent overshooting the target distance
            
    #     # Hard brake instantly once the background odometry says target is reached
    #     self.drivetrain.stop()
    def drive_distance(self, distance: float, speed: int = 70, ramp_dist: float = 8.0, min_speed: int = 15) -> None:
        """Drives straight with smooth acceleration and deceleration using Odometry."""
        total_dist = abs(distance)
        if total_dist < 0.1:
            return

        forward = distance >= 0      
        start_x, start_y = self.pose.get_position()
        actual_ramp = min(ramp_dist, total_dist / 3.0)
        
        while True:
            x, y = self.pose.get_position()
            traveled = math.hypot(x - start_x, y - start_y)
            if traveled >= total_dist:
                break
                
            dist_remaining = total_dist - traveled
            accel_scale = min(1.0, traveled / actual_ramp) if actual_ramp > 0 else 1.0
            decel_scale = min(1.0, dist_remaining / actual_ramp) if actual_ramp > 0 else 1.0
            speed_scale = min(accel_scale, decel_scale)

            current_abs_speed = min_speed + (abs(speed) - min_speed) * speed_scale
            motor_speed = current_abs_speed if forward else -current_abs_speed

            self.drivetrain.drive(motor_speed, motor_speed)
            time.sleep(0.005)
            
        self.drivetrain.stop()
        
    def _normalize_angle(self,angle):
        while angle > math.pi:
            angle-= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle
    
    def turn_to_heading(self,
                        target_heading:float,
                        speed: int = 50,
                        minpower = 15
                        ):
        tolerance = math.radians(5)
        kp = 3.5
        min_power  = minpower
        heading = self.pose.get_heading()
        error = target_heading - heading
        while True:
            heading = self.pose.get_heading()
            error = target_heading - heading
            error = self._normalize_angle(error)

            if abs(error) <= tolerance:
                break 

            calculated_power = kp * error*10

            sign =1 if calculated_power > 0 else -1
            final_speed = sign * max(abs(calculated_power), min_power) 


            if error > 0:
                self.drivetrain.turn_right(final_speed)
            else:
                self.drivetrain.turn_left(final_speed)
            
            time.sleep(0.01)
    
        self.drivetrain.stop()
        
    def turn_relative(self,angle_degrees:float,speed:int=50):
        current_heading = self.pose.get_heading()
        target_heading = current_heading + math.radians(angle_degrees)
        self.turn_to_heading(target_heading,speed=speed)
        
    def goto_position(self,
                    x,
                    y) -> None:
        current_x,current_y = self.pose.get_position()
        dx = x-current_x
        dy = y-current_y

        target_heading = math.atan2(dy,dx)
        distance = math.hypot(dx,dy)
        self.turn_to_heading(target_heading)
        self.drive_distance(distance=distance)
        
    def follow_path(self):
        pass

    def return_home(self):
        pass

    def follow_line_distance(self,distance):
        start_x, start_y = self.pose.get_position()
        self.line.start_following()
        while True:
            x,y = self.pose.get_position()
            traveled = math.hypot(x-start_x, y-start_y)
            if traveled >= distance:
                break
            time.sleep(0.01)
        self.line.stop_following()

    def follow_line_until(self,condition):

        self.line.start_following()
        
        while not condition():
            time.sleep(0.01)
        
        self.line.stop_following()

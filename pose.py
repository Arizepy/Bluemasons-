#!/usr/bin/env python3

"""
    Pose is a thread-safe container that stores the robot's estimate position and orientation.
    
    The pose consists of :
        x           -X posisiton in centimeters
        y           -Y posistion in centimeters
        heading     -Orientatino in radians

    Pose does not calculate movement. Only stores and provides robot position
"""
import threading


class Pose:

    def __init__(self,x=0.0,y=0.0,heading=0.0):
        self._x = x
        self._y = y
        self._heading = heading

        self._pose_lock = threading.Lock()

    def get_pose(self):
        with self._pose_lock:
            return self._x, self._y, self._heading
        
    def set_pose(self,
                x:float,
                y:float,
                heading:float) -> None:
        with self._pose_lock:
            self._x = x
            self._y = y
            self._heading = heading

    def reset(self):
        self.set_pose(0.0,0.0,0.0)

    def set_heading(self,heading:float)-> None:
        with self._pose_lock:
            self._heading = heading

    def set_position(self,
                    x:float,
                    y:float) -> None:
        with self._pose_lock:
            self._x = x
            self._y = y

    def get_heading(self) -> float:
        with self._pose_lock:
            return self._heading
    
    def get_position(self):
        with self._pose_lock:
            return self._x, self._y


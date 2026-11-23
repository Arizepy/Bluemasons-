# #!/usr/bin/env python3

# #pose  = estimated position
# import threading
# import math
# from worker import StoppableThread
# from pose import Pose
# from debug import log

# WHEEL_DIAMETER = 6.24 #cm 
# TRACK_WIDTH = 21.18 #cm

# class Odometry(StoppableThread):
#     def __init__(self,
#                 left_motor,
#                 right_motor,
#                 pose:Pose,
#                 gyro,
#                 log:log
#                 ):
#         super().__init__()

#         self.left_motor = left_motor
#         self.right_motor = right_motor

#         self._last_left = self.left_motor.position
#         self._last_right = self.right_motor.position

#         self._wheel_diameter = WHEEL_DIAMETER
#         self._track_width = TRACK_WIDTH
#         self._wheel_cirmference = self._wheel_diameter*math.pi
        
#         self.pose = pose
#         self.gyro = gyro
#         self.log = log


#     def loop(self):
#         dleft,dright = self._read_encoders()

#         dl,dr = self._encoders_to_distances(dleft,dright)
        
#         forward_distance = (dl+dr)/2.0


#         x, y, heading = self.pose.get_pose()

#         x += forward_distance *math.cos(heading)
#         y += forward_distance*math.sin(heading)
#         log("[ODO]","X:{:.1f} Y:{:.1f}".format(x)(y))
#         self.pose.set_position(x, y)
    
#     def _read_encoders(self) :
#         left = self.left_motor.position
#         right = self.right_motor.position

#         dleft = left -  self._last_left
#         dright = right - self._last_right

#         self._last_left = left
#         self._last_right = right
#         return dleft,dright
    
#     def _degrees_to_distance(self,degree: float) -> float :
#         distance = degree/360*self._wheel_cirmference
#         return distance
    
#     def  _encoders_to_distances(self,
#                                 dleft:float,
#                                 dright:float):
#         dl = self._degrees_to_distance(dleft)
#         dr = self._degrees_to_distance(dright)
#         return dl,dr
    
#     def _normalize_angle(self):
#         pass

#!/usr/bin/env python3

#pose  = estimated position
import threading
import math
import time
from worker import StoppableThread
from pose import Pose
from debug import log

WHEEL_DIAMETER = 6.24 #cm 
TRACK_WIDTH = 10.5 #cm

def log(*args, **kwargs):pass

class Odometry(StoppableThread):
    def __init__(self,
                left_motor,
                right_motor,
                pose: Pose,
                gyro,
                log: log
                ):
        super().__init__()

        self.left_motor = left_motor
        self.right_motor = right_motor

        self._last_left = self.left_motor.position
        self._last_right = self.right_motor.position

        self._wheel_diameter = WHEEL_DIAMETER
        self._track_width = TRACK_WIDTH
        self._wheel_circumference = self._wheel_diameter * math.pi
        
        self.pose = pose
        self.gyro = gyro
        self.log = log
        self._last_log_time = 0.0 # To prevent logging performance lag

    def loop(self):
        dleft, dright = self._read_encoders()
        dl, dr = self._encoders_to_distances(dleft, dright)
        
        forward_distance = (dl + dr) / 2.0

        # Sync heading directly from your gyro thread via the Pose object
        x, y, heading = self.pose.get_pose()

        x += forward_distance * math.cos(heading)
        y += forward_distance * math.sin(heading)
        
        self.pose.set_position(x, y)

        # Throttle telemetry logging to 5Hz so your CPU isn't overloaded
        current_time = time.time()
        if current_time - self._last_log_time >= 0.2:
            log("[ODO]", "X:{:.1f} Y:{:.1f} H:{:.2f}".format(x, y, heading))
            self._last_log_time = current_time
            
        # Small delay to mimic typical loop rates if StoppableThread doesn't include one
        # time.sleep(0.01)
    
    def _read_encoders(self):
        left = self.left_motor.position
        right = self.right_motor.position

        dleft = left - self._last_left
        dright = right - self._last_right

        self._last_left = left
        self._last_right = right
        return dleft, dright
    
    def _degrees_to_distance(self, degree: float) -> float:
        distance = (degree / 360.0) * self._wheel_circumference
        return distance
    
    def _encoders_to_distances(self, dleft: float, dright: float):
        dl = self._degrees_to_distance(dleft)
        dr = self._degrees_to_distance(dright)
        return dl, dr

# #!/usr/bin/env python3


# import threading
# import time
# import math

# from ev3dev2.sensor.lego import GyroSensor
# from pose import Pose
# from debug import log

# # def log(*args, **kwargs):pass

# class GyroHeading(threading.Thread):

#     def __init__(self, gyro_sensor, pose: Pose):
#         super().__init__(daemon=True)
#         self.sensor = gyro_sensor
#         self._lock = threading.Lock()

#         self._raw_degrees = 0.0
#         self._zero_offset = 0.0
#         self._heading_rad = 0.0

#         self.pose = pose
#         # Set sensor to angle mode
#         self.sensor.mode = GyroSensor.MODE_GYRO_ANG
#         # Optionally reset sensor to zero
#         self.sensor.reset()
#         time.sleep(0.1)  # Allow time for the sensor to reset

#     def run(self):
#         self._running = True
#         while self._running:
#             try:
#                 raw = float(self.sensor.angle)
#             except (ValueError,OSError):
#                 time.sleep(0.005)
#                 continue

#             # calculate heading relative to your last zero() point
#             heading_deg = raw - self._zero_offset

#             #convert to radians
#             heading_rad = math.radians(heading_deg)

#             #bound the angle strictly between -pi and pi
#             heading_rad = ((heading_rad + math.pi) % (2 * math.pi)) - math.pi

#             with self._lock:
#                 self._raw_degrees = raw
#                 self._heading_rad = heading_rad

#                 self.pose.set_heading(self._heading_rad)
#                 log("[GYRO]", self._raw_degrees)
#             time.sleep(0.005)

#     def zero(self):
#         while True:
#             try:
#                 raw = float(self.sensor.angle)
#                 break
#             except ValueError:
#                 time.sleep(0.01)
#                 continue
#         with self._lock:
#             self._raw_degrees = raw
#             self._zero_offset = raw
#             self._heading_rad = 0.0
#             self.pose.set_heading(0.0)

#     def get_raw_degrees(self):
#         with self._lock:
#             return self._raw_degrees

#     def get_heading(self):
#         with self._lock:
#             return self._heading_rad
        
#     def stop(self):
#         self._running = False
#         self.join(timeout=1.0)

#!/usr/bin/env python3

import threading
import time
import math

from ev3dev2.sensor.lego import GyroSensor
from debug import log
from pose import Pose

# def log(*args, **kwargs):pass

class GyroHeading(threading.Thread):

    def __init__(self, gyro_sensor, pose: Pose):
        super().__init__(daemon=True)
        self.sensor = gyro_sensor
        self._lock = threading.Lock()

        self._raw_degrees = 0.0
        self._zero_offset = 0.0
        self._heading_rad = 0.0

        self.pose = pose
        # Set sensor to angle mode
        self.sensor.mode = GyroSensor.MODE_GYRO_ANG
        # Optionally reset sensor to zero
        self.sensor.reset()
        time.sleep(0.1)  # Allow time for the sensor to reset

    def run(self):
        self._running = True
        last_log_time = 0.0  # Track time to throttle logs
        
        while self._running:
            try:
                raw = float(self.sensor.angle)
            except (ValueError, OSError):
                time.sleep(0.005)
                continue

            # calculate heading relative to your last zero() point
            heading_deg = raw - self._zero_offset
            log(heading_deg)
            # convert to radians
            heading_rad = math.radians(heading_deg)

            # bound the angle strictly between -pi and pi
            heading_rad = ((heading_rad + math.pi) % (2 * math.pi)) - math.pi

            with self._lock:
                self._raw_degrees = raw
                self._heading_rad = heading_rad

                self.pose.set_heading(self._heading_rad)
                
                # Throttle log output to every 0.2 seconds to save CPU cycles
                current_time = time.time()
                if current_time - last_log_time >= 0.2:
                    last_log_time = current_time 
                    
            time.sleep(0.005)

    def zero(self):
        while True:
            try:
                raw = float(self.sensor.angle)
                break
            except ValueError:
                time.sleep(0.01)
                continue
        with self._lock:
            self._raw_degrees = raw
            self._zero_offset = raw
            self._heading_rad = 0.0
            self.pose.set_heading(0.0)

    def get_raw_degrees(self):
        with self._lock:
            return self._raw_degrees

    def get_heading(self):
        with self._lock:
            return self._heading_rad
        
    def stop(self):
        self._running = False
        self.join(timeout=1.0)

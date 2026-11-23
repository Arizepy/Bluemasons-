# #!/usr/bin/env python3
# """
# JourneyRecorder - Records and replays robot movements using Gyro and Time/Speed 
# (bypassing large motor encoders).
# """

# import time
# import json
# import os
# import math

# class JourneyRecorder:
#     def __init__(self, robot, filename="journey_log.json"):
#         self.robot = robot
#         self.filename = filename
#         self.recording = False
#         self.journey_log = []
#         self._start_time = 0.0

#     def start_recording(self):
#         """Clears previous log and starts recording actions."""
#         self.journey_log = []
#         self.recording = True
#         self._start_time = time.time()
#         print("[RECORDER] Recording started...", flush=True)

#     def log_action(self, action_type, params):
#         """Logs an atomic movement action with a timestamp."""
#         if not self.recording:
#             return
#         timestamp = time.time() - self._start_time
#         entry = {
#             "time": timestamp,
#             "type": action_type,  # 'drive', 'turn', 'wait', 'arm'
#             "params": params
#         }
#         self.journey_log.append(entry)

#     def stop_recording(self):
#         """Stops recording and saves the journey log to a JSON file."""
#         self.recording = False
#         try:
#             with open(self.filename, 'w') as f:
#                 json.dump(self.journey_log, f, indent=4)
#             print("[RECORDER] Journey successfully saved to {} ({:d} actions recorded).".format(
#                 self.filename, len(self.journey_log)), flush=True)
#         except Exception as e:
#             print("[RECORDER ERROR] Failed to save file: {}".format(e), flush=True)

#     def play_journey(self):
#         """Loads and replays the recorded journey sequence on the map."""
#         if not os.path.exists(self.filename):
#             print("[RECORDER ERROR] No recorded journey file found at {}".format(self.filename), flush=True)
#             return

#         try:
#             with open(self.filename, 'r') as f:
#                 journey = json.load(f)
#         except Exception as e:
#             print("[RECORDER ERROR] Failed to load journey file: {}".format(e), flush=True)
#             return

#         print("[RECORDER] Replaying journey ({:d} actions)...".format(len(journey)), flush=True)
        
#         # Zero the gyro before playback to align orientation
#         self.robot.gyro.zero()
#         time.sleep(0.1)

#         for action in journey:
#             action_type = action["type"]
#             params = action["params"]

#             if action_type == "drive":
#                 # params: [distance, speed, target_heading]
#                 dist, speed, heading = params
#                 print("[REPLAY] Driving distance: {:.1f}cm at speed {:d}".format(dist, speed), flush=True)
#                 self.robot.drive_distance_gyro(dist, heading, base_speed=speed)

#             elif action_type == "turn":
#                 # params: [target_heading_deg, speed]
#                 deg, speed = params
#                 print("[REPLAY] Turning to heading: {:.1f}".format(deg), flush=True)
#                 self.robot.navigation.turn_to_heading(math.radians(deg), speed=speed)

#             elif action_type == "wait":
#                 # params: [duration]
#                 duration = params[0]
#                 time.sleep(duration)

#             elif action_type == "arm":
#                 # params: [action_name, target]
#                 cmd, target = params
#                 if cmd == "lower":
#                     self.robot.lower_arm(target)
#                 elif cmd == "lift":
#                     self.robot.lift_arm(target)
#                 elif cmd == "grab_down":
#                     self.robot.grab_arm_down(target)
#                 elif cmd == "grab_up":
#                     self.robot.grab_arm_up(target)

#             time.sleep(0.05)  # Short breather between instructions

#         print("[RECORDER] Journey playback complete!", flush=True)



#!/usr/init/env python3
"""
JourneyRecorder - Records and replays all mission functions (driving, line following, 
gyro navigation, arms, wobbles, and vision scans) bypassing large motor encoders.
"""

import time
import json
import os
import math

class JourneyRecorder:
    def __init__(self, robot, filename="map.json"):
        self.robot = robot
        self.filename = filename
        self.recording = False
        self.journey_log = []
        self._start_time = 0.0

    def start_recording(self):
        """Clears previous log and starts recording all mission actions."""
        self.journey_log = []
        self.recording = True
        self._start_time = time.time()
        print("[RECORDER] Mission recording started...", flush=True)

    def log_action(self, action_type, params):
        """Logs any mission function call with relative execution timestamp."""
        if not self.recording:
            return
        timestamp = time.time() - self._start_time
        entry = {
            "time": timestamp,
            "type": action_type, 
            "params": params
        }
        self.journey_log.append(entry)

    def stop_recording(self):
        """Stops recording and saves the complete mission log to JSON."""
        self.recording = False
        try:
            with open(self.filename, 'w') as f:
                json.dump(self.journey_log, f, indent=4)
            print("[RECORDER] Mission journey saved to {} ({:d} actions recorded).".format(
                self.filename, len(self.journey_log)), flush=True)
        except Exception as e:
            print("[RECORDER ERROR] Failed to save file: {}".format(e), flush=True)

    def _load_journey_file(self):
        """Safely loads the journey JSON file with validation checks."""
        if not os.path.exists(self.filename) or os.path.getsize(self.filename) == 0:
            print("[RECORDER ERROR] Journey file '{}' is missing or empty.".format(self.filename), flush=True)
            return None
        try:
            with open(self.filename, 'r') as f:
                return json.load(f)
        except json.JSONDecodeError:
            print("[RECORDER ERROR] Journey file '{}' contains invalid JSON data.".format(self.filename), flush=True)
            return None
        except Exception as e:
            print("[RECORDER ERROR] Failed to load journey file: {}".format(e), flush=True)
            return None

    def play_journey(self):
        """Loads and replays the complete recorded mission sequence on the map."""
        journey = self._load_journey_file()
        if journey is None:
            return

        print("[RECORDER] Replaying full mission journey ({:d} actions)...".format(len(journey)), flush=True)
        
        # Zero the gyro before playback to align starting orientation
        self.robot.gyro.zero()
        time.sleep(0.1)

        for action in journey:
            action_type = action["type"]
            params = action["params"]

            if action_type == "drive_gyro":
                dist, heading, speed = params
                print("[REPLAY] Drive Gyro: {:.1f}cm, Heading: {}, Speed: {}".format(dist, heading, speed), flush=True)
                self.robot.drive_distance_gyro(dist, heading, base_speed=speed)

            elif action_type == "follow_line_gyro":
                dist, heading, speed = params
                print("[REPLAY] Follow Line Gyro: {:.1f}cm, Heading: {}, Speed: {}".format(dist, heading, speed), flush=True)
                self.robot.follow_line_gyro_distance(dist, heading, base_speed=speed)

            elif action_type == "follow_line":
                dist, speed = params
                print("[REPLAY] Follow Line Distance: {:.1f}cm, Speed: {}".format(dist, speed), flush=True)
                self.robot.follow_line_for_distance(dist, base_speed=speed)

            elif action_type == "turn":
                deg, speed = params
                print("[REPLAY] Turn to Heading: {:.1f}, Speed: {}".format(deg, speed), flush=True)
                self.robot.navigation.turn_to_heading(math.radians(deg), speed=speed)

            elif action_type == "drive_to_line":
                speed = params[0]
                print("[REPLAY] Seeking line intersection at speed {}".format(speed), flush=True)
                self.robot.drive_to_line(search_speed=speed)

            elif action_type == "lower_arm":
                target = params[0]
                print("[REPLAY] Lower arm to target {}".format(target), flush=True)
                self.robot.lower_arm(target)

            elif action_type == "lift_arm":
                target = params[0]
                print("[REPLAY] Lift arm to target {}".format(target), flush=True)
                self.robot.lift_arm(target)

            elif action_type == "grab_down":
                pos = params[0]
                print("[REPLAY] Grab arm down to position {}".format(pos), flush=True)
                self.robot.grab_arm_down(target_pos=pos)

            elif action_type == "grab_up":
                pos = params[0]
                print("[REPLAY] Grab arm up to position {}".format(pos), flush=True)
                self.robot.grab_arm_up(target_pos=pos)

            elif action_type == "wobble":
                count = params[0]
                print("[REPLAY] Wobble action (count: {})".format(count), flush=True)
                self.robot.drive.wobble(count=count)

            elif action_type == "scan_tray":
                print("[REPLAY] Executing tray scan snapshot", flush=True)
                self.robot.scan_tray_once()

            elif action_type == "wait":
                duration = params[0]
                time.sleep(duration)

            time.sleep(0.02)

        print("[RECORDER] Mission journey playback complete!", flush=True)
        
    def undo_last_action(self):
        """Removes the last logged action while recording is active."""
        if self.recording and self.journey_log:
            removed = self.journey_log.pop()
            print("[RECORDER] Undid last action: {} with params {}".format(
                removed["type"], removed["params"]), flush=True)
        else:
            print("[RECORDER] Nothing to undo or not currently recording.", flush=True)

    def list_journey(self):
        """Prints all saved actions in the file with their index numbers for easy editing."""
        journey = self._load_journey_file()
        if journey is None:
            return

        print("\n--- JOURNEY LOG ({}) ---".format(self.filename))
        for i, action in enumerate(journey):
            print("[{:d}] Type: {} | Params: {}".format(i, action["type"], action["params"]))
        print("----------------------------------------\n")

    def delete_saved_action(self, index):
        """Deletes a specific action from the saved JSON file by its index."""
        journey = self._load_journey_file()
        if journey is None:
            return

        if 0 <= index < len(journey):
            removed = journey.pop(index)
            try:
                with open(self.filename, 'w') as f:
                    json.dump(journey, f, indent=4)
                print("[RECORDER] Deleted action [{:d}] ({type}): {params}".format(
                    index, type=removed["type"], params=removed["params"]), flush=True)
            except Exception as e:
                print("[RECORDER ERROR] Failed to update file: {}".format(e), flush=True)
        else:
            print("[RECORDER ERROR] Index {:d} out of range (0 to {:d}).".format(
                index, len(journey) - 1), flush=True)
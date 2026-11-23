#!/usr/bin/env python3
"""
Robot high‑level mission controller.
Configures hardware, starts background threads and executes the mission.
"""

import sys
import time
import threading
import math
from ev3dev2.motor import  MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, SpeedPercent, LargeMotor
from ev3dev2.sensor.lego import ColorSensor, GyroSensor
from ev3dev2.sensor import INPUT_1, INPUT_4, INPUT_2, INPUT_3

from pose import Pose
from drivetrain import DriveBase
from gyro import GyroHeading
from linefollow1 import LineFollower
from debug import log
from navigation import Navigator
from colors import ColorSensorManager
from odometry import Odometry
from vision import COLOR_NAMES

class Robot:
    def __init__(self):
        # Motors
        self.left_motor = MediumMotor(OUTPUT_A)
        self.right_motor = MediumMotor(OUTPUT_B)
        self.arm_motor = MediumMotor(OUTPUT_D)   
        self.grab_arm = LargeMotor(OUTPUT_C)  

        # Sensors
        self.left_color  = ColorSensor(INPUT_1)
        self.right_color = ColorSensor(INPUT_2)
        self.junction_color = ColorSensor(INPUT_3)   
        self.gyro_sensor = GyroSensor(INPUT_4)

        self.left_color.mode = 'COL-REFLECT'
        self.right_color.mode = 'COL-REFLECT'
        self.junction_color.mode = 'COL-COLOR'
        
        self.camera = None
        self.color = ColorSensorManager(self.left_color, self.right_color, self.junction_color)
        self.vision = None
        self.image_captured_raw = False
        self._vision_loading = False
        
        # Pose (odometry + gyro)
        self.pose = Pose()
        self.gyro = GyroHeading(self.gyro_sensor, self.pose)
        self.odometry = Odometry(self.left_motor,self.right_motor,self.pose,self.gyro,log)
        self.drive = DriveBase(self.left_motor, self.right_motor)
        self.line = LineFollower(self.color, self.drive)
        self.navigation = Navigator(self.pose, self.drive, self.line)
        
        # Initialize default journey recorder
        self.init_recorder("map.json")


        log("ROBOT", "hardware initialised")
        
    # def initialise_vision_async(self):
    #     if self.camera is not None or self._vision_loading:
    #         return  
    #     self._vision_loading = True
        
    #     def _load_worker():
    #         log("VISION_ASYNC","Background loading modules and opening serial")
    #        # from pycam import CameraController
    #         from vision import TrayVision
        
    #        # cam_obj = CameraController()
    #        # vision_obj = TrayVision(cam_obj)
            
    #         self.camera = cam_obj
    #         self.vision = vision_obj
    #         self._vision_loading = False
    #         log("VISION_ASYNC", "Serial channel established and running.")
            
    #     t = threading.Thread(target=_load_worker)
    #     t.daemon = True
    #     t.start()
        
    def reset_arm_encoders(self):
        self.arm_motor.position = 0
        self.grab_arm.position = 0
        log("ARMS","Absolute servo zeros established")
        
    def _run_motor_to_target(self,motor, speed:int,degrees:float,timeout_sec:float=2.0):
        motor.reset()
        target = degrees
        speed_sp = SpeedPercent(speed)
        motor.on_for_degrees(speed_sp, target, block=True,brake=True)
        start_time = time.time()
        while motor.is_running:
            if time.time() - start_time > timeout_sec:
                log("MOTOR_WARN","Timeout reached while running motor to target")
                motor.stop(stop_action = "brake")
                break
            time.sleep(0.01)

    def _move_to_absolute_position(self,motor,speed:int,target_position:int,timeout_sec:float=2.0):
        current_pos = motor.position
        degrees_to_move = target_position - current_pos
        
        if abs(degrees_to_move) < 5:
            return
            
        run_speed = abs(speed)
        motor.stop_action="hold"
        motor.on_for_degrees(SpeedPercent(run_speed), degrees_to_move, block=True, brake=True)

    def lower_arm(self,target=150):
        self._move_to_absolute_position(self.arm_motor,speed=-20,target_position=target)

    def lift_arm(self,target=0):
        self._move_to_absolute_position(self.arm_motor,speed=-20,target_position=target)
    
    def grab_arm_down(self,target_pos=120):
        self._move_to_absolute_position(self.grab_arm,speed=30,target_position=target_pos)
        
    def grab_arm_up(self,target_pos=0):
        self._move_to_absolute_position(self.grab_arm,speed=30,target_position=target_pos)
        
    def follow_line_for_distance(self, distance, base_speed=35, kp=9.5, kd=5.0): #0.66 0.85, 0.8 1.2 ,0.9, 1.3,1.0 1.5, 2.0 1.5,2.5 1.8,2.2 1.7,2.5 1.75,2.8 1.90
        if abs(distance) < 0.1:
            return
        
        forward = distance >= 0
        direction_modifier = 1.0 if forward else -1.0
        target_speed = abs(base_speed)*direction_modifier
        
        start_x, start_y = self.pose.get_position()
        last_error = 0.0
        integral  = 0.0
        ki = 0.001 #0.001,

        while True:
            x, y = self.pose.get_position()
            traveled = math.hypot(x - start_x, y - start_y)
            if traveled >= abs(distance):
                break

            left_reflect = self.left_color.reflected_light_intensity
            right_reflect = self.right_color.reflected_light_intensity

            error = left_reflect - right_reflect
            
            # OPTIMIZATION: Anti-windup. Reset integral if we cross the line center
            if (error > 0 and last_error < 0) or (error < 0 and last_error > 0):
                integral = 0.0
                
            integral += error
            integral = max(min(integral,1000.0),-1000.0)
            
            derivative = error - last_error
            last_error = error

            raw_gain = (kp * error) + (ki*integral)+ (kd * derivative)
            turn_gain = raw_gain*direction_modifier
            
            left_speed = target_speed + turn_gain
            right_speed = target_speed - turn_gain

            self.drive.drive(left_speed, right_speed)
            time.sleep(0.005) 
            
        self.drive.stop()
        
    def follow_line_gyro_distance(self, distance, target_heading_deg, base_speed=35, kp_line=0.3, kp_gyro=1.0, ramp_dist=8.0, min_speed=15):  #kp_line was 0.3
        total_dist = abs(distance)
        if total_dist < 0.1:
            return

        forward = distance >= 0
        direction_modifier = 1.0 if forward else -1.0
        start_x, start_y = self.pose.get_position()
        target_heading_rad = math.radians(target_heading_deg)

        actual_ramp = min(ramp_dist, total_dist / 3.0)

        while True:
            x, y = self.pose.get_position()
            traveled = math.hypot(x - start_x, y - start_y)
            if traveled >= total_dist:
                break

            # 1. Ramped Speed Profile
            dist_remaining = total_dist - traveled
            accel_scale = min(1.0, traveled / actual_ramp) if actual_ramp > 0 else 1.0
            decel_scale = min(1.0, dist_remaining / actual_ramp) if actual_ramp > 0 else 1.0
            speed_scale = min(accel_scale, decel_scale)

            current_abs_speed = min_speed + (abs(base_speed) - min_speed) * speed_scale
            current_speed = current_abs_speed * direction_modifier

            # 2. Line + Gyro Steering Calculation
            left_reflect = self.left_color.reflected_light_intensity
            right_reflect = self.right_color.reflected_light_intensity
            current_heading = self.pose.get_heading()

            line_error = left_reflect - right_reflect
            gyro_error = target_heading_rad - current_heading
            gyro_error = ((gyro_error + math.pi) % (2 * math.pi)) - math.pi

            line_correction = kp_line * line_error
            raw_gyro = kp_gyro * math.degrees(gyro_error)
            gyro_correction = max(min(raw_gyro, 15), -15) 
            
            turn_gain = (line_correction + gyro_correction) * direction_modifier
            
            left_speed = current_speed + turn_gain
            right_speed = current_speed - turn_gain

            self.drive.drive(left_speed, right_speed)
            time.sleep(0.005)

        self.drive.stop()
    
    def scan_tray_once(self):
        log("ROBOT", "Capturing single tray scan...")    
        while self.camera is None:
            time.sleep(0.01)
            
        log("ROBOT","Executing instant tray capture")
        current_grid = self.vision.process_snapshot(robot_instance=self)
        log("[SUCCESS]","Processing complete")
        
        print("\n--- TRAY SCAN RESULT ---")
        for row in current_grid:
            print(" | ".join([COLOR_NAMES[cell] for cell in row]))
        print("------------------------\n")
        return current_grid

    def run_mission(self):
        self.flush_motors()
        # self.initialise_vision_async()
        self.reset_arm_encoders()
       
        self.drive.stop()
        log("[LIFT]","starting")
        self.lift_arm(90)
        self.grab_arm_up()
        log("ROBOT", "Starting mission sequence")
        self.pose.reset()
        log("[ROBOT]","moving out of home")
        self.navigation.turn_to_heading(math.radians(-90), speed=10)
        self.navigation.drive_distance(2.5,30)        
        self.navigation.turn_to_heading(math.radians(0), speed=10)
        self.navigation.drive_distance(20,speed=10)
        # self.drive_to_line()
        self.drive.stop()
        time.sleep(0.05)
        log("[ROBOT]","exited home successfully")
        #entering line for cement bowl
        self.follow_line_gyro_distance(78,0,10) #50
        self.navigation.turn_to_heading(math.radians(-90), speed=50) #50
        
        self.navigation.drive_distance(-2,speed=10)
        self.lower_arm(-180)
        
        
        # positioning to move rectangular trowel
        self.navigation.turn_to_heading(math.radians(-7))
        self.navigation.drive_distance(-20,10)
        self.navigation.turn_to_heading(math.radians(0),10) #50
        self.navigation.drive_distance(-10,10) #50
        self.drive_distance_gyro(23.5,0) # go back a bit
        log("[ROBOT]","done rectangular trowel")
        
        #turn 90 to go back to the line
        self.navigation.turn_to_heading(math.radians(-90))
        # self.navigation.drive_distance(5)
        self.drive_to_line(20)
        self.navigation.turn_to_heading(math.radians(0)) #now aligned with the line
        #moving towards parking area (lot)
        self.follow_line_gyro_distance(75,0,50) #50
        self.navigation.turn_to_heading(math.radians(60))
        self.navigation.drive_distance(-5.5)
        self.lift_arm(90) #
        log("[ROBOT]","Successfully dropped masonry bowl")
        
        #move into line to chase for green elements
        self.navigation.turn_to_heading(math.radians(0),70)
        # self.drive_to_line(15)
        # self.navigation.drive_distance(5,60)
        self.drive.stop()
        self.drive_to_line()
        self.navigation.turn_to_heading(math.radians(-90),70) #now aligned with the line
        self.follow_line_gyro_distance(33,-90,70) # move towards Green Element's junction 30, formerly 35
        
        #turn to green element
        self.navigation.turn_to_heading(math.radians(180))
        self.navigation.drive_distance(-26,80) #now at green elements 50
        
        show = False
        Green = False
        
        if self.color.is_green() and not Green: #check if its at green junction
            log("[GREEN]","I see you")
            self.lower_arm(-180)
            self.drive.stop()
            self.drive.wobble(count=2)
            self.navigation.turn_to_heading(math.radians(180),50)
            
            log("[ROBOT]","turn to 180degrees")
            show = True
            Green = True
            
        if  not Green: # if the sensor missed the green but at the green junction 
            self.lower_arm(-180)
            self.drive.wobble(count=2)
            self.drive.stop()
            self.navigation.turn_to_heading(math.radians(180),50)
            log("[ROBOT]","turn to 180degrees")
            show = True
            Green = True
            
        Yellow = False
        if show:
            log("[ROBOT]","heading to green yard")
            self.navigation.turn_to_heading(math.radians(180),50)
            self.drive_distance_gyro(28,0,60) # moving to drop the green elements
            self.navigation.turn_to_heading(0,50) #turn to face green element yard
            self.drive.stop()
            # time.sleep(0.1)
            # self.follow_line_gyro_distance(-27,0,60)
            # # moving to drop the green elements
            # self.drive_distance_gyro(-25,0,60)
            self.drive_distance_gyro(-15,0,70) # formally self.drive_distance_gyro(-25,0,60)
            self.navigation.turn_to_heading(math.radians(0),70)
            self.drive_distance_gyro(-10,0,70) # formally self.drive_distance_gyro(-25,0,60)
            self.navigation.turn_to_heading(math.radians(0),70)
            self.drive_distance_gyro(-5,0,70)
            
            if self.color.is_green() and Green:
                self.drive.stop()
                self.lift_arm(90)
                log("[HALLELUJAH]","PRAISE THE LORD")
                self.navigation.turn_to_heading(0,45)
                Yellow = True
            
            if Green and not self.color.is_green() and not Yellow:  #i
                self.drive.stop()
                self.lift_arm(90)
                # self.navigation.drive_distance(5,50)
                self.navigation.turn_to_heading(0,45)
                Yellow = True
            self.replay_record()
        # triangle = False
        
        # next = False
        # if Yellow:
        #     self.drive_distance_gyro(5,0) # formerly navigation.drive_distance(5,0)
        #     self.navigation.turn_to_heading(0,60)
        #     self.drive_distance_gyro(25,0) #35 now about to pick yellow, at the green junction
        #     self.navigation.turn_to_heading(math.radians(-90),80)
        #     self.drive_distance_gyro(30,-90,60)
        #     self.drive_distance_gyro(-6,-90,60)
        #     self.navigation.turn_to_heading(math.radians(180),90)#80
        #     time.sleep(0.1)
        #     self.navigation.drive_distance(-29,60) #formerly follow line for distance #-20,90,50
        #     self.drive.stop()
        #     next = True
            
        #     if self.color.is_yellow():
        #         log("[Yellow]","Detected Color Yellow")
        #         self.lower_arm(-180)
        #         time.sleep(0.1)
        #         self.drive.wobble(count=2)
        #         self.drive.stop()

        #         self.navigation.turn_to_heading(180,45) #heading out of yellow elements area
        #         #self.drive_distance_gyro(-30,0,90)
        #         self.follow_line_gyro_distance(30,180,70) #-30,0,60 #formerly follow line for dist(ffld)
        #         self.drive.stop()
        #         self.navigation.turn_to_heading(math.radians(90),50) #

        #         #self.drive_distance_gyro(-80,-90,50,accel=True) #ffld
        #         self.follow_line_gyro_distance(40,90,50) #driving towards masonry trowel
        #         self.navigation.turn_to_heading(math.radians(-90),50) #
        #         self.follow_line_gyro_distance(-40,-90,50) # still driving towards masonry trowel

        #         self.navigation.turn_to_heading(0,70)
        #         self.drive_distance_gyro(-80,0,70) #ffld 
        #         self.navigation.drive_distance(10)
        #         self.navigation.turn_to_heading(math.radians(90),80)
        #         self.drive_distance_gyro(-40,90,70)#-35,-90,70
        #         self.lift_arm(90)
        #         self.drive_distance_gyro(30,90,70)
        #         self.navigation.turn_to_heading(math.radians(0))
        #         self.drive_distance_gyro(-40,0,70)
        #         next = False

        # if not self.color.is_yellow() and next:
        #     self.lower_arm(-180)
        #     time.sleep(0.1)
        #     self.drive.wobble(count=2)
        #     self.drive.stop()

        #     self.navigation.turn_to_heading(180,45) #heading out of yellow elements area
        #     #self.drive_distance_gyro(-30,0,90)
        #     self.follow_line_gyro_distance(30,180,70) #-30,0,60 #formerly follow line for dist(ffld)
        #     self.drive.stop()
        #     self.navigation.turn_to_heading(math.radians(90),50) #

        #     #self.drive_distance_gyro(-80,-90,50,accel=True) #ffld
        #     self.follow_line_gyro_distance(40,90,50) #driving towards masonry trowel
        #     self.navigation.turn_to_heading(math.radians(-90),50) #
        #     self.follow_line_gyro_distance(-40,-90,50) # still driving towards masonry trowel

        #     self.navigation.turn_to_heading(0,70)
        #     self.drive_distance_gyro(-80,0,70) #ffld 
        #     self.navigation.drive_distance(10)
        #     self.navigation.turn_to_heading(math.radians(90),80)
        #     self.drive_distance_gyro(-40,90,70)#-35,-90,70
        #     self.lift_arm(90)
        #     self.drive_distance_gyro(30,90,70)
        #     self.navigation.turn_to_heading(math.radians(0))
        #     self.drive_distance_gyro(-40,0,70)
        #     next = False

        # if triangle:
        #     self.navigation.turn_to_heading(math.radians(35),80)
        #     self.drive_distance_gyro(30,35,70)
            
        # self.navigation.turn_to_heading(math.radians(35),80)
        # self.lower_arm(-180)
        # self.drive_distance_gyro(20,0,30)
        
        # # self.navigation.goto_position(50,45)
        # self.mosaic_mission()
        # # self.camera.send_command('b')
        # # my_grid = self.scan_tray_once()
        # self.load_and_deploy_slide()
        self.drive.stop()

    def mosaic_mission(self):
        self.lift_arm(90)
        self.navigation.goto_position(10,5)
        self.navigation.turn_to_heading(math.radians(-90),70)
        self.follow_line_gyro_distance(45,-90,50)
        self.navigation.turn_to_heading(0,60)
        self.follow_line_gyro_distance(40,0,80)
        self.drive.stop()
        my_grid = self.scan_tray_once()

        pass

    def setup_supply_inventory(self):
        """
        Hardcoded global Odometry (X, Y) coordinates for the 3x2 supply yards.
        (You will need to physically measure these X,Y points on your mat)
        """
        from vision import GREEN, YELLOW, BLUE, WHITE
        
        self.supply_inventory = {
            # Example: 3 columns, 2 rows of Green blocks
            YELLOW: [
                (18.5, 12.5),(18.5,12.5), #front row
                (12, 18.5), (12,12.5), # middle row
                (6, 18.5), (6, 12.5)  # Back row
            ],
            BLUE: [
                (18.5, 28), (18.5, 35),
                (12, 28),(12, 35),
                (6, 28), (6, 35)
            ],
            GREEN: [
                (18.5, 44), (18.5, 50),
                (12, 44),(12, 50),
                (6, 44), (6, 50)
            ],
            WHITE: [
                (18.5, 60), (18.5, 67),
                (12, 60), (12, 67),
                (6, 60), (6, 67)
            ]
        }

    def load_and_deploy_slide(self):
        """
        Scans the blueprint, gathers all required pieces from the supply yards,
        sorts them into the onboard 3-column slide using the ESP32 servo, 
        and delivers the full payload to the tray.
        """
        from vision import UNKNOWN, COLOR_NAMES
        
        # 1. Initialize the 3x2 supply tracking
        self.setup_supply_inventory()
        
        # 2. Map matrix columns to ESP32 servo serial commands
        column_to_servo = {
            0: 'a',  # Column 0 routes via command 'a'
            1: 'b',  # Column 1 routes via command 'b'
            2: 'c'   # Column 2 routes via command 'c'
        }
        
        # 3. Get the 4x3 Blueprint
        log("MISSION", "Scanning target tray...")
        self.camera.send_command('s')  # Trigger ESP32 if needed[cite: 3]
        blueprint_grid = self.scan_tray_once()[cite: 4]
        
        # 4. Fill the onboard slide
        # CRITICAL: Determine your loading order! 
        # If the first block dropped into the slide ends up at the BOTTOM of the tray,
        # you need to load Row 3 first, then Row 2, Row 1, Row 0.
        # We use reversed(range(4)) here assuming First-In-First-Out.
        for row in reversed(range(4)):
            for col in range(3):
                target_color = blueprint_grid[row][col]
                
                if target_color == UNKNOWN:
                    continue
                    
                log("LOADER", "Fetching {} for Col {}".format(COLOR_NAMES[target_color], col))
                
                # --- A. DRIVE TO SUPPLY & PICK UP ---
                pickup_pos = self.get_next_pickup_coordinate(target_color)
                if not pickup_pos:
                    log("LOADER", "Skipping: Out of {} blocks!".format(COLOR_NAMES[target_color]))
                    continue 
                    
                pick_x, pick_y = pickup_pos
                self.navigation.goto_position(pick_x, pick_y)[cite: 2]
                
                # Grab the block from the yard
                self.grab_arm_up()[cite: 4]
                self.lower_arm(-180)[cite: 4]
                time.sleep(0.3)
                self.grab_arm_down(target_pos=150)[cite: 4]
                time.sleep(0.3)
                self.lift_arm()[cite: 4]
                
                # --- B. ROUTE THE SLIDE SERVO ---
                # Tell the ESP32 to move the routing servo to the correct column
                servo_cmd = column_to_servo[col]
                self.camera.send_command(servo_cmd)[cite: 3]
                
                # Give the ESP32 servo a fraction of a second to physically move into position
                time.sleep(0.4) 
                
                # --- C. DROP INTO ROBOT SLIDE ---
                # Position arm over the slide (if needed, add arm rotation here)
                self.grab_arm_up() # Release block into the hopper[cite: 4]
                time.sleep(0.2)
                
        # 5. Drive to the target tray for mass deployment
        log("MISSION", "Magazine fully loaded. Proceeding to target tray.")
        TRAY_DEPLOY_X = 100.0
        TRAY_DEPLOY_Y = 20.0
        self.navigation.goto_position(TRAY_DEPLOY_X, TRAY_DEPLOY_Y)[cite: 2]
        
        # 6. Trigger your physical slide release mechanism here
        # e.g., self.release_slide_motor.on_for_degrees(SpeedPercent(50), 90)
        log("MISSION", "Payload deployed!")

    def get_next_pickup_coordinate(self, target_color):
        """Removes and returns the first available (X, Y) location for a color."""
        if len(self.supply_inventory[target_color]) > 0:
            return self.supply_inventory[target_color].pop(0)
        else:
            log("ERROR", "Out of inventory for this color!")
            return None

    def drive_to_line(self, search_speed=45, black_threshold=15):
        """Drives forward to find a black line, then squares up perfectly on it."""
        log("NAV", "Searching for line intersection...")
        
        self.drive.forward(search_speed)
        while True:
            left = self.left_color.reflected_light_intensity
            right = self.right_color.reflected_light_intensity
            if left <= black_threshold and right <= black_threshold:
                break
            time.sleep(0.002)
        
        while True:
            left = self.left_color.reflected_light_intensity
            right = self.right_color.reflected_light_intensity
            
            if left <= black_threshold and right <= black_threshold:
                break
                
            l_motor_speed = -5 if left > black_threshold else 5
            r_motor_speed = -5 if right > black_threshold else 5
            
            self.drive.drive(l_motor_speed, r_motor_speed)
            time.sleep(0.002)
            
    
            
        self.drive.stop()
        time.sleep(0.05)
        # OPTIMIZATION: Removed self.pose.set_position(0.0, 0.0)
        # Your global odometry map will no longer violently reset here, allowing distance tracking to work.
        log("NAV", "Perfectly squared and centered on track line.")
        
        
    
    # --- ADD THESE METHODS TO YOUR Robot CLASS IN robot.py ---

    # def init_recorder(self, filename="journey_log.json"):
    #     from recorder import JourneyRecorder
    #     self.recorder = JourneyRecorder(self, filename=filename)

    # def record_drive_distance(self, distance, target_heading_deg, base_speed=50):
    #     """Wrapper that executes drive_distance_gyro and logs it."""
    #     if hasattr(self, 'recorder') and self.recorder.recording:
    #         self.recorder.log_action("drive", [distance, base_speed, target_heading_deg])
    #     self.drive_distance_gyro(distance, target_heading_deg, base_speed=base_speed)

    # def record_turn(self, target_heading_deg, speed=50):
    #     """Wrapper that executes turn_to_heading and logs it."""
    #     if hasattr(self, 'recorder') and self.recorder.recording:
    #         self.recorder.log_action("turn", [target_heading_deg, speed])
    #     self.navigation.turn_to_heading(math.radians(target_heading_deg), speed=speed)
    def resume_recording(self):
        """Resumes recording, keeping existing logged actions and appending new ones."""
        # Load from file if memory log is currently empty
        if not self.journey_log:
            loaded = self._load_journey_file()
            if loaded is not None:
                self.journey_log = loaded
        
        self.recording = True
        
        # Adjust start time so new timestamps continue sequentially from the last action
        if self.journey_log:
            last_time = self.journey_log[-1].get("time", 0.0)
            self._start_time = time.time() - last_time
        else:
            self._start_time = time.time()
            
        print("[RECORDER] Resumed recording. Total actions in memory: {:d}".format(len(self.journey_log)), flush=True)
        
    def replay_record(self,file="map.json"):
        self.init_recorder("map.json")
        self.recorder.play_journey()
        
        
    
    def init_recorder(self, filename="map.json"):
        from recorder import JourneyRecorder
        self.recorder = JourneyRecorder(self, filename=filename)

    def rec_drive_gyro(self, distance, target_heading_deg, base_speed=50):
        if hasattr(self, 'recorder') and self.recorder.recording:
            self.recorder.log_action("drive_gyro", [distance, target_heading_deg, base_speed])
        self.drive_distance_gyro(distance, target_heading_deg, base_speed=base_speed)

    def rec_follow_line_gyro(self, distance, target_heading_deg, base_speed=35):
        if hasattr(self, 'recorder') and self.recorder.recording:
            self.recorder.log_action("follow_line_gyro", [distance, target_heading_deg, base_speed])
        self.follow_line_gyro_distance(distance, target_heading_deg, base_speed=base_speed)

    def rec_follow_line(self, distance, base_speed=35):
        if hasattr(self, 'recorder') and self.recorder.recording:
            self.recorder.log_action("follow_line", [distance, base_speed])
        self.follow_line_for_distance(distance, base_speed=base_speed)

    def rec_turn(self, target_heading_deg, speed=50):
        if hasattr(self, 'recorder') and self.recorder.recording:
            self.recorder.log_action("turn", [target_heading_deg, speed])
        self.navigation.turn_to_heading(math.radians(target_heading_deg), speed=speed)

    def rec_drive_to_line(self, search_speed=45):
        if hasattr(self, 'recorder') and self.recorder.recording:
            self.recorder.log_action("drive_to_line", [search_speed])
        self.drive_to_line(search_speed=search_speed)

    def rec_lower_arm(self, target=150):
        if hasattr(self, 'recorder') and self.recorder.recording:
            self.recorder.log_action("lower_arm", [target])
        self.lower_arm(target)

    def rec_lift_arm(self, target=0):
        if hasattr(self, 'recorder') and self.recorder.recording:
            self.recorder.log_action("lift_arm", [target])
        self.lift_arm(target)

    def rec_grab_down(self, target_pos=120):
        if hasattr(self, 'recorder') and self.recorder.recording:
            self.recorder.log_action("grab_down", [target_pos])
        self.grab_arm_down(target_pos=target_pos)

    def rec_grab_up(self, target_pos=0):
        if hasattr(self, 'recorder') and self.recorder.recording:
            self.recorder.log_action("grab_up", [target_pos])
        self.grab_arm_up(target_pos=target_pos)

    def rec_wobble(self, count=2):
        if hasattr(self, 'recorder') and self.recorder.recording:
            self.recorder.log_action("wobble", [count])
        self.drive.wobble(count=count)

    def rec_scan_tray(self):
        if hasattr(self, 'recorder') and self.recorder.recording:
            self.recorder.log_action("scan_tray", [])
        return self.scan_tray_once()

    def resume_recorder(self):
        if hasattr(self, 'recorder'):
            self.recorder.resume_recording()
        else:
            print("[ROBOT ERROR] Recorder not initialized.")       
    
    def start(self):
        self.gyro.start()
        self.odometry.start()
        self.line.start()
        log("ROBOT", "Background threads active")

    def stop(self):
        self.lift_arm(90)
        self.arm_motor.stop(stop_action="coast")
        self.grab_arm.stop(stop_action="coast")
        self.line.stop()
        self.gyro.stop()
        self.drive.stop()
        self.odometry.stop()
        if self.camera and not isinstance(self.camera,str):
            self.camera.disconnect()
        try:
            if self.vision is not None:
                self.vision.stop()
        except RuntimeError:
            pass
        self.flush_motors()
        log("ROBOT", "All devices shutdown safely")

    # def drive_distance_gyro(self, distance, target_heading_deg, base_speed=50, kp_gyro=2.0, 
    #                         decel_dist=8.0, accel=False, decel=True, min_speed=15):
    #     """
    #     Drives straight with optional acceleration and deceleration.
    #     Defaults to Decel-Only for a fast start and smooth stopping.
    #     """
    #     total_dist = abs(distance)
    #     if total_dist < 0.1:
    #         return

    #     forward = distance >= 0
    #     direction_modifier = 1.0 if forward else -1.0
    #     start_x, start_y = self.pose.get_position()
    #     target_heading_rad = math.radians(target_heading_deg)

    #     actual_decel = min(decel_dist, total_dist / 2.0)

    #     while True:
    #         x, y = self.pose.get_position()
    #         traveled = math.hypot(x - start_x, y - start_y)
    #         if traveled >= total_dist:
    #             break

    #         dist_remaining = total_dist - traveled

    #         # Calculate speed scale
    #         accel_scale = min(1.0, traveled / actual_decel) if (accel and actual_decel > 0) else 1.0
    #         decel_scale = min(1.0, dist_remaining / actual_decel) if (decel and actual_decel > 0) else 1.0
    #         speed_scale = min(accel_scale, decel_scale)

    #         # Interpolate target speed
    #         current_abs_speed = min_speed + (abs(base_speed) - min_speed) * speed_scale
    #         current_speed = current_abs_speed * direction_modifier

    #         # Gyro correction
    #         current_heading = self.pose.get_heading()
    #         gyro_error = target_heading_rad - current_heading
    #         gyro_error = ((gyro_error + math.pi) % (2 * math.pi)) - math.pi

    #         raw_gyro = kp_gyro * math.degrees(gyro_error)
    #         gyro_correction = max(min(raw_gyro, 25), -25)

    #         turn_gain = gyro_correction * direction_modifier

    #         self.drive.drive(current_speed + turn_gain, current_speed - turn_gain)
    #         time.sleep(0.005)

    #     self.drive.stop()
    def drive_distance_gyro(self, distance, target_heading_deg, base_speed=50, kp_gyro=2.0, 
                            decel_dist=8.0, accel=False, decel=True, min_speed=15):
        """
        Drives straight using self.pose for distance tracking and gyro for heading maintenance,
        avoiding multi-threading collisions with the background odometry thread.
        """
        total_dist = abs(distance)
        if total_dist < 0.1:
            return

        forward = distance >= 0
        direction_modifier = 1.0 if forward else -1.0
        
        # Track distance safely using the odometry pose (updated by background thread)
        start_x, start_y = self.pose.get_position()
        target_heading_rad = math.radians(target_heading_deg)
        actual_decel = min(decel_dist, total_dist / 2.0)

        while True:
            x, y = self.pose.get_position()
            traveled_cm = math.hypot(x - start_x, y - start_y)
            
            if traveled_cm >= total_dist:
                break

            dist_remaining = total_dist - traveled_cm

            # 1. Ramped Speed Profile
            accel_scale = min(1.0, traveled_cm / actual_decel) if (accel and actual_decel > 0) else 1.0
            decel_scale = min(1.0, dist_remaining / actual_decel) if (decel and actual_decel > 0) else 1.0
            speed_scale = min(accel_scale, decel_scale)

            current_abs_speed = min_speed + (abs(base_speed) - min_speed) * speed_scale
            current_speed = current_abs_speed * direction_modifier

            # 2. Gyro Correction with correct polarity
            current_heading = self.pose.get_heading()
            gyro_error = target_heading_rad - current_heading
            gyro_error = ((gyro_error + math.pi) % (2 * math.pi)) - math.pi

            raw_gyro = kp_gyro * math.degrees(gyro_error)
            gyro_correction = max(min(raw_gyro, 25), -25)

            if forward:
                left_speed = current_speed - gyro_correction
                right_speed = current_speed + gyro_correction
            else:
                left_speed = current_speed + gyro_correction
                right_speed = current_speed - gyro_correction

            self.drive.drive(left_speed, right_speed)
            time.sleep(0.005)

        self.drive.stop()
    def flush_motors(self, unjam=False):
        """
        Stops all motors, clears error flags, releases active holds,
        and resets encoder positions back to zero.
        """
        log("MOTORS", "Flushing all motor ports...")
        motors = [self.left_motor, self.right_motor, self.arm_motor, self.grab_arm]
        
        for motor in motors:
            try:
                motor.stop(stop_action="coast") # Release holding brake torque
                motor.reset()                  # Reset internal pid & encoder counts
            except Exception as e:
                log("MOTOR_WARN","Failed to flush motor: {0}".format(e))

        # Optional mechanical purge: wiggles motors back and forth to clear stuck mechanisms
        if unjam:
            log("MOTORS", "Running physical un-jam cycle...")
            for motor in motors:
                try:
                    motor.on_for_degrees(SpeedPercent(20), 15, block=False)
                    time.sleep(0.05)
                    motor.on_for_degrees(SpeedPercent(-20), 15, block=False)
                    time.sleep(0.05)
                    motor.stop(stop_action="coast")
                except Exception:
                    pass

        log("MOTORS", "Flush complete. All motors coasting.")
if __name__ == "__main__":
    bot = Robot()
    try:
        bot.start()
        bot.reset_arm_encoders()
        # bot.initialise_vision_async()
        bot.init_recorder("map.json")
        
        if len(sys.argv) > 1 and sys.argv[1] == "--test":
            print("\n" + "="*50)
            print("ROBOT INTERACTIVE TESTING TERMINAL ACTIVE")
            print("="*50)
            print("Available shortcuts:")
            print("  m()               -> Run full mission sequence")
            print("  f(dist, speed)    -> Drive distance (e.g. f(5, 50))")
            print("  t(deg, speed)     -> Turn to heading angle in DEGREES")
            print("  lf(dist)          -> Follow line for distance cm")
            print("  la() / ua()       -> Lower Arm / Lift Arm")
            print("  gd() / gu()       -> Grab Arm Down / Grab Arm Up")
            print("  exit              -> Terminate shell safely")
            print("="*50 + "\n")
            
            shell_context = {
                'bot': bot,
                'm': bot.run_mission,
                'la': lambda target: bot.lower_arm(target),
                'ua': lambda target: bot.lift_arm(target),
                'gd':lambda target: bot.grab_arm_down(target_pos=target),
                'gu': lambda target: bot.grab_arm_up(target_pos=target),
                'f': lambda dist, speed=50: bot.navigation.drive_distance(dist, speed=speed),
                't': lambda deg, speed=40: bot.navigation.turn_to_heading(math.radians(deg), speed=speed),
                'lf': lambda dist, speed=40: bot.follow_line_for_distance(dist, base_speed=speed),
                'math': math,
                'fl':lambda dist,speed=50: bot.follow_line_for_distance(dist,base_speed=speed,kp=0.3,kd=0.81),
                's':lambda bte: bot.camera.send_command(bte),
                'grid': lambda: print("\n".join([" | ".join([COLOR_NAMES[cell] for cell in row]) for row in bot.vision.get_grid()])),
                'scan':bot.scan_tray_once,
                'goto': lambda x,y : bot.navigation.goto_position(x,y),
                'wob':bot.drive.wobble,
                'mos':bot.mosaic_mission,
                'mos1':bot.load_and_deploy_slide,
                'start':bot.recorder.start_recording,
                'rd':lambda dist,heading,speed:bot.rec_drive_gyro(dist,heading,speed),
                'rt':lambda heading,speed:bot.rec_turn(heading,speed),
                'stop':bot.recorder.stop_recording,
                'rep': bot.replay_record,
                'del': lambda num: bot.recorder.delete_saved_action(num),
                'undo':bot.recorder.undo_last_action,
                'rla':bot.rec_lift_arm,
                'rlo':bot.rec_lower_arm,
                'rwob':bot.rec_wobble,
                'flg': lambda dist,heading,speed: bot.rec_follow_line_gyro(dist,heading,speed),
                'dg':lambda dist,heading:bot.rec_drive_gyro(dist,heading,base_speed=60),
                'start': bot.recorder.start_recording,
                'resume': bot.resume_recorder
            }
            
            while True:
                try:
                    cmd = input("bot-shell> ").strip()
                    if cmd.lower() in ['exit','quit','q']:
                        break
                    if not cmd:
                        continue
                    eval(cmd,{"__builtins__":__builtins__},shell_context)
                except Exception as e:
                    print("Exiting shell due to error: ", e)
                    # bot.stop()
                    # import traceback
                    # traceback.print_exc()
        else:
            bot.run_mission()

    finally:
        bot.stop()
        
        
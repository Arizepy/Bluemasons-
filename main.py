#!/usr/bin/env python3

# from robot import Robot
# from ev3dev2.sound import Sound


# robot = Robot()
# sound = Sound()


# try:
#     robot.start()
#     sound.beep()
#     robot.run_mission()
# except KeyboardInterrupt:
#     print("Keyboard interrupt")
#     sound.beep()
# finally:
#     robot.stop()
#     sound.beep()



#!/usr/bin/env python3
"""
EV3 Robot Main - Loads then waits for Enter button
"""

from ev3dev2.button import Button
from ev3dev2.sound import Sound
from ev3dev2.led import Leds
import time

from robot import Robot

robot = Robot()
sound = Sound()
leds = Leds()
btn = Button()

print("=== LOADING SYSTEMS ===")
sound.beep()

robot.start()
robot.reset_arm_encoders()
#robot.initialise_vision_async()

leds.set_color("LEFT", "ORANGE")
leds.set_color("RIGHT", "ORANGE")

print("Waiting for vision...")
while robot._vision_loading:
    time.sleep(0.3)

print(" READY - Press ENTER to start mission")
sound.beep()
leds.set_color("LEFT", "GREEN")
leds.set_color("RIGHT", "GREEN")

# Wait for Enter
while not btn.enter:
    time.sleep(0.05)

print(" MISSION START!")
sound.beep()
leds.set_color("LEFT", "AMBER")
leds.set_color("RIGHT", "AMBER")

try:
    robot.run_mission()
except KeyboardInterrupt:
    print("Interrupted")
except Exception as e:
    print("Error:", e)
finally:
    robot.stop()
    leds.set_color("LEFT", "RED")
    leds.set_color("RIGHT", "RED")
    sound.beep()
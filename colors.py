#!/usr/bin/env python3
import threading
from ev3dev2.sensor.lego import ColorSensor
from worker import StoppableThread
from debug import log

# ev3dev2 releases expose this as either COLOR_NONE or COLOR_NOCOLOR.
COLOR_NONE = getattr(ColorSensor, 'COLOR_NONE',
                     getattr(ColorSensor, 'COLOR_NOCOLOR', 0))


class ColorSensorManager(StoppableThread):
    poll_interval = 0.01

    def __init__(self, left_sensor, right_sensor, junction_sensor):
        super().__init__()

        self.left_sensor = left_sensor
        self.right_sensor = right_sensor
        self.junction_sensor = junction_sensor

        self.lock = threading.Lock()

        # latest sensor values
        self.left_reflection = 0
        self.right_reflection = 0
        self.junction_color = COLOR_NONE
        self._last_reported_junction = COLOR_NONE

        self.left_black = 0
        self.left_white = 100
        self.right_black = 0
        self.right_white = 100

        self.left_sensor.mode = 'COL-REFLECT'
        self.right_sensor.mode = 'COL-REFLECT'
        self.junction_sensor.mode = 'COL-COLOR'

    def calibrate_left(self, black, white):
        self.left_black = black
        self.left_white = white

    def calibrate_right(self, black, white):
        self.right_black = black
        self.right_white = white

    @staticmethod
    def normalize(value, black, white):
        if white == black:
            return 0
        result = (value - black) / (white - black)
        return max(0, min(1, result))

    def get_line_position(self):
        with self.lock:
            left = self.normalize(self.left_reflection, self.left_black, self.left_white)
            right = self.normalize(self.right_reflection, self.right_black, self.right_white)
            return left, right

    def loop(self):
        try:
            left = self.left_sensor.reflected_light_intensity
            right = self.right_sensor.reflected_light_intensity
            junction = self.junction_sensor.color

            with self.lock:
                self.left_reflection = left
                self.right_reflection = right
                self.junction_color = junction

            if junction != self._last_reported_junction:
                log("SENSORS", "Junction colour changed to {}".format(junction))
                self._last_reported_junction = junction

        except Exception as e:
            print("color sensor error:", e)

    # line following values
    def get_line_following(self):
        with self.lock:
            return self.left_reflection, self.right_reflection

    def get_left_reflection(self):
        with self.lock:
            return self.left_reflection

    def get_right_reflection(self):
        with self.lock:
            return self.right_reflection

    # junction color
    def get_junction_color(self):
        with self.lock:
            return self.junction_color

    def is_color(self, color):
        with self.lock:
            return self.junction_color == color

    def is_black(self):
        return self.is_color(ColorSensor.COLOR_BLACK)

    def is_yellow(self):
        return self.is_color(ColorSensor.COLOR_YELLOW)

    def is_green(self):
        return self.is_color(ColorSensor.COLOR_GREEN)

    def is_blue(self):
        return self.is_color(ColorSensor.COLOR_BLUE)

    def is_white(self):
        return self.is_color(ColorSensor.COLOR_WHITE)


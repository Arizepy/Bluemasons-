#!/usr/bin/env python3
"""
linefollow1.py

Reusable PID line follower for the MaBot.

Reads the two downward-facing colour sensors through ``ColorSensorManager``
(see colors.py) and drives the motors through ``DriveBase`` (see drivetrain.py).

The follower runs as a background ``StoppableThread`` so the caller can
enable/disable it independently of other subsystems.

Typical use from robot.py::

    bot.line = LineFollower(bot.color, bot.drive)
    bot.line.start()

    bot.line.enable(base_speed=35)          # start following
    ...
    bot.line.disable()                       # stop following

    # or one-shot helpers:
    bot.line.follow_for_distance(50, bot.pose)
    bot.line.follow_until_color(ColorSensor.COLOR_GREEN)
"""

import threading
import time
import math

from worker import StoppableThread


class LineFollower(StoppableThread):
    """
    PID line follower driven by the two reflective colour sensors.

    Error signal
    ------------
    ``error = left_norm - right_norm``  (both in [0, 1])

    * positive  → the left sensor sees more white, line is under the
                  right sensor, robot drifted LEFT  → steer RIGHT
    * negative  → the mirror image                     → steer LEFT

    Control
    -------
    The PID output is a differential speed applied symmetrically around
    the base speed::

        left_speed  = target + correction * direction
        right_speed = target - correction * direction

    Multiplying by ``direction`` flips the steering sense when driving
    in reverse, so the robot always turns *toward* the line regardless
    of whether it is moving forward or backward (this matches the
    convention used by ``Robot.follow_line_for_distance``).
    """

    poll_interval = 0.005          # ~200 Hz control loop (matches robot.py)

    # ------------------------------------------------------------------
    # Default tunables
    # ------------------------------------------------------------------
    DEFAULT_BASE_SPEED     = 30.0
    DEFAULT_KP             = 50.0
    DEFAULT_KI             = 0.0
    DEFAULT_KD             = 10.0
    DEFAULT_MAX_CORRECTION = 70.0
    DEFAULT_MAX_INTEGRAL   = 1.0   # anti-windup clamp on ∫error·dt

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------
    def __init__(self, color_manager, drivebase,
                 base_speed=DEFAULT_BASE_SPEED,
                 kp=DEFAULT_KP, ki=DEFAULT_KI, kd=DEFAULT_KD,
                 max_correction=DEFAULT_MAX_CORRECTION,
                 max_integral=DEFAULT_MAX_INTEGRAL):
        super().__init__()
        self.color = color_manager
        self.drive = drivebase

        # Tunables
        self.base_speed     = float(base_speed)
        self.kp             = float(kp)
        self.ki             = float(ki)
        self.kd             = float(kd)
        self.max_correction = float(max_correction)
        self.max_integral   = float(max_integral)

        # Runtime state (guarded by _lock)
        self._lock         = threading.Lock()
        self._enabled      = False
        self._direction    = 1.0                # +1 forward, -1 reverse
        self._integral     = 0.0
        self._last_error   = 0.0
        self._last_time    = time.time()
        self._last_error_v = 0.0                # for external introspection

    # ------------------------------------------------------------------
    # Configuration
    # ------------------------------------------------------------------
    def configure(self, base_speed=None, kp=None, ki=None, kd=None,
                  max_correction=None, max_integral=None):
        """Update PID gains / limits without starting the controller."""
        with self._lock:
            if base_speed     is not None: self.base_speed     = float(base_speed)
            if kp             is not None: self.kp             = float(kp)
            if ki             is not None: self.ki             = float(ki)
            if kd             is not None: self.kd             = float(kd)
            if max_correction is not None: self.max_correction = float(max_correction)
            if max_integral   is not None: self.max_integral   = float(max_integral)

    # ------------------------------------------------------------------
    # Enable / disable
    # ------------------------------------------------------------------
    def enable(self, direction=1, base_speed=None, kp=None, ki=None, kd=None):
        """Begin actively following the line (background thread does the work)."""
        self.configure(base_speed=base_speed, kp=kp, ki=ki, kd=kd)
        with self._lock:
            self._direction  = 1.0 if direction >= 0 else -1.0
            self._integral   = 0.0
            self._last_error = 0.0
            self._last_time  = time.time()
            self._enabled    = True

    def disable(self, stop_motors=True):
        """Stop following the line and (by default) release the motors."""
        with self._lock:
            self._enabled = False
        if stop_motors:
            try:
                self.drive.stop()
            except Exception:
                pass

    def is_enabled(self):
        with self._lock:
            return self._enabled

    def get_error(self):
        with self._lock:
            return self._last_error_v

    # ------------------------------------------------------------------
    # Main control loop — called repeatedly by StoppableThread
    # ------------------------------------------------------------------
    def loop(self):
        with self._lock:
            if not self._enabled:
                return
            base_speed = self.base_speed
            direction  = self._direction
            kp, ki, kd = self.kp, self.ki, self.kd
            max_corr   = self.max_correction
            max_int    = self.max_integral

        # Normalised reflection (0 = black line, 1 = white background)
        left, right = self.color.get_line_position()
        error = left - right

        now = time.time()
        with self._lock:
            dt = now - self._last_time
            if dt <= 0.0:
                dt = self.poll_interval

            # Anti-windup: drop the integral whenever the line is crossed
            if error * self._last_error < 0.0:
                self._integral = 0.0
            self._integral += error * dt
            self._integral = max(min(self._integral, max_int), -max_int)

            derivative         = (error - self._last_error) / dt
            self._last_error   = error
            self._last_time    = now
            integral           = self._integral
            self._last_error_v = error

        # PID output, clamped
        correction = (kp * error) + (ki * integral) + (kd * derivative)
        correction = max(min(correction, max_corr), -max_corr)

        # Flipping by direction keeps "steer toward the line" true in reverse
        target              = base_speed * direction
        effective_correction = correction * direction
        left_speed          = target + effective_correction
        right_speed         = target - effective_correction

        try:
            self.drive.drive(left_speed, right_speed)
        except Exception:
            # Never kill the polling thread on a transient motor fault
            pass

    # ------------------------------------------------------------------
    # Synchronous helpers (block the caller, run the thread underneath)
    # ------------------------------------------------------------------
    def follow_for_time(self, seconds, **kwargs):
        """Follow the line for a fixed duration, then stop."""
        self.enable(**kwargs)
        try:
            deadline = time.time() + seconds
            while time.time() < deadline and self.is_enabled():
                time.sleep(0.01)
        finally:
            self.disable()

    def follow_for_distance(self, distance_cm, pose, **kwargs):
        """
        Follow the line until ``distance_cm`` of travel is measured.

        ``pose`` should be the robot's ``Pose`` object (fed by the odometry
        thread) so distance is measured from real travel, not from a
        guessed wheel rotation.
        """
        if abs(distance_cm) < 0.1:
            return
        direction = 1 if distance_cm >= 0 else -1
        self.enable(direction=direction, **kwargs)

        start_x, start_y = pose.get_position()
        target = abs(distance_cm)
        try:
            while self.is_enabled():
                x, y = pose.get_position()
                if math.hypot(x - start_x, y - start_y) >= target:
                    break
                time.sleep(0.005)
        finally:
            self.disable()

    def follow_until_color(self, color, timeout=None, **kwargs):
        """
        Follow the line until the junction colour sensor reports ``color``.

        ``color`` may be a single constant (e.g. ``ColorSensor.COLOR_GREEN``)
        or an iterable of constants.
        Returns ``True`` on detection, ``False`` on timeout / stop.
        """
        colors = (color,) if isinstance(color, int) else tuple(color)

        self.enable(**kwargs)
        deadline = None if timeout is None else time.time() + timeout
        try:
            while self.is_enabled():
                if self.color.get_junction_color() in colors:
                    return True
                if deadline is not None and time.time() > deadline:
                    return False
                time.sleep(0.005)
        finally:
            self.disable()
        return False

    # ------------------------------------------------------------------
    # Safe shutdown (called by Robot.stop via self.line.stop())
    # ------------------------------------------------------------------
    def stop(self, *args, **kwargs):
        """Ensure motors are released before the thread is joined."""
        try:
            self.disable(stop_motors=True)
        finally:
            try:
                super().stop(*args, **kwargs)
            except Exception:
                pass
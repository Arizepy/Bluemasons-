#!/usr/bin/env python3
"""Shared base class for ev3dev background worker threads.

Every thread in this project (odometry, sensors, gyro, camera, vision,
line-follow) used to hand-roll the same boilerplate: a `running` flag, a
`run()` loop checking it, a `stop()` setting it. That duplication is
exactly where bugs like a `while True` loop that never actually checks
`running` creep in (linefollow.py had this -- `stop_following()` called
`self.join()` on a thread whose loop could never exit, so it would hang
forever). This class is that pattern written once, correctly: subclasses
just implement `loop()` (one iteration of work) and optionally override
`poll_interval`.

`stop()` sets the flag AND blocks (via `join()`) until the thread has
actually exited, so callers never have to guess whether a thread is still
running before, say, cutting motor power.
"""

import threading
import time


class StoppableThread(threading.Thread):
    poll_time = 0.005

    def __init__(self, ):
        super().__init__(daemon=True)


        self.running = True
        
    
    def loop(self):
        raise NotImplementedError
    
    def run(self):
        while self.running:
            self.loop()
            time.sleep(self.poll_time)

    def stop(self):
        self.running = False
        self.join()
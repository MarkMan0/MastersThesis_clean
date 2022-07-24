import time
import math
import random


class MotorSim:

    def __init__(self) -> None:
        self.ang_speed_target = 0.0
        self.ang_speed_current = 0.0
        self.angle = 0.0
        self.accel = 000.0
        self.running = True
        self._time_const = 0.1 + random.randint(-30, 30) / 1000
        self._last_y = 0

    def stop(self) -> None:
        self.running = False

    def _apply_acceleration(self, dt):
        if self.accel == 0:
            self.ang_speed_current = self.ang_speed_target
            return

        if self.ang_speed_target > self.ang_speed_current:
            acc = self.accel
        else:
            acc = -self.accel

        increment = acc * dt
        if abs(increment) > abs(self.ang_speed_target - self.ang_speed_current):
            increment = self.ang_speed_target - self.ang_speed_current

        self.ang_speed_current += increment

    def _advance_first_order_system(self, dt):
        t = self._time_const + random.randint(-50, 50) / 1000
        A = -1.0/t
        B = 1.0/t
        C = 1

        dy = A * self._last_y + B * self.ang_speed_current
        self._last_y += dy*dt


    def tick(self, dt) -> None:
        self._apply_acceleration(dt)
        self._advance_first_order_system(dt)

        self.angle += self._last_y * dt

        if self.angle > 180:
            self.angle -= 2*180
        elif self.angle < -180:
            self.angle += 2*180

    def thread(self) -> None:
        t_last = time.perf_counter()
        while self.running:
            time.sleep(0.01)
            t_now = time.perf_counter()
            self.tick(t_now - t_last)
            t_last = t_now

    def reset(self) -> None:
        self.ang_speed_current = 0.0
        self.ang_speed_target = 0.0
        self.angle = 0.0
        self._last_y = 0.0

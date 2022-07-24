
import motorsim
import threading
import time
import math
from logging import Logger


def _get_increment(last: float, curr: float):
    diff = curr - last
    if diff < -200:
        diff += 360
    if diff > 200:
        diff -= 360
    return diff


class Robot:

    def __init__(self):
        self.wheel_radius = 0.08  # meters
        self.wheel_distance = 0.330503  # meters

        self.l_motor = motorsim.MotorSim()
        self.r_motor = motorsim.MotorSim()
        self.running = True

        self.position = {'x': 0.0,
                         'y': 0.0,
                         'theta': 0.0,
                         'l_pos': 0.0,
                         'r_pos': 0.0, }
        self.last_encoder = (self.l_motor.angle, self.r_motor.angle, )

    def create_threads(self) -> tuple:
        return (
            threading.Thread(target=self.l_motor.thread),
            threading.Thread(target=self.r_motor.thread),
            threading.Thread(target=self.thread),
        )

    def stop(self):
        self.running = False
        self.l_motor.running = False
        self.r_motor.running = False

    def tick(self, dt):
        enc_now = (self.l_motor.angle, self.r_motor.angle, )
        d_left = _get_increment(self.last_encoder[0], enc_now[0])
        self.position['l_pos'] += d_left
        d_right = _get_increment(self.last_encoder[1], enc_now[1])
        self.position['r_pos'] += d_right

        self.last_encoder = enc_now

        d_left = math.radians(d_left) * self.wheel_radius
        d_right = math.radians(d_right) * self.wheel_radius
        d_theta = (d_right - d_left) / self.wheel_distance

        dl = (d_left + d_right) / 2

        self.position['x'] += dl * math.cos(self.position['theta'])
        self.position['y'] += dl * math.sin(self.position['theta'])
        self.position['theta'] += d_theta

    def print_pos(self):
        print(
            f"x: {self.position['x']}, y: {self.position['y']}, theta: {self.position['theta']}")

    def log_line(self) -> str:
        ret = str()
        ret += str(self.position['x'])
        ret += ", " + str(self.position['y'])
        ret += ", " + str(self.position['theta'])
        ret += ", " + str(self.l_motor.angle)
        ret += ", " + str(self.l_motor.ang_speed_current)
        ret += ", " + str(self.r_motor.angle)
        ret += ", " + str(self.r_motor.ang_speed_current)
        return ret

    def thread(self):
        last = time.perf_counter()
        logger = Logger('simlog.txt')
        while self.running:
            time.sleep(0.1)
            now = time.perf_counter()
            self.tick(now - last)
            logger.log(self.log_line())
            last = now
        logger.finish()

    def reset(self):
        self.l_motor.reset()
        self.r_motor.reset()
        self.last_encoder = (self.l_motor.angle, self.r_motor.angle, )
        for k, v in self.position.items():
            self.position[k] = 0.0

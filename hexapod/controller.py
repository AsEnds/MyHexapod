import time
import threading
import importlib

from .geometry import Position3, Velocity
from .gait import (
    GaitPrg,
    N_POINTS,
    MIN_Z_PACE,
    MAX_R_PACE,
    dir_offsets,
    angle_offsets,
    legs,
    default_angles,
)


class Hexapod:
    def __init__(self):
        self.gait = GaitPrg(len(legs), default_angles, dir_offsets, angle_offsets, legs)
        self._round = 0

    def start(self):
        th = threading.Thread(target=self.move_loop, daemon=True)
        th.start()
        return th

    def move_loop(self):
        time.sleep(0.1)
        pkg = importlib.import_module('hexapod')
        while True:
            while pkg.move_flag:
                for _ in range(N_POINTS):
                    start = time.time()

                    if self.gait.velocity.omega >= 0:
                        self._round = (self._round + 1) % N_POINTS
                    else:
                        if self._round == 0:
                            self._round = N_POINTS - 1
                        else:
                            self._round -= 1

                    self.gait.CEN_and_pace_cal()
                    self.gait.gait_programing(self._round, N_POINTS, MIN_Z_PACE)

                    round_time = self.gait.get_pace_time() / N_POINTS
                    self.gait.move(round_time, self._round)
                    cost = time.time() - start
                    if cost < round_time:
                        time.sleep((round_time - cost) / 1000)
                    else:
                        time.sleep(0.001)

    # 封装旧接口
    def set_body_pos(self, x: float, y: float, z: float):
        self.gait.set_body_position(Position3(x, y, z))

    def set_velocity(self, Vx: float, Vy: float, omega: float):
        self.gait.set_velocity(Velocity(Vx, Vy, omega))

    def set_step_mode(self):
        self.gait.set_step_mode()

    def get_body_pos(self) -> Position3:
        return self.gait.get_body_pos()


# 全局实例，保持与旧版兼容的函数式接口
_robot = None

def hexapod_init():
    global _robot
    _robot = Hexapod()
    return _robot

def hexapod_move():
    if _robot is not None:
        _robot.move_loop()

def set_body_pos(x: float, y: float, z: float):
    if _robot is not None:
        _robot.set_body_pos(x, y, z)

def set_velocity(Vx: float, Vy: float, omega: float):
    if _robot is not None:
        _robot.set_velocity(Vx, Vy, omega)

def set_step_mode():
    if _robot is not None:
        _robot.set_step_mode()

def get_body_pos():
    if _robot is not None:
        return _robot.get_body_pos()
    return None

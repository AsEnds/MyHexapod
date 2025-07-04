import json
from typing import List

from conf_pkg.base_func import import_board_module
from core.geometry import Position3, Velocity
from core.kinematics import fk_solve, ik_solve, send_pulse
from core.leg import Leg
from runtime.config import HexapodConfig


class HexapodController:
    def __init__(self, config: HexapodConfig = HexapodConfig()):
        self.config = config
        self.board, _ = import_board_module()
        self.legs: List[Leg] = []
        self.default_positions: List[Position3] = []
        self.velocity = Velocity()
        self.load_leg_config()
        self.gait = None
        self.step_index = 0

    def load_leg_config(self, path: str = "leg_conf.json"):
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception:
            data = {
                "legs": [[1, 2, 3], [4, 5, 6], [7, 8, 9], [10, 11, 12], [13, 14, 15], [16, 17, 18]],
                "dir_offsets": [[1, 1, 1]] * 6,
                "angle_offsets": [[0, 0, 0]] * 6,
            }
        for servos, dirs, offs in zip(data["legs"], data["dir_offsets"], data["angle_offsets"]):
            self.legs.append(Leg(servos, dirs, offs))

        angles = [
            [self.config.PI / 4, self.config.THETA_STAND_2, self.config.THETA_STAND_3],
            [0, self.config.THETA_STAND_2, self.config.THETA_STAND_3],
            [-self.config.PI / 4, self.config.THETA_STAND_2, self.config.THETA_STAND_3],
            [3 * self.config.PI / 4, self.config.THETA_STAND_2, self.config.THETA_STAND_3],
            [self.config.PI, self.config.THETA_STAND_2, self.config.THETA_STAND_3],
            [5 * self.config.PI / 4, self.config.THETA_STAND_2, self.config.THETA_STAND_3],
        ]
        self.default_positions = [fk_solve(*ang) for ang in angles]

    def set_velocity(self, vx, vy, omega):
        self.velocity = Velocity(vx, vy, omega)

    def step(self):
        if self.gait is None:
            return
        points = self.gait.plan(self, self.step_index)
        for leg, point in zip(self.legs, points):
            angles = ik_solve(point)
            for servo_id, angle, dir_off in zip(leg.servos, angles, leg.dir_offsets):
                send_pulse(servo_id, angle, int(1000 / self.config.N_POINTS), dir_off, self.board)
        self.step_index = (self.step_index + 1) % self.config.N_POINTS

from threading import Thread
import time
from Hexapod import (
    GaitPrg,
    Position3,
    Velocity,
    default_angles,
    dir_offsets,
    angle_offsets,
    legs,
    leg_num,
    N_POINTS,
    MIN_Z_PACE,
)


class HexapodController:
    """High level controller wrapping :class:`GaitPrg`."""
    def __init__(self):
        self.gait = GaitPrg(leg_num, default_angles, dir_offsets, angle_offsets, legs)
        self.move_flag = False
        self.leg_round = 0
        self._thread = None

    def start(self):
        if self._thread is None:
            self._thread = Thread(target=self._move_loop, daemon=True)
            self._thread.start()

    def _move_loop(self):
        time.sleep(0.1)
        while True:
            while self.move_flag:
                for _ in range(N_POINTS):
                    start = time.time()
                    if self.gait.velocity.omega >= 0:
                        self.leg_round = (self.leg_round + 1) % N_POINTS
                    else:
                        if self.leg_round == 0:
                            self.leg_round = N_POINTS - 1
                        else:
                            self.leg_round -= 1
                    self.gait.CEN_and_pace_cal()
                    self.gait.gait_programing(self.leg_round, N_POINTS, MIN_Z_PACE)
                    round_time = self.gait.get_pace_time() / N_POINTS
                    self.gait.move(round_time, self.leg_round)
                    elapsed = time.time() - start
                    if elapsed < round_time:
                        time.sleep((round_time - elapsed) / 1000)
                    else:
                        time.sleep(0.001)

    # -- wrappers ----------------------------------------------------------
    def set_body_pos(self, x, y, z):
        self.gait.set_body_position(Position3(x, y, z))

    def set_velocity(self, Vx, Vy, omega):
        self.gait.set_velocity(Velocity(Vx, Vy, omega))

    def set_step_mode(self):
        self.gait.set_step_mode()

    def get_body_pos(self):
        return self.gait.get_body_pos()

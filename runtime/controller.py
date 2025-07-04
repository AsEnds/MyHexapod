"""机器人控制器，负责运动规划与舵机命令下发。"""

import json
from typing import List

from conf_pkg.base_func import import_board_module
from core.geometry import Position3, Velocity, EulerXYZ
from core.kinematics import fk_solve, ik_solve, send_pulse
from core.math_utils import rotate_vec, normalize_angle
from core.leg import Leg
from runtime.config import HexapodConfig


class HexapodController:
    """六足机器人控制逻辑"""

    def __init__(self, config: HexapodConfig = HexapodConfig()):
        # 保存配置与硬件句柄
        self.config = config
        self.board, _ = import_board_module()
        # 初始化腿部与默认位置
        self.legs: List[Leg] = []
        self.default_positions: List[Position3] = []
        self.velocity = Velocity()
        # 机体姿态与运动相关变量
        self.body_pos = Position3()
        self.body_euler = EulerXYZ()
        self.CEN = Position3()
        self.R_pace = 0.0
        self.pace_time = 0.0
        self.velocity_s = Velocity()
        self.step_mode = 0
        self.load_leg_config()
        self.gait = None
        self.step_index = 0

    def load_leg_config(self, path: str = "leg_conf.json"):
        """读取舵机配置，若失败则使用默认值"""

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
        # 计算初始位姿
        self.default_positions = [fk_solve(*ang) for ang in angles]

    def set_velocity(self, vx, vy, omega):
        """设置期望移动速度"""
        self.velocity = Velocity(vx, vy, omega)

    def set_step_mode(self):
        """切换单步模式"""
        self.step_mode = 0 if self.step_mode else 1

    def rotate_point(self, point: Position3) -> Position3:
        """按当前机体姿态旋转某个点"""

        return rotate_vec(point, self.body_euler)

    def set_body_position(self, pos: Position3):
        """设置机体相对于初始位置的偏移"""

        self.body_pos = pos
        self.default_positions = [p - self.body_pos for p in self.default_positions]

    def get_body_pos(self) -> Position3:
        """返回当前机体位置"""

        return self.body_pos

    def _update_dynamic_state(self):
        """根据当前速度计算 CEN 和摆动参数"""

        # 避免除零：速度为零时使用极小值
        vx = self.velocity.Vx or 0.001
        vy = self.velocity.Vy or 0.001
        omega = self.velocity.omega or 0.001

        # 计算瞬时 CEN 位置
        module_cen = self.config.K_CEN / omega * (vx ** 2 + vy ** 2) ** 0.5
        self.velocity_s.Vx = -vy
        self.velocity_s.Vy = vx
        if self.velocity_s.Vx >= 0:
            self.CEN.x = (module_cen ** 2 / (1 + vx ** 2 / vy ** 2)) ** 0.5
        else:
            self.CEN.x = -(module_cen ** 2 / (1 + vx ** 2 / vy ** 2)) ** 0.5
        self.CEN.y = -self.CEN.x * vx / vy

        # 根据速度求取步长与节拍时间
        module_speed = (vx ** 2 + vy ** 2 + omega ** 2) ** 0.5
        module_speed = min(module_speed, self.config.MAX_SPEED)
        self.R_pace = self.config.KR_2 * module_speed
        if self.R_pace <= self.config.MAX_R_PACE:
            self.pace_time = 1000
        else:
            self.pace_time = 1000 * self.config.MAX_R_PACE / self.R_pace
        self.R_pace = min(self.R_pace, self.config.MAX_R_PACE)

    def step(self):
        """执行一次步态计算并下发舵机角度"""
        if self.gait is None:
            return
        # 更新动态参数
        self._update_dynamic_state()
        points = self.gait.plan(self, self.step_index)
        for leg, point in zip(self.legs, points):
            angles = ik_solve(point)
            for servo_id, angle, dir_off in zip(leg.servos, angles, leg.dir_offsets):
                send_pulse(
                    servo_id,
                    normalize_angle(angle),
                    int(1000 / self.config.N_POINTS),
                    dir_off,
                    self.board,
                )
        self.step_index = (self.step_index + 1) % self.config.N_POINTS

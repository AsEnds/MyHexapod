import json
import numpy as np
from pathlib import Path

from .geometry import Position3, Thetas, Velocity
from conf_pkg.base_func import import_board_module

Board, is_debug_mode = import_board_module()

# 常量定义
PSTEP = 16
PI = np.pi
N_POINTS = 4

LEG_LEN1 = 57.0
LEG_LEN2 = 120.0
LEG_LEN3 = 186.0

CHASSIS_LEN = 255.0
CHASSIS_WIDTH = 177.0
CHASSIS_FRONT_WIDTH = 132.0

THETA_STAND_2 = 60.0 / 180.0 * PI
THETA_STAND_3 = -140 / 180.0 * PI

K_CEN = 500.0
KR_2 = 1.0

MAX_R_PACE = 150.0
MAX_SPEED = 0.5 * 660

MIN_Z_PACE = 100
MAX_Z_PACE = 200

MIN_JOINT2_RAD = -0.5 * PI
MAX_JOINT2_RAD = PI / 2.0
MIN_JOINT3_RAD = -(18.0 / 19.0) * PI
MAX_JOINT3_RAD = PI / 2

K_W = 1.0 / 56.56854

SMOOTH_BETA = 0.3  # IK 角度低通滤波系数

LegControl_round = 0

# 默认初始姿态参数
leg_num = 6
default_angles = [
    [PI / 4, THETA_STAND_2, THETA_STAND_3],
    [0, THETA_STAND_2, THETA_STAND_3],
    [-PI / 4, THETA_STAND_2, THETA_STAND_3],
    [3 * PI / 4, THETA_STAND_2, THETA_STAND_3],
    [PI, THETA_STAND_2, THETA_STAND_3],
    [5 * PI / 4, THETA_STAND_2, THETA_STAND_3],
]


# 配置加载
CONF_PATH = Path(__file__).resolve().parent.parent / 'conf_pkg' / 'leg_conf.json'
with open(CONF_PATH, 'r', encoding='utf-8') as f:
    data = json.load(f)

dir_offsets = data['dir_offsets']
legs = data['legs']
angle_offsets = data['angle_offsets']


def limits_ok(thetas, margin=0.01):
    theta1, theta2, theta3 = thetas
    if not (MIN_JOINT2_RAD + margin <= theta2 <= MAX_JOINT2_RAD - margin):
        return False
    if not (MIN_JOINT3_RAD + margin <= theta3 <= MAX_JOINT3_RAD - margin):
        return False
    return True


class GaitPrg:
    def __init__(self, leg_num, default_angles, dir_offsets, angle_offsets, legs):
        self.last_pulses = {}
        self.step_mode = 0
        self.leg_num = leg_num
        self.pace_time = 1000
        self.actions = np.zeros((N_POINTS, 6, 3))
        self.points = np.zeros((N_POINTS, 6, 3))
        self.default_angles = default_angles

        self.Pws = [self.fkine(*default_angles[i]) for i in range(6)]
        self.Pws_default = np.copy(self.Pws)

        self.P_legs = [
            Position3(CHASSIS_FRONT_WIDTH / 2, CHASSIS_LEN / 2, 0),
            Position3(CHASSIS_WIDTH / 2, 0, 0),
            Position3(CHASSIS_FRONT_WIDTH / 2, -CHASSIS_LEN / 2, 0),
            Position3(-CHASSIS_FRONT_WIDTH / 2, CHASSIS_LEN / 2, 0),
            Position3(-CHASSIS_WIDTH / 2, 0, 0),
            Position3(-CHASSIS_FRONT_WIDTH / 2, -CHASSIS_LEN / 2, 0)
        ]

        self.dir_offsets = dir_offsets
        self.angle_offsets = angle_offsets
        self.legs = legs

        self.CEN = Position3()
        self.R_pace = 0
        self.body_pos = Position3()
        self.velocity = Velocity()
        self.velocity_s = Velocity()
        self.rotate_angle = Position3()

    @staticmethod
    def fkine(angle1, angle2, angle3):
        x = np.cos(angle1) * (LEG_LEN1 + LEG_LEN3 * np.cos(angle2 + angle3) + LEG_LEN2 * np.cos(angle2))
        y = np.sin(angle1) * (LEG_LEN1 + LEG_LEN3 * np.cos(angle2 + angle3) + LEG_LEN2 * np.cos(angle2))
        z = LEG_LEN3 * np.sin(angle2 + angle3) + LEG_LEN2 * np.sin(angle2)
        return Position3(x, y, z)

    @staticmethod
    def ikine(pos, theta_prev=None):
        x, y, z = pos.x, pos.y, pos.z
        R = np.hypot(x, y)
        if R < 1e-6:
            return theta_prev if theta_prev else Thetas(0.0, THETA_STAND_2, THETA_STAND_3)
        Lr = np.hypot(z, R - LEG_LEN1)
        theta1 = np.arctan2(y, x)

        D = (LEG_LEN2**2 + LEG_LEN3**2 - Lr**2) / (2*LEG_LEN2*LEG_LEN3)
        D = np.clip(D, -1.0, 1.0)
        theta3_dn = np.arctan2(+np.sqrt(1-D**2), D) * (-1)
        theta3_up = np.arctan2(-np.sqrt(1-D**2), D) * (-1)

        def hip_pitch(t3):
            phi = np.arctan2(z, R - LEG_LEN1)
            psi = np.arctan2(LEG_LEN3*np.sin(t3), LEG_LEN2 + LEG_LEN3*np.cos(t3))
            return phi + psi

        cand = [
            (theta1, hip_pitch(theta3_dn), theta3_dn),
            (theta1, hip_pitch(theta3_up), theta3_up),
        ]

        legal = [c for c in cand if limits_ok(c)]
        if not legal:
            raise ValueError('目标点不可达')

        if theta_prev is None or len(legal) == 1:
            choice = legal[0]
        else:
            choice = min(legal, key=lambda c: np.linalg.norm(np.subtract(c, theta_prev)))

        if theta_prev is not None:
            beta = SMOOTH_BETA
            smoothed = [prev + beta * (new - prev) for new, prev in zip(choice, theta_prev.angle)]
            return Thetas(*smoothed)
        return Thetas(*choice)

    # 以下方法保持与旧实现一致，未做深入优化
    def hexapod_rotate(self, point, index):
        cos_a, sin_a = np.cos(self.rotate_angle.y), np.sin(self.rotate_angle.y)
        cos_b, sin_b = np.cos(self.rotate_angle.z), np.sin(self.rotate_angle.z)
        cos_g, sin_g = np.cos(self.rotate_angle.x), np.sin(self.rotate_angle.x)

        rotation_matrix = np.array([
            [cos_b * cos_a, cos_b * sin_a, -sin_b],
            [sin_g * sin_b * cos_a - cos_g * sin_a, sin_g * sin_b * sin_a + cos_g * cos_a, sin_g * cos_b],
            [cos_g * sin_b * cos_a + sin_g * sin_a, cos_g * sin_b * sin_a - sin_g * cos_a, cos_g * cos_b]
        ])

        new_coords = rotation_matrix.dot(point.data)
        return Position3(*new_coords)

    def set_velocity(self, velocity: Velocity):
        self.velocity = velocity

    def set_height(self, height: float):
        for i in range(6):
            self.Pws[i].z = self.Pws_default[i].z - height

    def get_body_pos(self) -> Position3:
        return self.body_pos

    def set_body_position(self, body_pos: Position3):
        self.body_pos = body_pos
        for i in range(6):
            self.Pws[i] = self.Pws_default[i] - self.body_pos

    def CEN_and_pace_cal(self):
        self.velocity.Vx = 0.001 if self.velocity.Vx == 0.0 else self.velocity.Vx
        self.velocity.Vy = 0.001 if self.velocity.Vy == 0.0 else self.velocity.Vy
        self.velocity.omega = 0.001 if self.velocity.omega == 0.0 else self.velocity.omega

        module_CEN = K_CEN / self.velocity.omega * np.sqrt(self.velocity.Vx ** 2 + self.velocity.Vy ** 2)

        self.velocity_s.Vx = -self.velocity.Vy
        self.velocity_s.Vy = self.velocity.Vx

        if self.velocity_s.Vx >= 0:
            self.CEN.x = np.sqrt((module_CEN ** 2) / (1 + self.velocity.Vx ** 2 / self.velocity.Vy ** 2))
        else:
            self.CEN.x = -np.sqrt((module_CEN ** 2) / (1 + self.velocity.Vx ** 2 / self.velocity.Vy ** 2))

        module_speed = (self.velocity.Vx ** 2 + self.velocity.Vy ** 2 + self.velocity.omega ** 2) ** 0.5
        module_speed = module_speed if module_speed < MAX_SPEED else MAX_SPEED
        self.R_pace = KR_2 * module_speed
        self.pace_time = 1000 if self.R_pace <= MAX_R_PACE else 1000 * MAX_R_PACE / self.R_pace
        self.R_pace = min(self.R_pace, MAX_R_PACE)

        self.CEN.y = -self.CEN.x * self.velocity.Vx / self.velocity.Vy
        self.CEN.y = -self.CEN.x * self.velocity.Vx / self.velocity.Vy

    def gait6_programing(self, LegControl_round, N_POINTS, MIN_Z_PACE):
        Vec_CEN2leg_ends = [Position3() for _ in range(6)]
        angle_off = [0.0] * 6
        norm_CEN2legs = [0.0] * 6
        Rp_ratios = [0.0] * 6
        Vec_Leg_Start2CEN_s = [Position3() for _ in range(6)]

        for i in range(6):
            Vec_CEN2leg_ends[i] = self.Pws[i] + self.P_legs[i] - self.CEN
            angle_off[i] = np.arctan2(Vec_CEN2leg_ends[i].y, Vec_CEN2leg_ends[i].x)
            norm_CEN2legs[i] = np.sqrt(Vec_CEN2leg_ends[i].x ** 2 + Vec_CEN2leg_ends[i].y ** 2)
            Vec_Leg_Start2CEN_s[i] = self.CEN - self.P_legs[i]

        max_norm_CEN2legs = max(norm_CEN2legs)

        R_paces = [0.0] * 6
        for i in range(6):
            Rp_ratios[i] = norm_CEN2legs[i] / max_norm_CEN2legs
            R_paces[i] = Rp_ratios[i] * self.R_pace

        d_theta = 2 * R_paces[0] / norm_CEN2legs[0]
        step_size = d_theta / (N_POINTS / 2)

        for i in range(0, 6, 2):
            if LegControl_round < N_POINTS / 2:
                angle_t = angle_off[i] + d_theta / 2 - step_size * LegControl_round
                point_x = Vec_Leg_Start2CEN_s[i].x + norm_CEN2legs[i] * np.cos(angle_t)
                point_y = Vec_Leg_Start2CEN_s[i].y + norm_CEN2legs[i] * np.sin(angle_t)
                point_z = self.Pws[i].z
            else:
                angle_t = angle_off[i] - d_theta / 2 + step_size * (LegControl_round - N_POINTS / 2)
                point_x = Vec_Leg_Start2CEN_s[i].x + norm_CEN2legs[i] * np.cos(angle_t)
                point_y = Vec_Leg_Start2CEN_s[i].y + norm_CEN2legs[i] * np.sin(angle_t)

                y_temp = -self.R_pace + (LegControl_round - N_POINTS / 2) * (self.R_pace * 4 / N_POINTS)
                if (self.step_mode == 0):
                    if 0.5 < self.R_pace < MIN_Z_PACE:
                        point_z = np.sqrt(self.R_pace ** 2 - y_temp ** 2) * Rp_ratios[i] * 3 + self.Pws[i].z
                    else:
                        point_z = np.sqrt(self.R_pace ** 2 - y_temp ** 2) * Rp_ratios[i] + self.Pws[i].z
                else:
                    if 0.5 < self.R_pace < MIN_Z_PACE:
                        point_z = np.sqrt(self.R_pace ** 2 - y_temp ** 2) * Rp_ratios[i] * 3 * PSTEP + self.Pws[i].z
                    else:
                        point_z = np.sqrt(self.R_pace ** 2 - y_temp ** 2) * Rp_ratios[i] * PSTEP + self.Pws[i].z




            point = Position3(point_x, point_y, point_z)
            rotated_point = self.hexapod_rotate(point, i)
            self.points[LegControl_round, i] = rotated_point.data + self.P_legs[i].data
            self.actions[LegControl_round, i] = self.ikine(rotated_point).angle

        for i in range(1, 6, 2):
            if LegControl_round < N_POINTS / 2:
                angle_t = angle_off[i] - d_theta / 2 + step_size * LegControl_round
                point_x = Vec_Leg_Start2CEN_s[i].x + norm_CEN2legs[i] * np.cos(angle_t)
                point_y = Vec_Leg_Start2CEN_s[i].y + norm_CEN2legs[i] * np.sin(angle_t)

                y_temp = -self.R_pace + LegControl_round * (self.R_pace * 4 / N_POINTS)
                if (self.step_mode == 0):
                    if 0.5 < self.R_pace < MIN_Z_PACE:
                        point_z = np.sqrt(self.R_pace ** 2 - y_temp ** 2) * Rp_ratios[i] * 3 + self.Pws[i].z
                    else:
                        point_z = np.sqrt(self.R_pace ** 2 - y_temp ** 2) * Rp_ratios[i] + self.Pws[i].z
                else:
                    if 0.5 < self.R_pace < MIN_Z_PACE:
                        point_z = np.sqrt(self.R_pace ** 2 - y_temp ** 2) * Rp_ratios[i] * 3 * PSTEP + self.Pws[i].z
                    else:
                        point_z = np.sqrt(self.R_pace ** 2 - y_temp ** 2) * Rp_ratios[i] * PSTEP + self.Pws[i].z
            else:
                angle_t = angle_off[i] + d_theta / 2 - step_size * (LegControl_round - N_POINTS / 2)
                point_x = Vec_Leg_Start2CEN_s[i].x + norm_CEN2legs[i] * np.cos(angle_t)
                point_y = Vec_Leg_Start2CEN_s[i].y + norm_CEN2legs[i] * np.sin(angle_t)
                point_z = self.Pws[i].z


            point = Position3(point_x, point_y, point_z)
            rotated_point = self.hexapod_rotate(point, i)
            self.points[LegControl_round, i] = rotated_point.data + self.P_legs[i].data
            self.actions[LegControl_round, i] = self.ikine(rotated_point).angle

    def gait4_programing(self, LegControl_round, N_POINTS, MIN_Z_PACE):
        Vec_CEN2leg_ends = [Position3() for _ in range(6)]
        angle_off = [0.0] * 6
        norm_CEN2legs = [0.0] * 6
        Rp_ratios = [0.0] * 6
        Vec_Leg_Start2CEN_s = [Position3() for _ in range(6)]

        for i in range(6):
            Vec_CEN2leg_ends[i] = self.Pws[i] + self.P_legs[i] - self.CEN
            angle_off[i] = np.arctan2(Vec_CEN2leg_ends[i].y, Vec_CEN2leg_ends[i].x)
            norm_CEN2legs[i] = np.sqrt(Vec_CEN2leg_ends[i].x ** 2 + Vec_CEN2leg_ends[i].y ** 2)
            Vec_Leg_Start2CEN_s[i] = self.CEN - self.P_legs[i]

        max_norm_CEN2legs = max(norm_CEN2legs)

        R_paces = [0.0] * 6
        for i in range(6):
            Rp_ratios[i] = norm_CEN2legs[i] / max_norm_CEN2legs
            R_paces[i] = Rp_ratios[i] * self.R_pace

        d_theta = 2 * R_paces[0] / norm_CEN2legs[0]
        step_size = d_theta / (N_POINTS / 2)
        for i in [0, 5]:
            if LegControl_round < N_POINTS / 2:
                angle_t = angle_off[i] + d_theta / 2 - step_size * LegControl_round
                point_x = Vec_Leg_Start2CEN_s[i].x + norm_CEN2legs[i] * np.cos(angle_t)
                point_y = Vec_Leg_Start2CEN_s[i].y + norm_CEN2legs[i] * np.sin(angle_t)
                point_z = self.Pws[i].z
            else:
                angle_t = angle_off[i] - d_theta / 2 + step_size * (LegControl_round - N_POINTS / 2)
                point_x = Vec_Leg_Start2CEN_s[i].x + norm_CEN2legs[i] * np.cos(angle_t)
                point_y = Vec_Leg_Start2CEN_s[i].y + norm_CEN2legs[i] * np.sin(angle_t)

                y_temp = -self.R_pace + (LegControl_round - N_POINTS / 2) * (self.R_pace * 4 / N_POINTS)
                if (self.step_mode == 0):
                    if 0.5 < self.R_pace < MIN_Z_PACE:
                        point_z = np.sqrt(self.R_pace ** 2 - y_temp ** 2) * Rp_ratios[i] * 3 + self.Pws[i].z
                    else:
                        point_z = np.sqrt(self.R_pace ** 2 - y_temp ** 2) * Rp_ratios[i] + self.Pws[i].z
                else:
                    if 0.5 < self.R_pace < MIN_Z_PACE:
                        point_z = np.sqrt(self.R_pace ** 2 - y_temp ** 2) * Rp_ratios[i] * 3 * PSTEP + self.Pws[i].z
                    else:
                        point_z = np.sqrt(self.R_pace ** 2 - y_temp ** 2) * Rp_ratios[i] * PSTEP + self.Pws[i].z


            point = Position3(point_x, point_y, point_z)
            rotated_point = self.hexapod_rotate(point, i)
            self.points[LegControl_round, i] = rotated_point.data + self.P_legs[i].data
            self.actions[LegControl_round, i] = self.ikine(rotated_point).angle

        for i in [2, 3]:
            if LegControl_round < N_POINTS / 2:
                angle_t = angle_off[i] - d_theta / 2 + step_size * LegControl_round
                point_x = Vec_Leg_Start2CEN_s[i].x + norm_CEN2legs[i] * np.cos(angle_t)
                point_y = Vec_Leg_Start2CEN_s[i].y + norm_CEN2legs[i] * np.sin(angle_t)

                y_temp = -self.R_pace + LegControl_round * (self.R_pace * 4 / N_POINTS)
                if (self.step_mode == 0):
                    if 0.5 < self.R_pace < MIN_Z_PACE:
                        point_z = np.sqrt(self.R_pace ** 2 - y_temp ** 2) * Rp_ratios[i] * 3 + self.Pws[i].z
                    else:
                        point_z = np.sqrt(self.R_pace ** 2 - y_temp ** 2) * Rp_ratios[i] + self.Pws[i].z
                else:
                    if 0.5 < self.R_pace < MIN_Z_PACE:
                        point_z = np.sqrt(self.R_pace ** 2 - y_temp ** 2) * Rp_ratios[i] * 3 * PSTEP + self.Pws[i].z
                    else:
                        point_z = np.sqrt(self.R_pace ** 2 - y_temp ** 2) * Rp_ratios[i] * PSTEP + self.Pws[i].z
            else:
                angle_t = angle_off[i] + d_theta / 2 - step_size * (LegControl_round - N_POINTS / 2)
                point_x = Vec_Leg_Start2CEN_s[i].x + norm_CEN2legs[i] * np.cos(angle_t)
                point_y = Vec_Leg_Start2CEN_s[i].y + norm_CEN2legs[i] * np.sin(angle_t)
                point_z = self.Pws[i].z

            point = Position3(point_x, point_y, point_z)
            rotated_point = self.hexapod_rotate(point, i)
            self.points[LegControl_round, i] = rotated_point.data + self.P_legs[i].data
            self.actions[LegControl_round, i] = self.ikine(rotated_point).angle

    def gait_programing(self, LegControl_round, N_POINTS, MIN_Z_PACE):
        if self.leg_num == 6:
            self.gait6_programing(LegControl_round, N_POINTS, MIN_Z_PACE)
        elif self.leg_num == 4:
            self.gait4_programing(LegControl_round, N_POINTS, MIN_Z_PACE)
        else:
            print("leg_num error!")

    def get_pace_time(self):
        return self.pace_time

    # def setJointAngle(self, dirOffset, legIndex, angle, time):
    #     pulse = 500 + angle / PI * 750 if dirOffset == 1 else 500 - angle / PI * 750
    #     Board.setBusServoPulse(legIndex, int(pulse), int(time))

    def set_leg(pws: Position3):
        pass

    def setJointAngle(self, dirOffset, legIndex, angle, time):
        pulse = 500 + angle / PI * 750 if dirOffset == 1 else 500 - angle / PI * 750
        pulse = int(pulse)

        # 若与上一次脉冲不变，则跳过
        if self.last_pulses.get(legIndex) != pulse:
            Board.setBusServoPulse(legIndex, pulse, int(time))
            self.last_pulses[legIndex] = pulse
        else:
            pass

    def move(self, round_time, LegControl_round):
        if self.leg_num == 6:
            for leg in range(6):
                theta_temp = self.actions[LegControl_round, leg] + self.angle_offsets[leg]
                if theta_temp[0] < -2 / 3 * PI:
                    theta_temp[0] += 2 * PI
                for joint in range(3):
                    self.setJointAngle(self.dir_offsets[leg][joint], self.legs[leg][joint], theta_temp[joint],
                                       round_time)
        elif self.leg_num == 4:
            for leg in [0, 2, 3, 5]:
                theta_temp = self.actions[LegControl_round, leg] + self.angle_offsets[leg]
                if theta_temp[0] < -2 / 3 * PI:
                    theta_temp[0] += 2 * PI
                for joint in range(3):
                    self.setJointAngle(self.dir_offsets[leg][joint], self.legs[leg][joint], theta_temp[joint],
                                       round_time)
        else:
            print("leg_num error!")
    
    def set_step_mode(self):
        if self.step_mode == 0:
            print("Switched to manual mode")
            self.step_mode = 1
        else:
            print("Switched to auto mode")
            self.step_mode = 0

"""机器人全局配置常量。"""

from dataclasses import dataclass
import numpy as np

@dataclass
class HexapodConfig:
    """所有与机器人结构和运动相关的常量"""

    PSTEP: int = 16  # 单步划分份数
    PI: float = np.pi
    N_POINTS: int = 4  # 一个步态周期内的离散点数

    LEG_LEN1: float = 57.0  # 大腿长度
    LEG_LEN2: float = 120.0  # 小腿长度
    LEG_LEN3: float = 186.0  # 足端长度

    CHASSIS_LEN: float = 255.0  # 机身长度
    CHASSIS_WIDTH: float = 177.0  # 机身宽度
    CHASSIS_FRONT_WIDTH: float = 132.0  # 前宽

    THETA_STAND_2: float = 60.0 / 180.0 * np.pi  # 默认第二关节角
    THETA_STAND_3: float = -140 / 180.0 * np.pi  # 默认第三关节角

    K_CEN: float = 500.0  # CEN 计算系数
    KR_2: float = 1.0

    MAX_R_PACE: float = 150.0  # 最大摆腿半径
    MAX_SPEED: float = 0.5 * 660  # 最大速度

    MIN_Z_PACE: float = 100  # 最低抬升高度
    MAX_Z_PACE: float = 200  # 最高抬升高度

    MIN_JOINT2_RAD: float = -0.5 * np.pi
    MAX_JOINT2_RAD: float = np.pi / 2.0
    MIN_JOINT3_RAD: float = -(18.0 / 19.0) * np.pi
    MAX_JOINT3_RAD: float = np.pi / 2

    K_W: float = 1.0 / 56.56854
    SMOOTH_BETA: float = 0.3  # IK 平滑因子

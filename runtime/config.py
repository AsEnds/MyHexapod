from dataclasses import dataclass
import numpy as np

@dataclass
class HexapodConfig:
    PSTEP: int = 16
    PI: float = np.pi
    N_POINTS: int = 4

    LEG_LEN1: float = 57.0
    LEG_LEN2: float = 120.0
    LEG_LEN3: float = 186.0

    CHASSIS_LEN: float = 255.0
    CHASSIS_WIDTH: float = 177.0
    CHASSIS_FRONT_WIDTH: float = 132.0

    THETA_STAND_2: float = 60.0 / 180.0 * np.pi
    THETA_STAND_3: float = -140 / 180.0 * np.pi

    K_CEN: float = 500.0
    KR_2: float = 1.0

    MAX_R_PACE: float = 150.0
    MAX_SPEED: float = 0.5 * 660

    MIN_Z_PACE: float = 100
    MAX_Z_PACE: float = 200

    MIN_JOINT2_RAD: float = -0.5 * np.pi
    MAX_JOINT2_RAD: float = np.pi / 2.0
    MIN_JOINT3_RAD: float = -(18.0 / 19.0) * np.pi
    MAX_JOINT3_RAD: float = np.pi / 2

    K_W: float = 1.0 / 56.56854
    SMOOTH_BETA: float = 0.3

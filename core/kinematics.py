"""正逆运动学与舵机脉冲相关工具函数。"""

import numpy as np
from .geometry import Position3
from runtime.config import HexapodConfig


config = HexapodConfig()


def fk_solve(angle1: float, angle2: float, angle3: float) -> Position3:
    """根据关节角求取末端位置"""

    x = np.cos(angle1) * (
        config.LEG_LEN1
        + config.LEG_LEN3 * np.cos(angle2 + angle3)
        + config.LEG_LEN2 * np.cos(angle2)
    )
    y = np.sin(angle1) * (
        config.LEG_LEN1
        + config.LEG_LEN3 * np.cos(angle2 + angle3)
        + config.LEG_LEN2 * np.cos(angle2)
    )
    z = config.LEG_LEN3 * np.sin(angle2 + angle3) + config.LEG_LEN2 * np.sin(angle2)
    return Position3(x, y, z)


def limits_ok(thetas, margin=0.01):
    """检查关节角是否在允许范围内"""

    _, theta2, theta3 = thetas
    if not (
        config.MIN_JOINT2_RAD + margin
        <= theta2
        <= config.MAX_JOINT2_RAD - margin
    ):
        return False
    if not (
        config.MIN_JOINT3_RAD + margin
        <= theta3
        <= config.MAX_JOINT3_RAD - margin
    ):
        return False
    return True


class IKContext:
    """保存逆解历史以实现角度平滑"""

    def __init__(self):
        self.prev_thetas = None


ctx = IKContext()


def ik_solve(pos: Position3):
    """根据末端坐标求取三个关节角"""

    x, y, z = pos.x, pos.y, pos.z
    R = np.hypot(x, y)
    if R < 1e-6:
        if ctx.prev_thetas:
            return ctx.prev_thetas
        return 0.0, config.THETA_STAND_2, config.THETA_STAND_3
    Lr = np.hypot(z, R - config.LEG_LEN1)
    theta1 = np.arctan2(y, x)

    D = (config.LEG_LEN2**2 + config.LEG_LEN3**2 - Lr**2) / (2*config.LEG_LEN2*config.LEG_LEN3)
    D = np.clip(D, -1.0, 1.0)
    theta3_dn = np.arctan2(+np.sqrt(1-D**2), D) * (-1)
    theta3_up = np.arctan2(-np.sqrt(1-D**2), D) * (-1)

    def hip_pitch(t3):
        phi = np.arctan2(z, R - config.LEG_LEN1)
        psi = np.arctan2(config.LEG_LEN3*np.sin(t3), config.LEG_LEN2 + config.LEG_LEN3*np.cos(t3))
        return phi + psi

    cand = [
        (theta1, hip_pitch(theta3_dn), theta3_dn),
        (theta1, hip_pitch(theta3_up), theta3_up),
    ]
    legal = [c for c in cand if limits_ok(c)]
    if not legal:
        raise ValueError("target out of range")
    if ctx.prev_thetas is None or len(legal) == 1:
        choice = legal[0]
    else:
        choice = min(legal, key=lambda c: np.linalg.norm(np.subtract(c, ctx.prev_thetas)))

    new_theta = np.array(choice)
    if ctx.prev_thetas is not None:
        beta = config.SMOOTH_BETA
        new_theta = ctx.prev_thetas + beta * (new_theta - ctx.prev_thetas)
    ctx.prev_thetas = new_theta
    return tuple(new_theta)


def send_pulse(servo_id: int, angle: float, time_ms: int, dir_offset: int, board):
    """向舵机板下发脉冲"""

    pulse = (
        500 + angle / config.PI * 750
        if dir_offset == 1
        else 500 - angle / config.PI * 750
    )
    board.setBusServoPulse(servo_id, int(pulse), int(time_ms))

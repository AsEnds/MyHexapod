"""通用数学工具函数。"""

import numpy as np
from .geometry import Position3, EulerXYZ


def rotate_vec(point: Position3, euler: EulerXYZ) -> Position3:
    """根据给定的欧拉角(XYZ顺序)旋转向量"""
    cos_a, sin_a = np.cos(euler.pitch), np.sin(euler.pitch)
    cos_b, sin_b = np.cos(euler.yaw), np.sin(euler.yaw)
    cos_g, sin_g = np.cos(euler.roll), np.sin(euler.roll)

    rotation_matrix = np.array([
        [cos_b * cos_a, cos_b * sin_a, -sin_b],
        [
            sin_g * sin_b * cos_a - cos_g * sin_a,
            sin_g * sin_b * sin_a + cos_g * cos_a,
            sin_g * cos_b,
        ],
        [
            cos_g * sin_b * cos_a + sin_g * sin_a,
            cos_g * sin_b * sin_a - sin_g * cos_a,
            cos_g * cos_b,
        ],
    ])

    new_coords = rotation_matrix.dot(point.data)
    # 重新构造 Position3 返回
    return Position3(*new_coords)


def normalize_angle(a: float) -> float:
    """将角度归一化到 ``[-pi, pi]`` 区间"""

    return (a + np.pi) % (2 * np.pi) - np.pi

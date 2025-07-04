import numpy as np
from .geometry import Position3, EulerXYZ


def rotate_vec(point: Position3, euler: EulerXYZ) -> Position3:
    """Rotate a vector by Euler angles (XYZ order)."""
    cos_a, sin_a = np.cos(euler.pitch), np.sin(euler.pitch)
    cos_b, sin_b = np.cos(euler.yaw), np.sin(euler.yaw)
    cos_g, sin_g = np.cos(euler.roll), np.sin(euler.roll)

    rotation_matrix = np.array([
        [cos_b * cos_a, cos_b * sin_a, -sin_b],
        [sin_g * sin_b * cos_a - cos_g * sin_a, sin_g * sin_b * sin_a + cos_g * cos_a, sin_g * cos_b],
        [cos_g * sin_b * cos_a + sin_g * sin_a, cos_g * sin_b * sin_a - sin_g * cos_a, cos_g * cos_b],
    ])

    new_coords = rotation_matrix.dot(point.data)
    return Position3(*new_coords)

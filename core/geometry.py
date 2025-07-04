"""基础几何相关的数据结构。"""

from dataclasses import dataclass
import numpy as np

@dataclass
class Position3:
    """三维坐标点"""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    @property
    def data(self):
        """以 ``numpy`` 数组的形式返回坐标"""
        return np.array([self.x, self.y, self.z], dtype=float)

    def __add__(self, other: "Position3"):
        """向量加法"""
        return Position3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "Position3"):
        """向量减法"""
        return Position3(self.x - other.x, self.y - other.y, self.z - other.z)

    def magnitude(self) -> float:
        """求向量长度"""
        return np.linalg.norm(self.data)


@dataclass
class Velocity:
    """线速度和角速度"""

    Vx: float = 0.0
    Vy: float = 0.0
    omega: float = 0.0


@dataclass
class EulerXYZ:
    """XYZ 欧拉角表示"""

    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

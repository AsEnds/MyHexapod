"""腿部舵机配置及辅助函数。"""

from dataclasses import dataclass
from typing import List


@dataclass
class Leg:
    """单条腿包含的舵机信息"""

    servos: List[int]
    dir_offsets: List[int]
    angle_offsets: List[float]

    def target_angles(self, raw_angles):
        """根据方向与偏移量计算真实舵机角度"""
        return [a + off for a, off in zip(raw_angles, self.angle_offsets)]

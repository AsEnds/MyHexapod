"""二足交替步态规划。"""

from typing import List
from core.geometry import Position3


class BipodGait:
    """二足交替步态实现"""

    def __init__(self):
        self.points: List[List[Position3]] = []

    def plan(self, controller, step_idx: int):
        """返回当前步的目标位置"""
        if not self.points:
            self.points = [
                [p for p in controller.default_positions]
                for _ in range(controller.config.N_POINTS)
            ]
        return self.points[step_idx]

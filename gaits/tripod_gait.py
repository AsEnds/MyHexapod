"""三角形步态规划。"""

from typing import List
from core.geometry import Position3


class TripodGait:
    """三角形步态实现"""

    def __init__(self):
        # 预生成的六条腿轨迹点
        self.points: List[List[Position3]] = []

    def plan(self, controller, step_idx: int):
        """计算当前步的六足目标位置"""

        # 目前仅返回静止姿态
        if not self.points:
            self.points = [
                [p for p in controller.default_positions]
                for _ in range(controller.config.N_POINTS)
            ]
        return self.points[step_idx]

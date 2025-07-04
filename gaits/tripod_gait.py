from typing import List
from core.geometry import Position3


class TripodGait:
    def __init__(self):
        self.points: List[List[Position3]] = []

    def plan(self, controller, step_idx: int):
        """Compute foot target positions for this step."""
        # Placeholder implementation: keep feet at default positions
        if not self.points:
            self.points = [[p for p in controller.default_positions] for _ in range(controller.config.N_POINTS)]
        return self.points[step_idx]

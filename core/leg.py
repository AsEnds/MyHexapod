from dataclasses import dataclass
from typing import List


@dataclass
class Leg:
    servos: List[int]
    dir_offsets: List[int]
    angle_offsets: List[float]

    def target_angles(self, raw_angles):
        """Apply direction and offset to raw servo angles."""
        return [a + off for a, off in zip(raw_angles, self.angle_offsets)]

from dataclasses import dataclass
import numpy as np

@dataclass
class Position3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    @property
    def data(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z], dtype=float)

    def magnitude(self) -> float:
        return float(np.linalg.norm(self.data))

    def __sub__(self, other: "Position3") -> "Position3":
        return Position3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __add__(self, other: "Position3") -> "Position3":
        return Position3(self.x + other.x, self.y + other.y, self.z + other.z)


@dataclass
class Thetas:
    angle1: float = 0.0
    angle2: float = 0.0
    angle3: float = 0.0

    @property
    def angle(self):
        return [self.angle1, self.angle2, self.angle3]


@dataclass
class Velocity:
    Vx: float = 0.0
    Vy: float = 0.0
    omega: float = 0.0

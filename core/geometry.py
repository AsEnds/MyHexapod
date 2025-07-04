from dataclasses import dataclass
import numpy as np

@dataclass
class Position3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    @property
    def data(self):
        return np.array([self.x, self.y, self.z], dtype=float)

    def __add__(self, other: "Position3"):
        return Position3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "Position3"):
        return Position3(self.x - other.x, self.y - other.y, self.z - other.z)

    def magnitude(self) -> float:
        return np.linalg.norm(self.data)


@dataclass
class Velocity:
    Vx: float = 0.0
    Vy: float = 0.0
    omega: float = 0.0


@dataclass
class EulerXYZ:
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

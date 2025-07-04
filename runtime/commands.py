from dataclasses import dataclass
from enum import Enum, auto


class CommandType(Enum):
    SET_VELOCITY = auto()
    SET_BODY_POS = auto()
    SWITCH_GAIT = auto()
    STEP_MODE = auto()


@dataclass
class Command:
    type: CommandType
    args: tuple

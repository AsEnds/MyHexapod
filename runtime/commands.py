"""线程间传递的命令定义。"""

from dataclasses import dataclass
from enum import Enum, auto


class CommandType(Enum):
    """可下发的命令类型"""

    SET_VELOCITY = auto()  # 设置速度
    SET_BODY_POS = auto()  # 调整机体位置
    SWITCH_GAIT = auto()  # 切换步态
    STEP_MODE = auto()  # 单步模式


@dataclass
class Command:
    """具体命令数据结构"""

    type: CommandType
    args: tuple

move_flag = False

from .controller import (
    Hexapod,
    hexapod_init,
    hexapod_move,
    set_body_pos,
    set_velocity,
    set_step_mode,
    get_body_pos,
)
from .gait import MAX_R_PACE

__all__ = [
    'Hexapod',
    'hexapod_init',
    'hexapod_move',
    'set_body_pos',
    'set_velocity',
    'set_step_mode',
    'get_body_pos',
    'move_flag',
    'MAX_R_PACE',
]

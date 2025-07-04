"""\
摇杆事件到控制器动作的映射。
"""

from core.geometry import Position3
from runtime.controller import HexapodController


class CommandHandler:
    """根据摇杆输入调用 ``HexapodController``"""

    def __init__(self, controller: HexapodController):
        self.ctrl = controller

    # 按钮按下回调 -------------------------------------------------------------
    def set_PSB_SELECT_button(self):
        """SELECT 按钮"""
        pass

    def set_PSB_R1_button(self):
        """R1 按钮: 机体上抬"""
        self.ctrl.set_body_position(Position3(0, 0, 100))

    def set_PSB_R2_button(self):
        """R2 按钮: 恢复高度"""
        self.ctrl.set_body_position(Position3(0, 0, 20))

    def set_PSB_L1_button(self):
        """L1 按钮: 预留"""
        pass

    def set_PSB_L2_button(self):
        """L2 按钮: 停止移动"""
        self.ctrl.set_velocity(0, 0, 0)

    def set_PSB_SQUARE_button(self):
        """方形按钮: 原地顺时针旋转"""
        self.ctrl.config.MAX_R_PACE = 60
        self.ctrl.set_velocity(0, 0, 200)

    def set_PSB_CIRCLE_button(self):
        """圆形按钮: 原地逆时针旋转"""
        self.ctrl.config.MAX_R_PACE = 60
        self.ctrl.set_velocity(0, 0, -200)

    def set_PSB_TRIANGLE_button(self):
        """三角按钮: 切换单步模式"""
        self.ctrl.set_step_mode()

    def set_PSB_CROSS_button(self):
        """X 按钮: 前移并前进"""
        self.ctrl.set_body_position(Position3(0, 100, 50))
        self.ctrl.set_velocity(200, 0, 0)

    def set_PSB_START_button(self):
        """START 按钮"""
        pass

    def set_LEFT_STICK_button(self, lx1, ly1):
        """左摇杆控制平移速度"""
        vx = lx1 * self.ctrl.config.MAX_SPEED
        vy = -ly1 * self.ctrl.config.MAX_SPEED
        self.ctrl.set_velocity(vx, vy, 0)

    def set_RIGHT_STICK_button(self, lx2, ly2):
        """右摇杆控制角速度"""
        w = lx2 * self.ctrl.config.MAX_SPEED
        self.ctrl.set_velocity(0, 0, w)

    # 十字键 ---------------------------------------------------------------
    def set_Right_button(self):
        self.ctrl.config.MAX_R_PACE = 60
        self.ctrl.set_velocity(0, -200, 0)

    def set_Left_button(self):
        self.ctrl.config.MAX_R_PACE = 60
        self.ctrl.set_velocity(0, 200, 0)

    def set_Up_button(self):
        self.ctrl.set_velocity(200, 0, 0)

    def set_Down_button(self):
        self.ctrl.set_velocity(-200, 0, 0)

    # 按钮松开 --------------------------------------------------------------
    def button_release(self):
        self.ctrl.config.MAX_R_PACE = 80
        self.ctrl.set_velocity(0, 0, 0)

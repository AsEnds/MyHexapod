# pod2.py
"""Mapping from joystick events to hexapod actions."""

import Hexapod as pd
from controller import HexapodController
import ActionGroupControl as AGC
import time

PI = pd.PI


class CommandHandler:
    """Translate joystick callbacks into :class:`HexapodController` actions."""

    def __init__(self, controller: HexapodController):
        self.ctrl = controller


# ---------------------------------------------------------------------------
# Command handler implementation
# ---------------------------------------------------------------------------

# Example mapping from button name to method name.

# 按钮按下时执行的函数
    
    def set_PSB_SELECT_button(self):
        """SELECT 按钮"""
        pass

    def set_PSB_R1_button(self):  # R1 按钮
        self.ctrl.set_body_pos(0, 0, 100)

    def set_PSB_R2_button(self):  # R2 按钮
        self.ctrl.set_body_pos(0, 0, 20)

    def set_PSB_L1_button(self):  # L1 按钮
        #     th = threading.Thread(target=AGC.runActionGroup, args=('7.7taijie', 0), daemon=True)  # 运行动作函数是阻塞式的，如果要循环运行一段时间后停止，请用线程来开启
#     th.start()
#     time.sleep(10)
#     AGC.stopAction()  # 3秒后发出停止指令
#     while th.is_alive(): # 等待动作完全停止
#         time.sleep(0.01)
# 
# #     AGC.runActionGroup('7.7taijie')  # 参数为动作组的名称，不包含后缀，以字符形式传入
#
        pass

    def set_PSB_L2_button(self):  # L2 按钮
        self.ctrl.set_velocity(0, 0, 0)

    def set_PSB_SQUARE_button(self):  # 正方形按钮
        pd.MAX_R_PACE = 60
        self.ctrl.set_velocity(0, 0, 200)

    def set_PSB_CIRCLE_button(self):  # 圆按钮
        pd.MAX_R_PACE = 60
        self.ctrl.set_velocity(0, 0, -200)

    def set_PSB_TRIANGLE_button(self):  # 三角形按钮
        self.ctrl.set_step_mode()

    def set_PSB_CROSS_button(self):  # X 按钮
        # 机器人身体前移100，以100速度朝前
        self.ctrl.set_body_pos(0, 100, 50)
        self.ctrl.set_velocity(200, 0, 0)

    def set_PSB_START_button(self):
        pass

    def set_LEFT_STICK_button(self, lx1, ly1):
        """左摇杆控制速度"""
        vx = lx1 * pd.MAX_SPEED
        vy = -ly1 * pd.MAX_SPEED
        self.ctrl.set_velocity(vx, vy, 0)

    def set_RIGHT_STICK_button(self, lx2, ly2):
        w = lx2 * pd.MAX_SPEED
        self.ctrl.set_velocity(0, 0, w)
    
#十字键
    # 十字键控制
    def set_Right_button(self):
        pd.MAX_R_PACE = 60
        self.ctrl.set_velocity(0, -200, 0)

    def set_Left_button(self):
        pd.MAX_R_PACE = 60
        self.ctrl.set_velocity(0, 200, 0)

    def set_Up_button(self):
        self.ctrl.set_velocity(200, 0, 0)

    def set_Down_button(self):
        self.ctrl.set_velocity(-200, 0, 0)


# 按钮松开时执行的函数
    def button_release(self):
        pd.MAX_R_PACE = 80
        self.ctrl.set_velocity(0, 0, 0)




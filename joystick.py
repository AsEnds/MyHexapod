"""\
读取手柄并将事件交给 ``CommandHandler`` 处理。
"""

import os
import time
import threading
import pygame

from pod2 import CommandHandler

# 按钮名称到索引的映射
key_map = {
    "PSB_CROSS": 0,
    "PSB_CIRCLE": 1,
    "PSB_SQUARE": 3,
    "PSB_TRIANGLE": 4,
    "PSB_L1": 6,
    "PSB_R1": 7,
    "PSB_L2": 8,
    "PSB_R2": 9,
    "PSB_SELECT": 10,
    "PSB_START": 11,
}

last_input_time = time.time()
TIMEOUT = 2


def update_last_input_time():
    """更新最近输入时间"""
    global last_input_time
    last_input_time = time.time()


def check_timeout(handler: CommandHandler):
    """若长时间无输入则停止机器人"""
    global last_input_time
    while True:
        if time.time() - last_input_time > TIMEOUT:
            handler.ctrl.set_velocity(0, 0, 0)
        time.sleep(1)


def joystick_init():
    """初始化 Pygame 与摇杆"""
    os.environ["SDL_VIDEODRIVER"] = "dummy"
    pygame.display.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() > 0:
        js = pygame.joystick.Joystick(0)
        js.init()
        print("Joystick:", js.get_name())
        print("Axes:", js.get_numaxes())
        print("Buttons:", js.get_numbuttons())
        print("Balls:", js.get_numballs())
        print("Hats:", js.get_numhats())


button_states = {key: False for key in key_map.keys()}
hat_flag = False


def get_joystick_data(handler: CommandHandler):
    """循环读取摇杆数据"""
    global hat_flag
    connected = False
    threading.Thread(target=check_timeout, args=(handler,), daemon=True).start()
    while True:
        if os.path.exists("/dev/input/js0"):
            if not connected:
                joystick_init()
                if pygame.joystick.get_count() > 0:
                    try:
                        js = pygame.joystick.Joystick(0)
                        js.init()
                        connected = True
                    except Exception as e:
                        print(e)
                else:
                    pygame.joystick.quit()
        else:
            if connected:
                connected = False
                js.quit()
                pygame.joystick.quit()

        if connected:
            pygame.event.pump()
            try:
                for button, index in key_map.items():
                    if js.get_button(index):
                        if not button_states[button]:
                            update_last_input_time()
                            getattr(handler, f"set_{button}_button")()
                            button_states[button] = True
                    else:
                        if button_states[button]:
                            handler.button_release()
                            button_states[button] = False

                lx1 = js.get_axis(0)
                ly1 = js.get_axis(1)
                if lx1 or ly1:
                    update_last_input_time()
                    handler.set_LEFT_STICK_button(lx1, ly1)

                lx2 = js.get_axis(2)
                ly2 = js.get_axis(3)
                if lx2 or ly2:
                    update_last_input_time()
                    handler.set_RIGHT_STICK_button(lx2, ly2)

                hat_x, hat_y = js.get_hat(0)
                if hat_x or hat_y:
                    hat_flag = True
                    if hat_x == 1:
                        update_last_input_time()
                        handler.set_Right_button()
                    elif hat_x == -1:
                        update_last_input_time()
                        handler.set_Left_button()
                    if hat_y == 1:
                        update_last_input_time()
                        handler.set_Up_button()
                    elif hat_y == -1:
                        update_last_input_time()
                        handler.set_Down_button()
                else:
                    if hat_flag:
                        handler.button_release()
                        hat_flag = False
            except Exception as e:
                print(e)
                connected = False
        time.sleep(0.01)

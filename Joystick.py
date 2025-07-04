import sys
import os
import time
import json
import pygame
import threading
import pod2 as pod

sys.path.append('/home/pi/SpiderPi/')

key_map = {"PSB_CROSS": 0, "PSB_CIRCLE": 1, "PSB_SQUARE": 3, "PSB_TRIANGLE": 4,
           "PSB_L1": 6, "PSB_R1": 7, "PSB_L2": 8, "PSB_R2": 9,
           "PSB_SELECT": 10, "PSB_START": 11}

last_input_time = time.time()
TIMEOUT = 2

def update_last_input_time():
    global last_input_time
    pod.pd.move_flag = True
    last_input_time = time.time()

def check_timeout():
    global last_input_time
    while True:
        if time.time() - last_input_time > TIMEOUT:
            pod.pd.move_flag = False
        time.sleep(1)

def joystick_init():
    os.environ["SDL_VIDEODRIVER"] = "dummy"
    pygame.display.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() > 0:
        js = pygame.joystick.Joystick(0)
        js.init()
        jsName = js.get_name()
        print("Name of the joystick:", jsName)
        jsAxes = js.get_numaxes()
        print("Number of axes:", jsAxes)
        jsButtons = js.get_numbuttons()
        print("Number of buttons:", jsButtons)
        jsBall = js.get_numballs()
        print("Number of balls:", jsBall)
        jsHat = js.get_numhats()
        print("Number of hats:", jsHat)

button_states = {key: False for key in key_map.keys()}
hat_flag = False

def get_joystick_data():
    global hat_flag
    connected = False
    while True:
        if os.path.exists("/dev/input/js0"):
            if not connected:
                joystick_init()
                jscount = pygame.joystick.get_count()
                if jscount > 0:
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
                            eval(f"pod.set_{button}_button()")
                            print(f"{button} pressed")
                            button_states[button] = True
                    else:
                        if button_states[button]:
                            pod.button_release()
                            print(f"{button} released")
                            button_states[button] = False

                lx1 = js.get_axis(0)
                ly1 = js.get_axis(1)

                if lx1 + ly1 != 0:  # 左摇杆
                    update_last_input_time()
                    pod.set_LEFT_STICK_button(lx1, ly1)
                    print(f"Left Stick: ({lx1},{ly1})")

                lx2 = js.get_axis(2)
                ly2 = js.get_axis(3)

                if lx2 + ly2 != 0:  # 右摇杆
                    update_last_input_time()
                    pod.set_RIGHT_STICK_button(lx2, ly2)
                    print(f"Right Stick: ({lx2},{ly2})")

                hat_x, hat_y = js.get_hat(0)

                if hat_x != 0 or hat_y != 0:
                    hat_flag = True
                    print(f"Hat Switch: ({hat_x}, {hat_y})")
                    if hat_x == 1:
                        update_last_input_time()
                        pod.set_Right_button()
                        print("Hat Switch: Right")
                    elif hat_x == -1:
                        update_last_input_time()
                        pod.set_Left_button()
                        print("Hat Switch: Left")
                    if hat_y == 1:
                        update_last_input_time()
                        pod.set_Up_button()
                        print("Hat Switch: Up")
                    elif hat_y == -1:
                        update_last_input_time()
                        pod.set_Down_button()
                        print("Hat Switch: Down")
                else:
                    if hat_flag:
                        pod.button_release()
                        print(f"hat released")
                        hat_flag = False

            except Exception as e:
                print(e)
                connected = False

        time.sleep(0.01)

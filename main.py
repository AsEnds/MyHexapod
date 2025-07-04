"""程序入口，启动各个线程。"""

import threading
import queue

from runtime.controller import HexapodController
from threads.input_thread import input_thread_loop
from threads.control_thread import control_thread_loop
from threads.vision_thread import vision_thread_loop
from gaits.tripod_gait import TripodGait
from pod2 import CommandHandler
import joystick


def main():
    """初始化控制器并启动线程"""
    controller = HexapodController()
    controller.gait = TripodGait()
    cmd_q = queue.Queue()

    handler = CommandHandler(controller)

    t_input = threading.Thread(target=input_thread_loop, args=(cmd_q,), daemon=True)
    t_control = threading.Thread(target=control_thread_loop, args=(controller, cmd_q), daemon=True)
    t_vision = threading.Thread(target=vision_thread_loop, daemon=True)
    t_js = threading.Thread(target=joystick.get_joystick_data, args=(handler,), daemon=True)

    t_input.start()
    t_control.start()
    t_vision.start()
    t_js.start()

    try:
        while t_control.is_alive():
            t_control.join(0.1)
    except KeyboardInterrupt:
        pass  # 支持 Ctrl+C 退出


if __name__ == "__main__":
    main()

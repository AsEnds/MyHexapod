import threading
import queue

from runtime.controller import HexapodController
from threads.input_thread import input_thread_loop
from threads.control_thread import control_thread_loop
from threads.vision_thread import vision_thread_loop
from gaits.tripod_gait import TripodGait


def main():
    controller = HexapodController()
    controller.gait = TripodGait()
    cmd_q = queue.Queue()

    t_input = threading.Thread(target=input_thread_loop, args=(cmd_q,), daemon=True)
    t_control = threading.Thread(target=control_thread_loop, args=(controller, cmd_q), daemon=True)
    t_vision = threading.Thread(target=vision_thread_loop, daemon=True)

    t_input.start()
    t_control.start()
    t_vision.start()

    try:
        while t_control.is_alive():
            t_control.join(0.1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

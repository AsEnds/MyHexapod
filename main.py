from controller import HexapodController
from pod2 import CommandHandler
import Joystick as js
import threading
import time

controller = HexapodController()
handler = CommandHandler(controller)
controller.start()

th1 = threading.Thread(target=js.get_joystick_data, args=(handler,))
th1.start()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Program terminated by user")

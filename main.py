import Joystick as js
import threading
import Hexapod as pd
import time

th1 = threading.Thread(target=js.get_joystick_data)
th2 = threading.Thread(target=pd.hexapod_move)

th1.start()
th2.start()

# 阻塞主线程，防止主线程退出
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Program terminated by user")

"""控制线程：从命令队列获取指令并驱动控制器。"""

import queue
import time

from runtime.commands import CommandType


def control_thread_loop(controller, cmd_q: queue.Queue):
    """主控制循环"""

    while True:
        try:
            cmd = cmd_q.get(timeout=0.01)
            if cmd is None:
                break
            if cmd.type == CommandType.SET_VELOCITY:
                controller.set_velocity(*cmd.args)
        except queue.Empty:
            pass
        controller.step()
        time.sleep(0.02)

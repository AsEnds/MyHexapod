"""输入线程：负责从命令行读取指令。"""

import queue
import time

from runtime.commands import Command, CommandType


def input_thread_loop(cmd_q: queue.Queue):
    """简单的键盘输入循环"""
    try:
        while True:
            line = input("cmd> ")  # 从终端读取指令
            if not line:
                continue
            if line.startswith("vel"):
                _, vx, vy, w = line.split()
                cmd_q.put(
                    Command(
                        CommandType.SET_VELOCITY,
                        (float(vx), float(vy), float(w)),
                    )
                )  # 将速度命令放入队列
            elif line.startswith("quit"):
                break  # 退出输入循环
    except KeyboardInterrupt:
        # Ctrl+C 终止
        pass
    finally:
        cmd_q.put(None)  # 通知控制线程退出

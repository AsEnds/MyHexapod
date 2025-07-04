import queue
import time

from runtime.commands import Command, CommandType


def input_thread_loop(cmd_q: queue.Queue):
    """Placeholder input loop reading from keyboard."""
    try:
        while True:
            line = input("cmd> ")
            if not line:
                continue
            if line.startswith("vel"):
                _, vx, vy, w = line.split()
                cmd_q.put(Command(CommandType.SET_VELOCITY, (float(vx), float(vy), float(w))))
            elif line.startswith("quit"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cmd_q.put(None)

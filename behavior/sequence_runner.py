"""执行预定义动作序列的辅助类。"""


class SequenceRunner:
    def __init__(self, plan):
        self.plan = plan

    def run(self):
        """按顺序产出每个目标点"""
        for pos in self.plan:
            yield pos

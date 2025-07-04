class SequenceRunner:
    def __init__(self, plan):
        self.plan = plan

    def run(self):
        for pos in self.plan:
            yield pos

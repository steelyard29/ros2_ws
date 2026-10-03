"""Bounded callback timing evidence, not a readiness gate or timeout override."""
import time


class ReaderTiming:
    def __init__(self, clock=time.monotonic, slow_s=.02, capacity=64):
        self.clock=clock
        self.slow_s=slow_s
        self.capacity=capacity
        self.report=dict(stages={},slow_calls=[],slow_calls_total=0,
                         note='Nested stage durations overlap; do not sum them.')

    def call(self, stage, function, *args, **kwargs):
        before=self.clock()
        try:
            return function(*args,**kwargs)
        finally:
            after=self.clock();elapsed=after-before
            row=self.report['stages'].setdefault(stage,dict(count=0,max_s=0.))
            row['count']+=1;row['max_s']=max(row['max_s'],elapsed)
            if elapsed>=self.slow_s:
                self.report['slow_calls_total']+=1
                events=self.report['slow_calls']
                events.append(dict(stage=stage,start=before,end=after,elapsed_s=elapsed))
                if len(events)>self.capacity:del events[0]

"""Bounded nonblocking JSON pipe writer; never drops/reorders sensor packets.

Queue exhaustion is an explicit session failure, not permission to replay old
poses or to renew their source timestamps. No threads, ROS, or device startup.
"""
from collections import deque
import json
import os
import time


class PacketWriter:
    def __init__(self, fd, capacity=65536, write=os.write, clock=time.monotonic):
        self.fd, self.capacity, self.write, self.clock = fd, capacity, write, clock
        self.queue = deque()
        self.offset = 0
        self.stats = dict(enqueued=0, sent=0, pending_bytes=0, peak_pending_bytes=0,
                          would_block=0, max_queue_residence_s=0.)

    def enqueue(self, packet):
        data = (json.dumps(packet, allow_nan=False)+'\n').encode()
        if self.stats['pending_bytes']+len(data) > self.capacity:
            raise BufferError('sensor transport queue exhausted; no packets dropped')
        self.queue.append((data, self.clock()))
        self.stats['enqueued'] += 1
        self.stats['pending_bytes'] += len(data)
        self.stats['peak_pending_bytes'] = max(self.stats['peak_pending_bytes'],
                                               self.stats['pending_bytes'])

    def pump(self, max_writes=8):
        # fd must be O_NONBLOCK. Bound work to leave time for ROS/stop/deadline.
        for _ in range(max_writes):
            if not self.queue:
                break
            data, enqueued = self.queue[0]
            try:
                count = self.write(self.fd, memoryview(data)[self.offset:])
            except (BlockingIOError, InterruptedError):
                self.stats['would_block'] += 1
                break
            if count <= 0:
                raise BrokenPipeError('sensor transport made no progress')
            self.offset += count
            self.stats['pending_bytes'] -= count
            if self.offset == len(data):
                self.queue.popleft()
                self.offset = 0
                self.stats['sent'] += 1
                self.stats['max_queue_residence_s'] = max(
                    self.stats['max_queue_residence_s'], self.clock()-enqueued)

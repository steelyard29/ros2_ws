"""Actual OS pipes plus deterministic short writes/backpressure; no hardware."""
import json
import os
from pathlib import Path
import sys
import time
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from disarmed_ev_transport import PacketWriter
from test_disarmed_ev_session import Rig


class TransportTests(unittest.TestCase):
    def test_partial_write_and_eagain_keep_exact_order(self):
        stream = bytearray(); calls = 0
        def write(fd, data):
            nonlocal calls
            calls += 1
            if calls % 4 == 0:
                raise BlockingIOError()
            count = min(7, len(data)); stream.extend(data[:count]); return count
        w = PacketWriter(-1, write=write)
        packets = [dict(seq=i, text='视觉') for i in range(40)]
        for p in packets:w.enqueue(p)
        for _ in range(1000):
            w.pump()
            if not w.queue:break
        self.assertFalse(w.queue)
        self.assertEqual([json.loads(x) for x in stream.splitlines()], packets)
        self.assertEqual(w.stats['sent'], 40)
        self.assertGreater(w.stats['would_block'], 0)
        self.assertEqual(w.stats['pending_bytes'], 0)

    def test_real_full_pipe_returns_without_blocking_and_later_delivers(self):
        read_fd, write_fd = os.pipe()
        try:
            os.set_blocking(read_fd, False); os.set_blocking(write_fd, False)
            while True:
                try:os.write(write_fd, b'x'*4096)
                except BlockingIOError:break
            w = PacketWriter(write_fd); w.enqueue(dict(seq=1))
            start = time.monotonic(); w.pump()
            self.assertLess(time.monotonic()-start, .1)
            self.assertEqual(w.stats['sent'], 0)
            self.assertEqual(w.stats['would_block'], 1)
            while True:
                try:os.read(read_fd, 65536)
                except BlockingIOError:break
            w.pump()
            self.assertEqual(json.loads(os.read(read_fd, 65536)), dict(seq=1))
        finally:os.close(read_fd);os.close(write_fd)

    def test_overflow_is_explicit_without_discarding_queued_packet(self):
        w = PacketWriter(-1, capacity=40)
        w.enqueue(dict(seq=1)); before=w.stats.copy()
        with self.assertRaises(BufferError):w.enqueue(dict(text='x'*50))
        self.assertEqual(w.stats, before)
        self.assertEqual(len(w.queue), 1)

    def test_broken_pipe_and_nonfinite_payload_fail(self):
        def broken(fd, data):raise BrokenPipeError()
        w = PacketWriter(-1, write=broken); w.enqueue(dict(seq=1))
        with self.assertRaises(BrokenPipeError):w.pump()
        with self.assertRaises(ValueError):w.enqueue(dict(x=float('nan')))

    def test_pump_work_is_bounded(self):
        w = PacketWriter(-1, write=lambda fd,data: 1)
        w.enqueue(dict(seq=1)); before=w.stats['pending_bytes']; w.pump(max_writes=3)
        self.assertEqual(w.stats['pending_bytes'], before-3)

    def test_trace_distinguishes_accepted_pose_and_true_source_hole(self):
        r=Rig(); r.ready()
        self.assertTrue(r.s.vision_trace[-1]['accepted_by_guard'])
        self.assertTrue(r.s.vision_trace[-1]['ev_candidate'])
        # Keep the existing source-gap boundary; do not hide the real failing gap.
        r.now += .30006958
        ns=int((100+r.now)*1e9)
        r.s.packet(r.packet('tracking',state=1),r.now,ns)
        r.s.packet(r.packet('pose',frame='odom',child='base_link',position=[0,0,0],
                            quaternion=[0,0,0,1]),r.now,ns)
        row=r.s.vision_trace[-1]
        self.assertFalse(row['accepted_by_guard'])
        self.assertAlmostEqual(row['source_gap_s'], .30006958, places=7)
        self.assertIn('VIO stale',row['fault'])
        count=r.s.sent; r.step();self.assertEqual(r.s.sent,count)


if __name__ == '__main__':unittest.main()

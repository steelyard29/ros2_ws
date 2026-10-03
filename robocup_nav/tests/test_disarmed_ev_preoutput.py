"""Actual discovery function with fake graph/time; network/process creation forbidden."""
import sys
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from disarmed_ev_session import wait_px4_graph


class PreoutputTests(unittest.TestCase):
    def run_probe(self,ready_at,*,duplicate=False,competing=False):
        state=NS(now=0.,closed=False)
        def count(topic):
            if topic.startswith('/fmu/in/'):return 1 if competing else 0
            if duplicate:return 2
            return 1 if state.now>=ready_at else 0
        node=NS(count_publishers=count,
            get_topic_names_and_types=lambda:[('/fmu/in/vehicle_visual_odometry',[])],
            destroy_node=lambda:setattr(state,'closed',True))
        def spin(n,timeout_sec):state.now=round(state.now+timeout_sec,8)
        ros=NS(create_node=lambda *a,**k:node,spin_once=spin)
        with patch.dict(sys.modules,{'rclpy':ros}),patch('socket.socket',side_effect=AssertionError('no network')):
            result=wait_px4_graph(10.,clock=lambda:state.now)
        self.assertTrue(state.closed)
        self.assertFalse(result['publisher_created'])
        return result,state

    def test_cold_topics_after_seven_seconds_wait_without_output(self):
        r,s=self.run_probe(7.)
        self.assertEqual(r['state'],'ready');self.assertGreaterEqual(s.now,7.)
        self.assertLess(s.now,10.)

    def test_hot_agent_does_not_add_fixed_sleep(self):
        r,s=self.run_probe(0.)
        self.assertEqual(r['state'],'ready');self.assertEqual(s.now,.05)

    def test_missing_or_exact_deadline_never_ready(self):
        for ready in (10.,20.):
            r,s=self.run_probe(ready)
            self.assertEqual(r['state'],'timeout');self.assertEqual(s.now,10.)

    def test_duplicates_fail_immediately(self):
        r,s=self.run_probe(0.,duplicate=True)
        self.assertEqual(r['state'],'conflict');self.assertEqual(s.now,.05)

    def test_foreign_flight_writer_fails_even_before_sources(self):
        r,s=self.run_probe(20.,competing=True)
        self.assertEqual(r['state'],'conflict');self.assertEqual(s.now,.05)


if __name__=='__main__':unittest.main()

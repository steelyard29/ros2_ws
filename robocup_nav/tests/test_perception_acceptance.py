"""Analytic timing controls; no ROS imports, discovery or hardware."""
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from perception_acceptance import AcceptanceWindow


class AcceptanceTests(unittest.TestCase):
    def feed(self,g,start,end,ready=True,skip=()):
        for i in range(round(start*30),round(end*30)+1):
            t=i/30
            for k in g.limits:
                if k not in skip and (k!='map' or i%6==0):
                    g.sample(k,t,1_000_000_000+round(t*1e9),.01)
            g.tick(t,ready)
        return g

    def measuring(self):
        g=self.feed(AcceptanceWindow(0,navigation=True),0,1.2)
        self.assertEqual(g.phase,'MEASURE')
        return g

    def test_startup_invalid_samples_preserved_without_pass(self):
        g=AcceptanceWindow(0,navigation=True)
        g.sample('map',.1,0,999.)
        g.sample('vio',.2,1_000_000_000,.53)
        self.assertEqual(g.tick(.3,False),'WAIT_INPUTS')
        self.feed(g,1,15)
        self.assertEqual(g.phase,'PASSED')
        self.assertGreaterEqual(len(g.events),3)
        self.assertFalse(g.report()['flight_ready'])

    def test_full_twelve_seconds_required(self):
        g=self.measuring()
        self.feed(g,1.233333,12.9)
        self.assertEqual(g.phase,'MEASURE')
        self.feed(g,12.933333,13.2)
        self.assertEqual(g.phase,'PASSED')
        self.assertGreaterEqual(g.finished-g.measure_started,12.)

    def test_missing_prerequisites_never_start(self):
        g=self.feed(AcceptanceWindow(0),0,18,False)
        self.assertEqual(g.fault,'startup deadline')
        self.assertIsNone(g.measure_started)

    def test_missing_map_never_start(self):
        g=self.feed(AcceptanceWindow(0,navigation=True),0,18,skip=('map',))
        self.assertEqual(g.phase,'FAILED')

    def test_readiness_cannot_win_at_expired_startup_deadline(self):
        g=AcceptanceWindow(0,startup_s=1.)
        self.feed(g,0,1.)
        self.assertEqual(g.phase,'FAILED')
        self.assertEqual(g.fault,'startup deadline')

    def test_duplicate_and_reversed_stamps_fail_latched(self):
        for delta in (0,-1):
            with self.subTest(delta=delta):
                g=self.measuring()
                g.sample('vio',1.21,g.last['vio'][1]+delta,.01)
                self.assertEqual(g.phase,'FAILED')
                reason=g.fault
                self.feed(g,1.233333,20)
                self.assertEqual(g.fault,reason)
                self.assertEqual(g.phase,'FAILED')

    def test_original_source_gap_is_not_relaxed(self):
        g=self.measuring()
        g.sample('vio',1.21,g.last['vio'][1]+300058350,.01)
        self.assertEqual(g.phase,'FAILED')

    def test_age_zero_stamp_nonfinite_fail(self):
        for key,stamp,age in [('vio',2_300_000_000,.251),('depth',2_300_000_000,.501),
                              ('map',0,.01),('map',2_300_000_000,.501),
                              ('vio',2_300_000_000,float('nan'))]:
            with self.subTest(key=key,age=age):
                g=self.measuring();g.sample(key,1.21,stamp,age)
                self.assertEqual(g.phase,'FAILED')

    def test_idle_without_callback_fails(self):
        g=self.measuring();g.tick(1.6,True)
        self.assertEqual(g.phase,'FAILED')

    def test_silent_depth_or_map_fails_despite_live_vio(self):
        for key in ('depth','map'):
            g=self.measuring();self.feed(g,1.233333,2,skip=(key,))
            self.assertEqual(g.phase,'FAILED')

    def test_lost_geometry_or_owner_latches(self):
        g=self.measuring();g.tick(1.21,False)
        self.feed(g,1.233333,15)
        self.assertEqual(g.phase,'FAILED')

    def test_startup_qualification_restarts_only_before_measurement(self):
        g=self.feed(AcceptanceWindow(0),0,.8)
        g.tick(.81,False)
        self.feed(g,.833333,1.6)
        self.assertEqual(g.phase,'WAIT_INPUTS')
        self.feed(g,1.633333,2.1)
        self.assertEqual(g.phase,'MEASURE')


class RolloutStaticTests(unittest.TestCase):
    def test_default_is_describe_without_process_or_ros(self):
        spec=importlib.util.spec_from_file_location('trial',ROOT/'scripts/perception_transport_trial.py')
        m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
        with patch.object(sys,'argv',['trial']),patch.object(m.subprocess,'Popen') as spawn:
            self.assertEqual(m.main(),0)
            spawn.assert_not_called()

    def test_stopped_mode_checks_conflicting_process_without_signals(self):
        spec=importlib.util.spec_from_file_location('trial',ROOT/'scripts/perception_transport_trial.py')
        m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
        with patch.object(m.Path,'glob',return_value=[Path('/proc/999/cmdline')]), \
                patch.object(m.Path,'read_bytes',return_value=b'ros2\0launch\0perception.launch.py'), \
                patch.object(m.os,'kill') as kill:
            with self.assertRaisesRegex(RuntimeError,'conflicting processes'):m.verify_stopped()
            kill.assert_not_called()


if __name__=='__main__':unittest.main()

import json
from pathlib import Path
import sys
import tempfile
import unittest
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from resident_status_binding import ResidentStatusBinding


class StatusTests(unittest.TestCase):
    def setUp(self):
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup)
        self.path=Path(tmp.name)/'status.json';self.now=100.
        self.data=dict(session='s',pid=123,process_identity={'start':4},updated_monotonic_s=100.,
            stopped=False,recent_dispatch=True,fault=None,output_topic='/fmu/in/vehicle_visual_odometry',
            source_domain=176,host_domain=0,sent_count=50,last_sent_stamp_us=100000000,
            ev_publisher_gids=[[9]*24],source_gids={k:[1]*24 for k in ('vio','tracking','status','flags','land')})
        self.write()
    def write(self):self.path.write_text(json.dumps(self.data))
    def bind(self):return ResidentStatusBinding(self.path,clock=lambda:self.now,inspect_process=lambda pid:{'start':4})

    def test_binding_does_not_fabricate_ev_receipt(self):
        binding=self.bind();self.assertEqual(binding.evidence.publisher_gid,tuple([9]*24))
        self.assertEqual(binding.evidence.count,0);self.assertFalse(binding.evidence.healthy(self.now))
        self.now+=.2;self.data.update(updated_monotonic_s=self.now,sent_count=56);self.write()
        binding.check();self.assertEqual(binding.evidence.count,0)

    def test_stale_status_latches_even_if_file_recovers(self):
        binding=self.bind();self.now+=.501
        with self.assertRaises(RuntimeError):binding.check()
        self.data['updated_monotonic_s']=self.now;self.write()
        with self.assertRaises(RuntimeError):binding.check()
        self.assertIsNotNone(binding.evidence.fault)

    def test_source_or_session_change_rejected(self):
        for key,value in (('session','new'),('ev_publisher_gids',[[8]*24]),('process_identity',{'start':5})):
            original=self.data[key];binding=self.bind();self.data[key]=value;self.write()
            with self.assertRaises(RuntimeError):binding.check()
            self.data[key]=original;self.write()

    def test_stopped_missing_or_invalid_gid_rejected(self):
        for key,value in (('stopped',True),('ev_publisher_gids',[]),('sent_count',0),('recent_dispatch',False)):
            original=self.data[key];self.data[key]=value;self.write()
            with self.assertRaises(RuntimeError):self.bind()
            self.data[key]=original;self.write()

    def test_counter_regression_rejected(self):
        binding=self.bind();self.data['sent_count']=49;self.write()
        with self.assertRaises(RuntimeError):binding.check()

    def test_live_graph_matches_all_required_sources(self):
        binding=self.bind()
        node=NS(get_publishers_info_by_topic=lambda topic:[NS(endpoint_gid=
            [9]*24 if topic.startswith('/fmu/in/') else [1]*24)])
        self.assertTrue(binding.verify_graph(node,node))

    def test_startup_absence_is_not_permission_or_latched_conflict(self):
        binding=self.bind();node=NS(get_publishers_info_by_topic=lambda topic:[])
        self.assertFalse(binding.verify_graph(node,node,allow_missing=True))
        self.assertIsNone(binding.fault)
        with self.assertRaises(RuntimeError):binding.verify_graph(node,node)

    def test_duplicate_is_rejected_even_during_discovery(self):
        binding=self.bind();node=NS(get_publishers_info_by_topic=lambda topic:
            [NS(endpoint_gid=[9]*24),NS(endpoint_gid=[9]*24)])
        with self.assertRaises(RuntimeError):binding.verify_graph(node,node,allow_missing=True)


if __name__=='__main__':unittest.main()

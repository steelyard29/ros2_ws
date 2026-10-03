"""No ROS, Docker, sensors or PX4: persistent input protocol and real OS pipes."""
import base64
import json
import os
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from resident_vio_transport import ResidentVioPort,ResidentVioPipe,MAX_PACKET


class TransportTests(unittest.TestCase):
    def setUp(self):
        self.now=100.;self.messages=[]
        self.r=ResidentVioPort('session',domain=183,isolated=True,
            clock=lambda:self.now,decode=lambda raw,kind:raw)
        self.topics=sorted(self.r.port.reads)
        for topic in self.topics:self.r.port.create_subscription(bytes,topic,self.messages.append,None)

    def packet(self,kind,**data):
        return dict(session='session',domain=183,seq=self.r.seq+1,at=self.now,kind=kind,**data)

    def graph(self):return self.packet('graph',graph={t:[[1]*24] for t in self.topics})

    def data(self):return self.packet('data',topic=self.topics[0],gid=[1]*24,
        cdr=base64.b64encode(b'original stamp and pose').decode())

    def test_persistent_delivery_beyond_bench_budget(self):
        for _ in range(601):
            self.r.accept(self.graph());self.r.accept(self.data());self.now+=.2
        self.assertEqual(len(self.messages),601)
        self.assertGreater(self.now,220.)
        self.assertIsNone(self.r.fault)
        self.assertEqual(set(self.messages),{b'original stamp and pose'})

    def test_source_change_latches_no_recovery(self):
        self.r.accept(self.graph())
        p=self.graph();p['graph'][self.topics[0]]=[[2]*24]
        with self.assertRaisesRegex(RuntimeError,'lost/replaced'):self.r.accept(p)
        with self.assertRaises(RuntimeError):self.r.accept(self.graph())
        self.assertEqual(self.messages,[])

    def test_stale_graph_fails_without_more_packets(self):
        self.r.accept(self.graph());self.now+=.501
        with self.assertRaisesRegex(RuntimeError,'graph stale'):
            self.r.port.get_publishers_info_by_topic(self.topics[0])

    def test_malformed_or_replayed_packet_latches(self):
        for field,value in (('session','other'),('domain',0),('seq',0),('at',90.),('at',float('nan'))):
            self.setUp();p=self.graph();p[field]=value
            with self.assertRaises(RuntimeError):self.r.accept(p)
            self.assertIsNotNone(self.r.fault)

    def test_no_data_before_discovery(self):
        with self.assertRaisesRegex(RuntimeError,'before source discovery'):self.r.accept(self.data())

    def test_duplicate_source_rejected(self):
        p=self.graph();p['graph'][self.topics[0]].append([2]*24)
        with self.assertRaisesRegex(RuntimeError,'duplicate'):self.r.accept(p)

    def test_real_pipe_fragmentation_then_eof(self):
        read,write=os.pipe()
        try:
            pipe=ResidentVioPipe(read,self.r)
            wire=(json.dumps(self.graph())+'\n').encode()
            os.write(write,wire[:9]);pipe.pump();self.assertEqual(self.r.seq,0)
            os.write(write,wire[9:]);pipe.pump();self.assertEqual(self.r.seq,1)
            os.write(write,(json.dumps(self.data())+'\n').encode());pipe.pump()
            self.assertEqual(len(self.messages),1)
            os.close(write);write=None
            with self.assertRaises(EOFError):pipe.pump()
            self.assertTrue(pipe.closed)
            with self.assertRaises(RuntimeError):pipe.pump()
        finally:
            os.close(read)
            if write is not None:os.close(write)

    def test_buffer_is_bounded(self):
        read,write=os.pipe()
        try:
            pipe=ResidentVioPipe(read,self.r);pipe.buffer=b'x'*MAX_PACKET
            os.write(write,b'x')
            with self.assertRaises(BufferError):pipe.pump()
            self.assertTrue(pipe.closed);self.assertEqual(pipe.buffer,b'')
        finally:os.close(read);os.close(write)


if __name__=='__main__':unittest.main()

"""Persistent, read-only VIO pipe receiver; no process/ROS/hardware startup.

The service owns the fd and pumps it alongside PX4 callbacks. Lifetime ends on
explicit close, EOF or a latched fault, not a bench-duration timer. Monotonic
packet times require host and container to share the same kernel time namespace.
Source timestamps remain inside CDR, unchanged. Graph identity is not per-message
authentication. Real deployment must still bind the service/session and owner.
"""
import base64
import json
import math
import os
import time
from types import SimpleNamespace as NS
from resident_vio_input import ScopedVioInput

MAX_PACKET=32768


class ResidentVioPort:
    def __init__(self,session,*,domain=176,isolated=False,clock=time.monotonic,decode=None):
        if not isinstance(session,str) or not session:raise ValueError('missing resident session')
        if domain not in (range(180,188) if isolated else (176,)):
            raise ValueError('invalid resident input domain')
        self.session,self.clock,self.decode=session,clock,decode
        self.context=NS(get_domain_id=lambda:domain)
        self.port=ScopedVioInput(self,isolated=isolated)
        self.callbacks={};self.graph={};self.identities={}
        self.seq=0;self.graph_at=None;self.fault=None;self.received=0

    def fail(self,reason):
        if self.fault is None:self.fault=str(reason)
        raise RuntimeError(self.fault)

    def create_subscription(self,kind,topic,callback,qos):
        if topic not in self.port.reads or topic in self.callbacks:
            raise ValueError('unapproved or repeated VIO subscription')
        self.callbacks[topic]=(kind,callback)
        return callback

    def get_publishers_info_by_topic(self,topic):
        if topic not in self.port.reads:raise ValueError('unapproved VIO graph query')
        if self.fault:raise RuntimeError(self.fault)
        if self.graph_at is None:return []
        if not 0<=self.clock()-self.graph_at<=.5:self.fail('resident source graph stale')
        return [NS(endpoint_gid=bytes(gid)) for gid in self.graph[topic]]

    def accept(self,p):
        if self.fault:raise RuntimeError(self.fault)
        try:self._accept(p)
        except Exception as exc:self.fail(exc)

    def _accept(self,p):
        age=self.clock()-p['at']
        if (p.get('session')!=self.session or p.get('domain')!=self.context.get_domain_id()
                or type(p.get('seq')) is not int or p['seq']!=self.seq+1
                or not math.isfinite(age) or not 0<=age<=.5):
            raise ValueError('resident packet session/domain/sequence/age mismatch')
        if p['kind']=='graph':
            graph=p['graph']
            if set(graph)!=self.port.reads:raise ValueError('resident graph topics mismatch')
            for topic,ends in graph.items():
                if not isinstance(ends,list) or len(ends)>1:raise ValueError('duplicate VIO source')
                for gid in ends:
                    if (not isinstance(gid,list) or len(gid)!=24
                            or any(type(x) is not int or not 0<=x<=255 for x in gid)):
                        raise ValueError('invalid VIO source identity')
                if topic in self.identities and ends!=[self.identities[topic]]:
                    raise ValueError('resident VIO source lost/replaced')
            for topic,ends in graph.items():
                if ends:self.identities[topic]=list(ends[0])
            self.graph={k:[list(g) for g in v] for k,v in graph.items()}
            self.graph_at=p['at']
        elif p['kind']=='data':
            topic=p['topic']
            if topic not in self.callbacks:raise ValueError('unregistered resident VIO input')
            if not self.get_publishers_info_by_topic(topic):raise ValueError('VIO data before source discovery')
            if p['gid']!=self.identities[topic]:raise ValueError('VIO packet source mismatch')
            if not isinstance(p['cdr'],str) or len(p['cdr'])>MAX_PACKET:
                raise ValueError('oversize resident CDR')
            raw=base64.b64decode(p['cdr'],validate=True)
            kind,callback=self.callbacks[topic]
            decoder=self.decode
            if decoder is None:
                from rclpy.serialization import deserialize_message
                decoder=deserialize_message
            callback(decoder(raw,kind))
            self.received+=1
        else:raise ValueError('unknown resident packet kind')
        self.seq=p['seq']


class ResidentVioPipe:
    def __init__(self,fd,receiver):
        self.fd,self.receiver=fd,receiver
        self.buffer=b'';self.closed=False
        # Only the caller-provided pipe fd; never opens a device or starts Docker.
        os.set_blocking(fd,False)

    def close(self,reason='resident VIO pipe closed'):
        self.closed=True;self.buffer=b''
        if self.receiver.fault is None:self.receiver.fault=reason
        # FD lifetime belongs to the service, not the task or this adapter.

    def pump(self,max_packets=16):
        if self.closed:raise RuntimeError(self.receiver.fault)
        try:
            for _ in range(max_packets):
                if b'\n' not in self.buffer:
                    try:chunk=os.read(self.fd,4096)
                    except (BlockingIOError,InterruptedError):return
                    if not chunk:raise EOFError('resident VIO pipe EOF')
                    self.buffer+=chunk
                    if len(self.buffer)>MAX_PACKET:raise BufferError('resident VIO packet/backlog limit')
                    if b'\n' not in self.buffer:continue
                line,self.buffer=self.buffer.split(b'\n',1)
                self.receiver.accept(json.loads(line))
        except Exception as exc:
            self.close(str(exc));raise

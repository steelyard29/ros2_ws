"""Persistent VIO pipe producer and isolated ROS factory; no real startup CLI.

Uses the receiver's exact wire format, unmodified CDR, and bounded nonblocking
writes. The owning service must supply lifecycle/stop handling and call pump.
No PX4 subscriptions, ROS publishers, hardware starts or automatic reconnection.
"""
import base64
import json
import os
import time
from resident_vio_transport import ResidentVioPort,MAX_PACKET
from disarmed_ev_transport import PacketWriter


class ResidentVioSender:
    def __init__(self,fd,session,*,domain=176,isolated=False,clock=time.monotonic,encode=None):
        # Reuse graph/session/domain validation without creating ROS endpoints.
        self.check=ResidentVioPort(session,domain=domain,isolated=isolated,clock=clock)
        self.clock,self.encode=clock,encode
        self.started=clock();self.seq=0;self.ready=False;self.fault=None
        self.input_timing={}
        os.set_blocking(fd,False)
        self.writer=PacketWriter(fd,capacity=MAX_PACKET,clock=clock)

    def stop(self,reason):
        if self.fault is None:self.fault=str(reason)
        self.ready=False
        self.writer.queue.clear();self.writer.offset=0
        self.writer.stats['pending_bytes']=0

    def emit(self,kind,**fields):
        if self.fault:raise RuntimeError(self.fault)
        packet=dict(session=self.check.session,domain=self.check.context.get_domain_id(),
                    seq=self.seq+1,at=self.clock(),kind=kind,**fields)
        if len((json.dumps(packet,allow_nan=False)+'\n').encode())>MAX_PACKET:
            raise BufferError('resident packet too large')
        self.writer.enqueue(packet);self.seq+=1

    def audit(self,graph):
        if self.fault:raise RuntimeError(self.fault)
        try:
            self.check.accept(dict(session=self.check.session,
                domain=self.check.context.get_domain_id(),seq=self.check.seq+1,
                at=self.clock(),kind='graph',graph=graph))
            self.ready=all(len(ends)==1 for ends in graph.values())
            if not self.ready and self.clock()-self.started>=10:
                raise TimeoutError('resident VIO discovery timeout')
            self.emit('graph',graph=graph)
        except Exception as exc:self.stop(exc);raise

    def receive(self,topic,msg):
        if self.fault:raise RuntimeError(self.fault)
        if not self.ready:return
        try:
            now=self.clock()
            item=self.input_timing.setdefault(topic,dict(count=0,last_at=None,max_gap_s=0.))
            if item['last_at'] is not None:
                item['max_gap_s']=max(item['max_gap_s'],now-item['last_at'])
            item['count']+=1;item['last_at']=now
            if not self.check.get_publishers_info_by_topic(topic):
                raise RuntimeError('resident VIO source missing')
            encoder=self.encode
            if encoder is None:
                from rclpy.serialization import serialize_message
                encoder=serialize_message
            raw=encoder(msg)
            self.emit('data',topic=topic,gid=self.check.identities[topic],
                      cdr=base64.b64encode(raw).decode('ascii'))
        except Exception as exc:self.stop(exc);raise

    def pump(self):
        if self.fault:raise RuntimeError(self.fault)
        try:
            if self.writer.queue and self.clock()-self.writer.queue[0][1]>.5:
                raise TimeoutError('resident writer queue stale')
            self.writer.pump()
        except Exception as exc:self.stop(exc);raise


def create_node(sender,*,isolated=False):
    if not isolated:raise ValueError('resident real reader lifecycle not integrated')
    if sender.check.context.get_domain_id() not in range(180,188):
        raise ValueError('resident reader requires isolated domain')
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String

    class Reader(Node):
        def __init__(self):
            super().__init__('resident_vio_pipe_reader',enable_rosout=False,
                start_parameter_services=False,use_global_arguments=False)
            if self.context.get_domain_id()!=sender.check.context.get_domain_id():
                self.destroy_node();raise ValueError('resident reader domain mismatch')
            for kind,key,qos in ((Odometry,'vio',qos_profile_sensor_data),(String,'tracking',10)):
                topic='/robocup/runtime_test/input/'+key
                self.create_subscription(kind,topic,lambda m,t=topic:sender.receive(t,m),qos)
            self.create_timer(.1,self.audit)

        def audit(self):
            try:
                graph={t:[list(end.endpoint_gid) for end in self.get_publishers_info_by_topic(t)]
                       for t in sender.check.port.reads}
                sender.audit(graph)
            except Exception as exc:sender.stop(exc);raise
    return Reader()

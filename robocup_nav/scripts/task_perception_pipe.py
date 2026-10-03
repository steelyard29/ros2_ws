"""Allowlisted container task input; default read-only, no flight commands.

CDR preserves the original messages. Depth transfers only the evidence used by
TaskSceneInput (header/dimensions/byte count), never fabricated pixel values.
Real navigation writes require a consumed dedicated task permit.
"""
import base64
import json
import math
import os
from pathlib import Path
import select
import subprocess
import time
from types import SimpleNamespace as NS
import uuid

ROOT=Path(__file__).resolve().parents[1]
MAX_PACKET=2*1024*1024


def specifications(isolated=False):
    from task_scene_input import TaskSceneInput
    from flight_runtime import topics
    runtime,_=topics(isolated)
    scene={k:('/robocup/task_test/input/'+k if isolated else v)
           for k,v in TaskSceneInput.TOPICS.items()}
    scene['vio']=runtime['vio']
    types=dict(vio='nav_msgs/msg/Odometry',depth='sensor_msgs/msg/Image',
        map='nvblox_msgs/msg/DistanceMapSlice',bounds='visualization_msgs/msg/Marker',
        h='std_msgs/msg/String',navigation='std_msgs/msg/String')
    reads={scene[k]:types[k] for k in scene}
    reads[runtime['tracking']]='std_msgs/msg/String'
    prefix='/robocup/task_test' if isolated else '/robocup/task'
    return reads,{prefix+'/navigation_goal',prefix+'/readiness'},scene['depth']


def depth_evidence(msg):
    return dict(sec=int(msg.header.stamp.sec),nanosec=int(msg.header.stamp.nanosec),
        frame_id=msg.header.frame_id,width=int(msg.width),height=int(msg.height),
        data_length=len(msg.data))


def restore_depth(data):
    for key in ('sec','nanosec','width','height','data_length'):
        if type(data.get(key)) is not int:raise ValueError('invalid depth metadata')
    if not (0<=data['nanosec']<10**9 and 0<=data['width']<=16384
            and 0<=data['height']<=16384 and 0<=data['data_length']<=64*1024*1024
            and isinstance(data['frame_id'],str)):
        raise ValueError('depth metadata bounds')
    # range has only a length here; there are no forwarded or synthetic pixels.
    return NS(header=NS(stamp=NS(sec=data['sec'],nanosec=data['nanosec']),frame_id=data['frame_id']),
        width=data['width'],height=data['height'],data=range(data['data_length']))


class ReadonlyPipePort:
    def __init__(self,session,*,isolated=False,clock=time.monotonic,navigation_writes=False,
                 live_permit=None,deadline=None):
        self.live_grant=None
        if live_permit is not None:
            if isolated or not navigation_writes:raise ValueError('task permit requires real navigation mode')
            from task_pipe_authority import grant_for
            self.live_grant=grant_for(live_permit,session,deadline)
        if navigation_writes and not isolated and self.live_grant is None:
            raise ValueError('real navigation requires consumed task permit')
        self.session,self.isolated,self.clock=session,isolated,clock
        self.reads,self.writes,self.depth_topic=specifications(isolated)
        self.context=NS(get_domain_id=lambda:183 if isolated else 176)
        self.callbacks={};self.graph={};self.identities={};self.graph_at=None
        self.seq=0;self.received={};self.max_pipe_age_s=0.;self.fault=None
        self.first_rejected=None
        self.navigation_writes=navigation_writes;self.command_enqueue=None
        self.command_seq=0;self.ack_seq=0;self.pending_commands={}
        self.discovery_only=False;self.activated_at=None;self.startup_discarded=0

    def create_subscription(self,kind,topic,callback,qos):
        if topic not in self.reads or kind.__name__!=self.reads[topic].split('/')[-1]:
            raise ValueError('unapproved task pipe subscription')
        self.callbacks.setdefault(topic,[]).append((kind,callback));return callback

    def create_publisher(self,kind,topic,qos):
        if not self.navigation_writes:
            raise RuntimeError('container task input port is read-only; no goal/control output')
        if topic not in self.writes or kind.__name__!='String':raise ValueError('unapproved navigation output')
        def publish(msg):
            if self.fault or self.command_enqueue is None or self.discovery_only:
                raise RuntimeError('navigation pipe unavailable')
            if self.graph_at is None or self.clock()-self.graph_at>.5 or self.count_publishers(topic)!=1:
                raise RuntimeError('navigation output source not ready')
            from task_navigation_pipe import MAX_REQUEST
            if not isinstance(msg.data,str) or len(msg.data.encode())>MAX_REQUEST:raise ValueError('oversize navigation request')
            self.command_seq+=1;now=self.clock()
            self.command_enqueue(dict(session=self.session,seq=self.command_seq,at=now,topic=topic,data=msg.data))
            self.pending_commands[self.command_seq]=now
        return NS(publish=publish)

    def get_clock(self):return NS(now=lambda:NS(nanoseconds=time.time_ns()))

    def get_publishers_info_by_topic(self,topic):
        if topic not in self.reads.keys()|self.writes:raise ValueError('unapproved graph query')
        return [NS(node_name=x['name'],node_namespace=x['namespace'],endpoint_gid=bytes(x['gid']))
                for x in self.graph.get(topic,[])]

    def count_publishers(self,topic):return len(self.get_publishers_info_by_topic(topic))

    def accept(self,packet):
        if self.fault:raise RuntimeError(self.fault)
        previous_seq=self.seq
        try:self._accept(packet)
        except Exception as exc:
            self.fault=str(exc)
            self.first_rejected=dict(reason=self.fault,seq=packet.get('seq'),
                kind=packet.get('kind'),topic=packet.get('topic'),last_accepted_seq=previous_seq)
            raise

    def _accept(self,p):
        now=self.clock()
        age=now-p['at']
        if (p.get('session')!=self.session or type(p.get('seq')) is not int or p['seq']!=self.seq+1
                or not math.isfinite(age) or age<0 or (age>.5 and not self.discovery_only)
                or p.get('domain')!=self.context.get_domain_id()):
            raise ValueError('task pipe session/sequence/domain/age mismatch')
        self.seq=p['seq'];self.max_pipe_age_s=max(self.max_pipe_age_s,age)
        if p['kind']=='graph':
            graph=p['graph']
            if set(graph)!=self.reads.keys()|self.writes:raise ValueError('graph topic mismatch')
            for topic,ends in graph.items():
                if len(ends)>1 or (topic in self.writes and ends and not self.navigation_writes):
                    raise ValueError('task pipe source/output competition: '+topic)
                if topic in self.writes and ends and ends[0]['name']!='robocup_task_pipe_reader':
                    raise ValueError('unexpected navigation writer')
                if topic in self.identities and (not ends or ends[0]['gid']!=self.identities[topic]):
                    raise ValueError('task pipe source missing/replaced: '+topic)
                for end in ends:
                    if (len(end['gid'])!=24 or any(type(x) is not int or not 0<=x<=255 for x in end['gid'])
                            or not isinstance(end['name'],str) or not isinstance(end['namespace'],str)):
                        raise ValueError('invalid endpoint identity')
                    self.identities[topic]=end['gid']
            self.graph=graph;self.graph_at=p['at'];return
        if p['kind']=='write_ack':
            seq=p['command_seq']
            if (not self.navigation_writes or seq!=self.ack_seq+1 or seq not in self.pending_commands
                    or p.get('topic') not in self.writes):
                raise ValueError('unexpected navigation acknowledgement')
            del self.pending_commands[seq];self.ack_seq=seq;return
        if p['kind']!='data' or p['topic'] not in self.reads:raise ValueError('unknown pipe topic/kind')
        topic=p['topic']
        if (self.graph_at is None or (now-self.graph_at>.5 and not self.discovery_only)
                or p['gid']!=self.identities.get(topic)):
            raise ValueError('data without fresh source audit')
        if self.discovery_only or (self.activated_at is not None and p['at']<self.activated_at):
            self.startup_discarded+=1
            return  # Explicit startup phase, not consumed as fresh task evidence.
        callbacks=self.callbacks.get(topic,[])
        if not callbacks:raise ValueError('unregistered task input')
        if topic==self.depth_topic:
            msg=restore_depth(p['depth'])
        else:
            from rclpy.serialization import deserialize_message
            raw=base64.b64decode(p['cdr'],validate=True)
            if len(raw)>MAX_PACKET:raise ValueError('oversize CDR')
            msg=deserialize_message(raw,callbacks[0][0])
        for _,callback in callbacks:callback(msg)
        self.received[topic]=self.received.get(topic,0)+1


class ContainerTaskInputs:
    def __init__(self,out,deadline,*,isolated=False,navigation_writes=False,live_permit=None,isolated_sortie=False):
        from task_domain_io import ScopedPerceptionNode
        self.out=Path(out);self.deadline=deadline;self.isolated=isolated
        self.session=uuid.uuid4().hex
        self.live_permit=live_permit
        from task_pipe_authority import reader_budget
        self.reader_limit=reader_budget(isolated=isolated,navigation_writes=navigation_writes,
                                       live=live_permit is not None,isolated_sortie=isolated_sortie)
        self.isolated_sortie=isolated_sortie
        self.proxy=ReadonlyPipePort(self.session,isolated=isolated,navigation_writes=navigation_writes,
                                    live_permit=live_permit,deadline=deadline)
        self.port=ScopedPerceptionNode(self.proxy,isolated=isolated)
        self.control=self.out/'task_reader.control.json';self.receipt=self.out/'task_reader.receipt.json'
        self.process=None;self.log=None;self.buffer=b'';self.cleanup=None
        self.command_writer=None
        self.quarantine_reason=None;self.stop_request_error=None

    def start(self,*,discovery_only=False):
        from disarmed_ev_lifecycle import atomic_json
        from task_pipe_authority import grant_for,READONLY_SECONDS,SORTIE_SECONDS
        limit=self.reader_limit
        if self.process is not None or not 0<self.deadline-time.monotonic()<=limit:
            raise ValueError('reader already started or invalid budget')
        grant=grant_for(self.live_permit,self.session,self.deadline) if self.live_permit is not None else None
        self.proxy.discovery_only=discovery_only
        atomic_json(self.control,dict(session=self.session,stop=False,task_authority=grant))
        root=Path('/workspaces/ros2_ws/robocup_nav')
        control=root/self.control.relative_to(ROOT);receipt=root/self.receipt.relative_to(ROOT)
        cmd=['docker','exec',*(['-i'] if self.proxy.navigation_writes else []),'isaac_ros_dev','timeout','--signal=TERM','--kill-after=1',str(int(limit)+1),
            'bash','-c','source /workspaces/ros2_ws/isaac_vio_debug/scripts/container_env.sh; exec "$@"',
            'task_input_reader','python3',str(root/'scripts/task_perception_reader.py'),
            '--session',self.session,'--deadline',str(self.deadline),
            '--control-file',str(control),'--receipt-file',str(receipt)]
        if self.isolated:cmd.append('--isolated')
        if self.isolated_sortie:cmd.append('--isolated-sortie')
        if self.proxy.navigation_writes:
            cmd.append('--authorized-task-navigation' if grant else '--isolated-navigation-writes')
        self.log=(self.out/'task_reader.stderr.log').open('w')
        self.process=subprocess.Popen(cmd,stdout=subprocess.PIPE,stderr=self.log,
            stdin=subprocess.PIPE if self.proxy.navigation_writes else subprocess.DEVNULL)
        os.set_blocking(self.process.stdout.fileno(),False)
        if self.proxy.navigation_writes:
            from disarmed_ev_transport import PacketWriter
            os.set_blocking(self.process.stdin.fileno(),False)
            self.command_writer=PacketWriter(self.process.stdin.fileno(),capacity=131072)
            self.proxy.command_enqueue=self.command_writer.enqueue

    def activate_inputs(self):
        if (not self.proxy.discovery_only or set(self.proxy.callbacks)!=set(self.proxy.reads)
                or any(not callbacks for callbacks in self.proxy.callbacks.values())
                or self.proxy.fault):
            raise RuntimeError('task input activation requires fresh discovery and complete callbacks')
        # Runtime construction can take longer than the graph freshness window.
        # Consume only discovery records/discarded data until a NEW audit arrives;
        # this does not relax any active input or consumer source-age threshold.
        end=min(self.deadline,time.monotonic()+.5)
        while self.proxy.graph_at is None or time.monotonic()-self.proxy.graph_at>.5:
            if time.monotonic()>=end:
                raise RuntimeError('task input activation discovery refresh timed out')
            self.pump()
        self.proxy.activated_at=time.monotonic();self.proxy.discovery_only=False

    def pump(self):
        if self.quarantine_reason is not None:
            raise RuntimeError('task reader quarantined; no replay')
        if self.process is None:raise RuntimeError('reader not started')
        if self.command_writer:self.command_writer.pump()
        end=time.monotonic()+.004
        while time.monotonic()<end:
            if b'\n' in self.buffer:
                line,self.buffer=self.buffer.split(b'\n',1)
                if len(line)>MAX_PACKET:raise BufferError('oversize task packet')
                self.proxy.accept(json.loads(line));continue
            if not select.select([self.process.stdout],[],[],0)[0]:break
            chunk=os.read(self.process.stdout.fileno(),65536)
            if not chunk:raise RuntimeError('container task reader exited')
            self.buffer+=chunk
            if len(self.buffer)>MAX_PACKET:raise BufferError('task input buffer exhausted')
        if (not self.proxy.discovery_only and self.proxy.graph_at is not None
                and time.monotonic()-self.proxy.graph_at>.5):
            raise RuntimeError('task reader graph stopped')
        if any(time.monotonic()-at>.5 for at in self.proxy.pending_commands.values()):
            raise RuntimeError('navigation request acknowledgement timeout; no replay')

    def quarantine(self,reason):
        """Non-waiting stop request; never hold up the independent PX4 loop."""
        if self.quarantine_reason is not None:return
        self.quarantine_reason=str(reason)
        self.proxy.command_enqueue=None
        self.command_writer=None  # Discard unsent requests; never flush on recovery.
        self.proxy.fault=self.proxy.fault or self.quarantine_reason
        if self.process is not None:
            from disarmed_ev_lifecycle import request_stop
            try:request_stop(self.control,self.session)
            except (OSError,ValueError) as exc:self.stop_request_error=str(exc)

    def report(self):
        return dict(received=self.proxy.received,packets=self.proxy.seq,fault=self.proxy.fault,
            first_rejected=self.proxy.first_rejected,
            source_counts={t:len(v) for t,v in self.proxy.graph.items()},
            source_audit='periodic endpoint graph; not per-message publisher identity',
            navigation_requests=self.proxy.command_seq,navigation_acks=self.proxy.ack_seq,
            navigation_pending=len(self.proxy.pending_commands),
            quarantine_reason=self.quarantine_reason,stop_request_error=self.stop_request_error,
            startup_discarded=self.proxy.startup_discarded,activated_at=self.proxy.activated_at,
            max_pipe_age_s=self.proxy.max_pipe_age_s,cleanup=self.cleanup,
            reader_exit_code=self.process.poll() if self.process else None)

    def close(self):
        from disarmed_ev_lifecycle import stop_reader
        self.proxy.command_enqueue=None
        try:
            if self.process:
                self.cleanup=stop_reader(self.process,self.control,self.receipt,self.session,timeout=2.)
                if not self.cleanup['confirmed'] or self.process.returncode!=0:
                    raise RuntimeError('task reader close not confirmed cleanly')
        finally:
            if self.process and self.process.poll() is not None:self.process.stdout.close()
            if self.process and self.process.stdin:self.process.stdin.close()
            if self.log:self.log.close()

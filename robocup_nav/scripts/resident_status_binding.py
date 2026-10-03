"""Task-side binding to one live resident status file, not flight authority.

Local process/session/graph provenance only, not authentication. Actual EV
receipt and PX4 fusion must still be checked separately. Never rebind/restart.
"""
import json
import math
from pathlib import Path
import time
from external_ev_evidence import ExternalEvEvidence
from resident_reader_lifecycle import clock_identity


def process_identity(pid):
    if type(pid) is not int or pid<=0:raise ValueError('invalid resident PID')
    # comm may contain spaces or parentheses: fields after the final ')' start
    # at stat field3; process starttime is field22.
    fields=Path(f'/proc/{pid}/stat').read_text().rpartition(')')[2].split()
    if fields[0]=='Z':raise ValueError('resident process is zombie')
    result=clock_identity()
    result['process_start_ticks']=int(fields[19])
    result['time_namespace']=Path(f'/proc/{pid}/ns/time').readlink().as_posix()
    return result


class ResidentStatusBinding:
    def __init__(self,path,*,clock=time.monotonic,inspect_process=process_identity):
        self.path=Path(path).resolve();self.clock=clock;self.inspect_process=inspect_process
        self.identity=None;self.last_count=None;self.last_stamp=None;self.last_update=None
        self.fault=None;self.evidence=None
        data=self.check()
        self.evidence=ExternalEvEvidence(data['session'],data['ev_publisher_gids'][0])

    def verify_graph(self,node,perception,*,allow_missing=False):
        """Check live endpoints, not merely a healthy status file."""
        self.check()
        expected=[(node,'/fmu/in/vehicle_visual_odometry',self.evidence.publisher_gid),
                  (perception,'/visual_slam/tracking/odometry',self.identity['source_gids']['vio'])]
        for key,topic in (('status','vehicle_status_v1'),('flags','estimator_status_flags'),
                          ('land','vehicle_land_detected')):
            expected.append((node,'/fmu/out/'+topic,self.identity['source_gids'][key]))
        try:
            missing=False
            for endpoint,topic,gid in expected:
                ends=endpoint.get_publishers_info_by_topic(topic)
                if not ends and allow_missing:
                    missing=True;continue
                if len(ends)!=1 or tuple(ends[0].endpoint_gid)!=tuple(gid):
                    raise ValueError('resident/task source differs: '+topic)
            return not missing
        except Exception as exc:
            self.fault=str(exc);self.evidence.stop(self.fault)
            raise RuntimeError(self.fault) from exc

    def check(self):
        if self.fault:raise RuntimeError(self.fault)
        try:
            data=json.loads(self.path.read_text())
            age=self.clock()-data['updated_monotonic_s']
            gids=data['ev_publisher_gids'];sources=data['source_gids']
            if (not math.isfinite(age) or not 0<=age<=.5 or data['stopped'] is not False
                    or data['recent_dispatch'] is not True or data.get('fault') is not None
                    or data['output_topic']!='/fmu/in/vehicle_visual_odometry'
                    or data['source_domain']!=176 or data['host_domain']!=0
                    or not isinstance(data['session'],str) or not data['session']
                    or type(data['sent_count']) is not int or data['sent_count']<1
                    or type(data['last_sent_stamp_us']) is not int or data['last_sent_stamp_us']<=0
                    or not isinstance(gids,list) or len(gids)!=1
                    or set(sources)!={'vio','tracking','status','flags','land'}):
                raise ValueError('resident status unhealthy/schema mismatch')
            for gid in gids+list(sources.values()):
                if not isinstance(gid,list) or len(gid)!=24 or any(type(x) is not int or not 0<=x<=255 for x in gid):
                    raise ValueError('invalid resident source GID')
            if self.inspect_process(data['pid'])!=data['process_identity']:
                raise ValueError('resident process replaced or clock namespace differs')
            identity={key:data[key] for key in ('session','pid','process_identity','ev_publisher_gids','source_gids')}
            if self.identity is not None and identity!=self.identity:
                raise ValueError('resident session/source changed')
            if self.last_count is not None and (data['sent_count']<self.last_count
                    or data['last_sent_stamp_us']<self.last_stamp
                    or data['updated_monotonic_s']<self.last_update):
                raise ValueError('resident status counters regressed')
            self.identity=identity;self.last_count=data['sent_count']
            self.last_stamp=data['last_sent_stamp_us'];self.last_update=data['updated_monotonic_s']
            return data
        except Exception as exc:
            self.fault=str(exc)
            if self.evidence:self.evidence.stop('resident binding lost: '+self.fault)
            raise RuntimeError(self.fault) from exc
